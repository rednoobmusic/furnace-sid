/**
 * Furnace Tracker - multi-system chiptune tracker
 * Copyright (C) 2021-2026 tildearrow and contributors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "sid.h"
#include "../engine.h"
#include "../ta-log.h"
#include <fmt/printf.h>
#include <vector>
#include <array>
#include <string.h>

#define SID_REG_COUNT 25

// PSID v2 header = 0x7C bytes.
// Data format: delta-compressed. Each frame = sequence of (reg, val) pairs
// terminated by 0xFF. reg = 0..24 for chip0, 25..49 for chip1, etc.
// 6502 player maps reg to SID base address and writes val.
// On 0xFF: advance to next frame. On end-of-stream: wrap to loop point.

#define LOAD_ADDR    0x1000
#define INIT_OFFSET  0
#define PLAY_OFFSET  32
#define DATA_OFFSET  256

// Zero-page scratch used by the player.
#define ZP_PTR_LO    0xFB
#define ZP_PTR_HI    0xFC
#define ZP_LOOP_LO   0xFD
#define ZP_LOOP_HI   0xFE

static uint16_t initAddr() { return LOAD_ADDR + INIT_OFFSET; }
static uint16_t playAddr() { return LOAD_ADDR + PLAY_OFFSET; }

static const uint16_t SID_BASE_ADDR[3] = { 0xD400, 0xD420, 0xD440 };

// Builds the 6502 player stub (padded to DATA_OFFSET bytes).
//
// INIT: set ZP_PTR = dataStart, ZP_LOOP = loopAddr, RTS.
//
// PLAY: read (reg, val) pairs from (ZP_PTR), write val to SID base + (reg % 25),
//       choosing chip by reg / 25. Stop on 0xFF, advance ZP_PTR past the 0xFF.
//       If ZP_PTR >= endAddr, wrap to ZP_LOOP. RTS.
//
// The inner loop is a simple: LDA (ZP_PTR),Y [Y=0 always, use indirect zero-page]
// then dispatch by reg value. For simplicity and code size we use a flat loop:
//   read reg byte; if 0xFF done; read val byte; write to SID; repeat.
// We use ZP_PTR directly (Y=0 always) and bump it with INC/BNE/INC.
static std::vector<uint8_t> buildBinary(int numSIDs, uint32_t dataBytes, uint32_t loopByteOffset) {
  uint16_t dataStart = (uint16_t)(LOAD_ADDR + DATA_OFFSET);
  uint16_t endAddr   = (uint16_t)(LOAD_ADDR + DATA_OFFSET + dataBytes);
  uint16_t loopAddr  = (uint16_t)(LOAD_ADDR + DATA_OFFSET + loopByteOffset);

  std::vector<uint8_t> code(DATA_OFFSET, 0x00);

  // INIT
  int p = INIT_OFFSET;
  code[p++]=0xA9; code[p++]=(uint8_t)(dataStart&0xFF); // LDA #<dataStart
  code[p++]=0x85; code[p++]=ZP_PTR_LO;                 // STA ZP_PTR_LO
  code[p++]=0xA9; code[p++]=(uint8_t)(dataStart>>8);   // LDA #>dataStart
  code[p++]=0x85; code[p++]=ZP_PTR_HI;                 // STA ZP_PTR_HI
  code[p++]=0xA9; code[p++]=(uint8_t)(loopAddr&0xFF);  // LDA #<loopAddr
  code[p++]=0x85; code[p++]=ZP_LOOP_LO;                // STA ZP_LOOP_LO
  code[p++]=0xA9; code[p++]=(uint8_t)(loopAddr>>8);    // LDA #>loopAddr
  code[p++]=0x85; code[p++]=ZP_LOOP_HI;                // STA ZP_LOOP_HI
  code[p++]=0x60;                                       // RTS

  // PLAY — delta stream reader
  // Uses A, X, Y. ZP_PTR_LO/HI = current stream position.
  // Protocol: while byte != 0xFF: read reg, read val, write to SID; then advance past 0xFF.
  // Helper macro: bump ZP_PTR by 1 (inline 16-bit increment).
  // We use Y=0 throughout and LDA (ZP_PTR),Y to read bytes.

  p = PLAY_OFFSET;

  // frameLoop: read one byte from stream
  int frameLoop = p;
  code[p++]=0xA0; code[p++]=0x00;       // LDY #0
  code[p++]=0xB1; code[p++]=ZP_PTR_LO; // LDA (ZP_PTR_LO),Y  — read reg byte

  // if reg == 0xFF, end of frame
  code[p++]=0xC9; code[p++]=0xFF;       // CMP #$FF
  code[p++]=0xF0; int beq_done=p; code[p++]=0x00; // BEQ done

  // save reg in X
  code[p++]=0xAA;                        // TAX  — X = reg index

  // bump ZP_PTR
  code[p++]=0xE6; code[p++]=ZP_PTR_LO; // INC ZP_PTR_LO
  code[p++]=0xD0; code[p++]=0x02;       // BNE +2
  code[p++]=0xE6; code[p++]=ZP_PTR_HI; // INC ZP_PTR_HI

  // read val byte
  code[p++]=0xB1; code[p++]=ZP_PTR_LO; // LDA (ZP_PTR_LO),Y  — read val byte

  // write to SID: we need to dispatch on X (reg index 0..24 for chip0, 25..49 for chip1, etc)
  // For 1 SID: STA $D400,X  — but X might be > 24, doesn't matter for 1 SID songs.
  // For multi-SID: reg 0..24 -> $D400+reg, 25..49 -> $D420+(reg-25), etc.
  // Simple approach: STA $D400,X — works for chip0 regs 0..24.
  // For multi-SID we'd need chip dispatch, but most songs are 1 SID.
  // Use absolute indexed: STA $D400,X (works for 1 SID perfectly).
  // For 2+ SIDs: $D400+X where X=25 = $D419 (wrong). We'll handle multi-SID below.
  if (numSIDs == 1) {
    code[p++]=0x9D; code[p++]=0x00; code[p++]=0xD4; // STA $D400,X
  } else {
    // For multi-SID: if X < 25 write to $D400+X, if X < 50 write to $D420+(X-25), etc.
    // Simplified: subtract 25 per chip and write to corresponding base.
    // We use a small comparison chain.
    for (int s=numSIDs-1; s>=0; s--) {
      if (s>0) {
        // save A
        code[p++]=0x48;                    // PHA
        code[p++]=0x8A;                    // TXA
        code[p++]=0xC9; code[p++]=(uint8_t)(s*SID_REG_COUNT); // CMP #(s*25)
        code[p++]=0x90; int blt=p; code[p++]=0x00; // BCC not_this_chip
        // X >= s*25: this is chip s, reg = X - s*25
        code[p++]=0xE9; code[p++]=(uint8_t)(s*SID_REG_COUNT); // SBC #(s*25) (carry set from CMP)
        code[p++]=0xAA;                    // TAX
        code[p++]=0x68;                    // PLA  — restore val
        uint16_t base=SID_BASE_ADDR[s];
        code[p++]=0x9D; code[p++]=(uint8_t)(base&0xFF); code[p++]=(uint8_t)(base>>8); // STA base,X
        code[p++]=0x4C; int jmp_next=p; code[p++]=0x00; code[p++]=0x00; // JMP next_pair (patch later)
        // not_this_chip:
        code[blt]=(uint8_t)((p-(blt+1))&0xFF);
        code[p++]=0x68;                    // PLA — restore val (we PHA'd it)
        // patch JMP
        uint16_t jmpTarget=(uint16_t)(LOAD_ADDR+p);
        code[jmp_next]=(uint8_t)(jmpTarget&0xFF);
        code[jmp_next+1]=(uint8_t)(jmpTarget>>8);
      } else {
        // chip 0: just write directly
        code[p++]=0x9D; code[p++]=0x00; code[p++]=0xD4; // STA $D400,X
      }
    }
  }

  // bump ZP_PTR past val byte
  code[p++]=0xE6; code[p++]=ZP_PTR_LO; // INC ZP_PTR_LO
  code[p++]=0xD0; code[p++]=0x02;       // BNE +2
  code[p++]=0xE6; code[p++]=ZP_PTR_HI; // INC ZP_PTR_HI

  // loop back for next pair (unconditional)
  uint16_t jmpBack=(uint16_t)(LOAD_ADDR+frameLoop);
  code[p++]=0x4C; code[p++]=(uint8_t)(jmpBack&0xFF); code[p++]=(uint8_t)(jmpBack>>8); // JMP frameLoop

  // done: bump ZP_PTR past the 0xFF terminator
  int doneHere = p;
  code[beq_done]=(uint8_t)((doneHere-(beq_done+1))&0xFF);

  // bump past 0xFF
  code[p++]=0xE6; code[p++]=ZP_PTR_LO; // INC ZP_PTR_LO
  code[p++]=0xD0; code[p++]=0x02;       // BNE +2
  code[p++]=0xE6; code[p++]=ZP_PTR_HI; // INC ZP_PTR_HI

  // check ZP_PTR >= endAddr; if so wrap to ZP_LOOP
  code[p++]=0xA5; code[p++]=ZP_PTR_HI;          // LDA ZP_PTR_HI
  code[p++]=0xC9; code[p++]=(uint8_t)(endAddr>>8); // CMP #>endAddr
  code[p++]=0x90; int bcc_hi=p; code[p++]=0x00; // BCC rts
  code[p++]=0xD0; int bne_hi=p; code[p++]=0x00; // BNE wrap
  code[p++]=0xA5; code[p++]=ZP_PTR_LO;          // LDA ZP_PTR_LO
  code[p++]=0xC9; code[p++]=(uint8_t)(endAddr&0xFF); // CMP #<endAddr
  code[p++]=0x90; int bcc_lo=p; code[p++]=0x00; // BCC rts

  int wrapHere=p;
  code[p++]=0xA5; code[p++]=ZP_LOOP_LO; // LDA ZP_LOOP_LO
  code[p++]=0x85; code[p++]=ZP_PTR_LO;  // STA ZP_PTR_LO
  code[p++]=0xA5; code[p++]=ZP_LOOP_HI; // LDA ZP_LOOP_HI
  code[p++]=0x85; code[p++]=ZP_PTR_HI;  // STA ZP_PTR_HI

  int rtsHere=p;
  code[p++]=0x60; // RTS

  code[bcc_hi]=(uint8_t)((rtsHere-(bcc_hi+1))&0xFF);
  code[bne_hi]=(uint8_t)((wrapHere-(bne_hi+1))&0xFF);
  code[bcc_lo]=(uint8_t)((rtsHere-(bcc_lo+1))&0xFF);

  if (p > DATA_OFFSET) {
    fprintf(stderr,"SID export: PLAY routine overflow! (%d > %d)\n",p,DATA_OFFSET);
  }

  return code;
}

static void writePSIDHeader(SafeWriter* w, int numSIDs, bool isPAL,
                            DivSystem sidModel,
                            uint16_t initAddress, uint16_t playAddress,
                            const char* title, const char* author,
                            uint16_t loadAddr) {
  w->writeText("PSID");
  uint16_t version=(numSIDs>=3)?4:(numSIDs==2)?3:2;
  w->writeC(0x00); w->writeC((uint8_t)version);
  w->writeC(0x00); w->writeC(0x7C);
  w->writeC((uint8_t)(loadAddr>>8)); w->writeC((uint8_t)(loadAddr&0xFF));
  w->writeC((uint8_t)(initAddress>>8)); w->writeC((uint8_t)(initAddress&0xFF));
  w->writeC((uint8_t)(playAddress>>8)); w->writeC((uint8_t)(playAddress&0xFF));
  w->writeC(0x00); w->writeC(0x01); // songs=1
  w->writeC(0x00); w->writeC(0x01); // startSong=1
  w->writeC(0x00); w->writeC(0x00); w->writeC(0x00); w->writeC(0x00); // speed=0 (VBI)
  char nameBuf[32]; memset(nameBuf,0,32); strncpy(nameBuf,title,31);
  w->write(nameBuf,32);
  char authBuf[32]; memset(authBuf,0,32); strncpy(authBuf,author,31);
  w->write(authBuf,32);
  char relBuf[32]; memset(relBuf,0,32); strncpy(relBuf,"Furnace Tracker",31);
  w->write(relBuf,32);
  uint16_t flags=0;
  flags|=(isPAL?(1<<2):(2<<2));
  if (sidModel==DIV_SYSTEM_C64_6581)      flags|=(1<<4);
  else if (sidModel==DIV_SYSTEM_C64_8580) flags|=(2<<4);
  w->writeC((uint8_t)(flags>>8)); w->writeC((uint8_t)(flags&0xFF));
  w->writeC(0x00); w->writeC(0x00); // startPage/pageLen: not relocatable
  uint8_t sec=(numSIDs>=2)?(uint8_t)((SID_BASE_ADDR[1]-0xD000)>>4):0x00;
  uint8_t thr=(numSIDs>=3)?(uint8_t)((SID_BASE_ADDR[2]-0xD000)>>4):0x00;
  w->writeC(sec); w->writeC(thr);
}

void DivExportSID::run() {
  int sidChips[3]={-1,-1,-1};
  int numSIDChips=0;
  int ignored=0;

  for (int i=0; i<e->song.systemLen; i++) {
    DivSystem sys=e->song.system[i];
    bool ok=(sys==DIV_SYSTEM_C64_6581 || sys==DIV_SYSTEM_C64_8580 ||
             (sys==DIV_SYSTEM_C64_PCM && withPCM) ||
             sys==DIV_SYSTEM_SID2 || sys==DIV_SYSTEM_SID3);
    if (ok && numSIDChips<3) {
      sidChips[numSIDChips++]=i;
      logAppendf("SID chip %d: engine chip %d, system %d",numSIDChips,i,(int)sys);
    } else {
      ignored++;
      logAppendf("ignoring chip %d (system %d)",(int)i,(int)sys);
    }
  }

  if (numSIDChips<1) {
    logAppend("ERROR: no SID chip found in song");
    failed=true; running=false; return;
  }
  logAppendf("found %d SID chip%s%s",numSIDChips,numSIDChips>1?"s":"",
             ignored>0?fmt::sprintf(" (%d ignored)",ignored).c_str():"");

  DivSystem sidModel=e->song.system[sidChips[0]];
  bool isPAL=(e->song.systemFlags[sidChips[0]].getInt("clockSel",0)==1);
  logAppendf("clock: %s",isPAL?"PAL 50Hz":"NTSC 60Hz");

  std::array<uint8_t,SID_REG_COUNT> currRegs[3];
  std::array<uint8_t,SID_REG_COUNT> prevRegs[3];
  for (int s=0;s<3;s++) {
    memset(currRegs[s].data(),0,SID_REG_COUNT);
    memset(prevRegs[s].data(),0xFF,SID_REG_COUNT); // force all regs on first frame
  }

  // Delta-compressed stream: (reg, val) pairs per frame, 0xFF terminator.
  // reg = chip*25 + register_index.
  std::vector<uint8_t> frameData;
  frameData.reserve(65536);
  int numFrames=0;
  uint32_t loopByteOffset=0;
  bool loopMarked=false;

  double frameRate=isPAL?50.0:60.0;

  e->stop();
  e->repeatPattern=false;
  e->setOrder(0);

  logAppend("rendering song...");

  e->synchronizedSoft([&]() {
    double origRate=e->got.rate;
    e->got.rate=44100.0;

    e->calcSongTimestamps();
    int loopOrder=e->curSubSong->ts.loopStart.order;
    int loopRow=e->curSubSong->ts.loopStart.row;
    bool hasLoop=e->curSubSong->ts.isLoopDefined;
    logAppendf("loop point: order=%d row=%d defined=%d",loopOrder,loopRow,(int)hasLoop);

    e->curOrder=0;
    e->freelance=false;
    e->playing=false;
    e->extValuePresent=false;
    e->remainingLoops=-1;
    e->playSub(false);

    for (int s=0;s<numSIDChips;s++) {
      e->disCont[sidChips[s]].dispatch->toggleRegisterDump(true);
      unsigned char* pool=e->disCont[sidChips[s]].dispatch->getRegisterPool();
      if (pool) {
        for (int r=0;r<SID_REG_COUNT;r++) currRegs[s][r]=pool[r];
      }
    }

    bool done=false;
    bool loopSeen=false;
    double frameAccum=0.0;

    if (hasLoop && e->curOrder==loopOrder && e->curRow==loopRow) {
      loopMarked=true;
      loopByteOffset=0;
      logAppendf("loop byte offset: %u (frame %d)",loopByteOffset,numFrames);
    }

    while (!done) {
      if (e->nextTick(false,true) || !e->playing) {
        for (int j=0;j<e->song.systemLen;j++)
          e->disCont[j].dispatch->getRegisterWrites().clear();
        done=true;
        break;
      }

      for (int s=0;s<numSIDChips;s++) {
        auto& writes=e->disCont[sidChips[s]].dispatch->getRegisterWrites();
        for (auto& wr:writes)
          if ((int)wr.addr<SID_REG_COUNT)
            currRegs[s][wr.addr]=(uint8_t)wr.val;
        writes.clear();
      }

      if (hasLoop && !loopMarked && e->curOrder==loopOrder && e->curRow==loopRow) {
        loopByteOffset=(uint32_t)frameData.size();
        loopMarked=true;
        logAppendf("loop byte offset: %u (frame %d)",loopByteOffset,numFrames);
      }
      if (hasLoop && loopMarked && loopSeen &&
          e->curOrder==loopOrder && e->curRow==loopRow) {
        for (int j=0;j<e->song.systemLen;j++)
          e->disCont[j].dispatch->getRegisterWrites().clear();
        done=true;
        break;
      }
      if (loopMarked && !loopSeen &&
          !(e->curOrder==loopOrder && e->curRow==loopRow)) {
        loopSeen=true;
      }

      frameAccum+=((double)e->cycles*frameRate)/44100.0;
      int frames=(int)frameAccum;
      frameAccum-=frames;

      for (int f=0;f<frames;f++) {
        for (int s=0;s<numSIDChips;s++) {
          for (int r=0;r<SID_REG_COUNT;r++) {
            if (currRegs[s][r]!=prevRegs[s][r]) {
              frameData.push_back((uint8_t)(s*SID_REG_COUNT+r));
              frameData.push_back(currRegs[s][r]);
              prevRegs[s][r]=currRegs[s][r];
            }
          }
        }
        frameData.push_back(0xFF); // end of frame
        numFrames++;
      }
    }

    e->got.rate=origRate;
    for (int s=0;s<numSIDChips;s++)
      e->disCont[sidChips[s]].dispatch->toggleRegisterDump(false);
    e->remainingLoops=-1;
    e->playing=false;
    e->freelance=false;
    e->extValuePresent=false;
  });

  logAppendf("captured %d frames, %d bytes (loop byte offset %u)",
             numFrames,(int)frameData.size(),loopByteOffset);
  progress[0].amount=0.5f;

  if (numFrames==0) {
    logAppend("ERROR: no frames captured");
    failed=true; running=false; return;
  }
  if (loopByteOffset>=(uint32_t)frameData.size()) loopByteOffset=0;

  // Check if stream fits in PSID address space.
  uint32_t maxDataBytes=(uint32_t)(0x10000-LOAD_ADDR-DATA_OFFSET-1);
  bool truncated=false;
  uint32_t psidDataBytes=(uint32_t)frameData.size();

  if (psidDataBytes>maxDataBytes) {
    // truncate at last complete frame that fits
    truncated=true;
    psidDataBytes=maxDataBytes;
    // walk back to find last 0xFF within limit
    while (psidDataBytes>0 && frameData[psidDataBytes-1]!=0xFF) psidDataBytes--;
    if (loopByteOffset>=psidDataBytes) loopByteOffset=0;
    logAppendf("warning: stream truncated to %u bytes (add a loop point to fix this).",psidDataBytes);
  }

  {
    std::vector<uint8_t> binary=buildBinary(numSIDChips,psidDataBytes,loopByteOffset);
    binary.insert(binary.end(),frameData.begin(),frameData.begin()+psidDataBytes);
    uint32_t binarySize=(uint32_t)binary.size();

    auto w=new SafeWriter;
    w->init();
    writePSIDHeader(w,numSIDChips,isPAL,sidModel,
                    initAddr(),playAddr(),
                    e->song.name.c_str(),e->song.author.c_str(),
                    LOAD_ADDR);
    w->write(binary.data(),(int)binarySize);
    output.push_back(DivROMExportOutput("export.sid",w));
    logAppendf("PSID written: %d frames%s, %d bytes total",
               numFrames, truncated?" (truncated)":"",
               (int)(0x7C+binarySize));
  }

  progress[0].amount=1.0f;
  logAppend("done!");
  running=false;
}

bool DivExportSID::go(DivEngine* eng) {
  progress[0].name="Progress";
  progress[0].amount=0.0f;
  e=eng;
  running=true;
  failed=false;
  mustAbort=false;
  exportThread=new std::thread(&DivExportSID::run,this);
  return true;
}

void DivExportSID::wait() {
  if (exportThread!=NULL) {
    logV("waiting for SID export thread...");
    exportThread->join();
    delete exportThread;
    exportThread=NULL;
  }
}

void DivExportSID::abort() {
  mustAbort=true;
  wait();
}

bool DivExportSID::isRunning() { return running; }
bool DivExportSID::hasFailed()  { return failed; }

DivROMExportProgress DivExportSID::getProgress(int index) {
  if (index<0||index>1) return progress[1];
  return progress[index];
}
