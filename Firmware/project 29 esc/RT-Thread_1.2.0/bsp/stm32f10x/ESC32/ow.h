/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#ifndef _OW_H
#define _OW_H

#include "misc.h"

#define OW_RESET_MIN	    480
#define OW_RESET_MAX	    580

#define OW_ROM_READ	    0x33    //读ROM
#define OW_ROM_MATCH	    0x55	    // not yet supported  匹配ROM
#define OW_ROM_SKIP	    0xCC  //跳过ROM
#define OW_VERSION	    0x03
#define OW_PARAM_READ	    0x04
#define OW_PARAM_WRITE	    0x05
#define OW_CONFIG_READ	    0x06
#define OW_CONFIG_WRITE	    0x07
#define OW_CONFIG_DEFAULT   0x08
#define OW_SET_MODE	    0x09
#define OW_GET_MODE	    0x0A

#define OW_UID_ADDRESS	    0x1FFFF7E8

enum {
    OW_RESET_STATE_0,
    OW_RESET_STATE_1,
    OW_RESET_STATE_2,
    OW_READ,
    OW_WRITE
};

extern uint8_t owROMCode[8];
extern uint8_t owBuf[16];
extern uint8_t *owBufPointer;
extern uint8_t owState;
extern uint8_t owLastCommand;
extern uint8_t owByteValue;
extern uint8_t owReadNum;
extern uint8_t owWriteNum;
extern uint8_t owBit;

extern void owInit(void);
extern void owReset(void);
extern void owEdgeDetect(uint8_t edge);
extern void owResetIsr(int state);
extern void owReadBitIsr(int unused);
extern void owWriteBitIsr(int state);
extern void owTimeoutIsr(int unused);

#endif
