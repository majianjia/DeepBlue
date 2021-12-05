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

#include "ow.h"
#include "main.h"
#include "timer.h"
#include "pwm.h"
#include "run.h"
#include "cli.h"
#include <string.h>

uint8_t owROMCode[8];
uint8_t owBuf[16];
uint8_t *owBufPointer;
uint8_t owState;
uint8_t owLastCommand;
uint8_t owByteValue;
uint8_t owReadNum;
uint8_t owWriteNum;
uint8_t owBit;

uint8_t owCRC(uint8_t inData, uint8_t seed) {
    uint8_t bitsLeft;
    uint8_t temp;

    for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
	temp = ((seed ^ inData) & 0x01);
	if (temp == 0) {
	    seed >>= 1;
	}
	else {
	    seed ^= 0x18;
	    seed >>= 1;
	    seed |= 0x80;
	}
	inData >>= 1;
    }

    return seed;
}

void owInit(void) {
    int i;

    owROMCode[0] = 0xAA;
    for (i = 0; i < 6; i++)
	owROMCode[i+1] = *(((uint8_t *)(OW_UID_ADDRESS))+i);

    owROMCode[7] = 0x00;
    for (i = 0; i < 7; i++)
	owROMCode[7] = owCRC(owROMCode[i], owROMCode[7]);
}

void owPullLow(void) {
    pwmIsrAllOff();
    PWM_OUTPUT;
}

void owRelease(void) {
    PWM_INPUT;
    pwmIsrRunOn();
}

void owTimeout(int i) {
    timerSetAlarm3(i*TIMER_MULT, owTimeoutIsr, 0);
}

void owReset(void) {
    inputMode = ESC_INPUT_OW;
    owResetIsr(OW_RESET_STATE_0);
}

void owReadBytes(uint8_t num) {
    owTimeout(300);

    owReadNum = num;
    owByteValue = 0;
    owBit = 0;
}

void owWriteBytes(uint8_t num) {
    owTimeout(300);

    owByteValue = *owBufPointer;
    owWriteNum = num;
    owBit = 0;
}

// command parser
void owReadComplete(void) {
    uint8_t *pointer;

    switch (owBuf[0]) {
	case OW_ROM_MATCH:
	    if (owLastCommand != owBuf[0]) {
		owLastCommand = owBuf[0];
		// read an additional 8 bytes
		owBufPointer = &owBuf[1];
		owReadBytes(8);
	    }
	    else {
		owLastCommand = 0x00;
		// does the ROM code match ours?
		if (!memcmp(&owBuf[1], owROMCode, 8)) {
		    // prepare to read next command
		    owBufPointer = owBuf;
		    owReadBytes(1);
		}
		else {
		    // terminate transaction
		    owTimeoutIsr(0);
		}
	    }
	    break;

	case OW_ROM_SKIP:
	    // prepare to read next command
	    owBufPointer = owBuf;
	    owReadBytes(1);
	break;

	case OW_ROM_READ:
	    // prepare to write out ROM CODE (8 bytes)
	    owState = OW_WRITE;
	    owBufPointer = owROMCode;
	    owWriteBytes(8);
	break;

	case OW_VERSION:
	    owState = OW_WRITE;
	    owBufPointer = version;
	    owWriteBytes(sizeof(version));
	break;

	case OW_PARAM_READ:
	    if (owLastCommand != owBuf[0]) {
		owLastCommand = owBuf[0];
		// read an additional 1 bytes
		owBufPointer = &owBuf[1];
		owReadBytes(1);
	    }
	    else {
		owLastCommand = 0x00;
		if (owBuf[1] < CONFIG_NUM_PARAMS) {
		    // copy config value into send buffer
		    pointer = (uint8_t *)&(p[owBuf[1]]);
		    owBuf[2] = pointer[0];
		    owBuf[3] = pointer[1];
		    owBuf[4] = pointer[2];
		    owBuf[5] = pointer[3];

		    owState = OW_WRITE;
		    owBufPointer = owBuf;
		    owWriteBytes(6);
		}
	    }
	break;

	case OW_PARAM_WRITE:
	    if (owLastCommand != owBuf[0]) {
		owLastCommand = owBuf[0];
		// read an additional 5 bytes
		owBufPointer = &owBuf[1];
		owReadBytes(5);
	    }
	    else {
		owLastCommand = 0x00;
		if (owBuf[1] < CONFIG_NUM_PARAMS) {
		    // set parameter
		    configSetParamByID(owBuf[1], *(float *)(&owBuf[2]));

		    // copy config value into send buffer
		    pointer = (uint8_t *)&p[owBuf[1]];
		    owBuf[2] = pointer[0];
		    owBuf[3] = pointer[1];
		    owBuf[4] = pointer[2];
		    owBuf[5] = pointer[3];

		    owState = OW_WRITE;
		    owBufPointer = owBuf;
		    owWriteBytes(6);
		}
	}
	break;

	case OW_CONFIG_READ:
	    configReadFlash();

	    // return command ack
	    owState = OW_WRITE;
	    owBufPointer = owBuf;
	    owWriteBytes(1);
	break;

	// required 30ms to complete
	case OW_CONFIG_WRITE:
	    configWriteFlash();
	break;

	case OW_CONFIG_DEFAULT:
	    configLoadDefault();

	    // return command ack
	    owState = OW_WRITE;
	    owBufPointer = owBuf;
	    owWriteBytes(1);
	break;

	case OW_GET_MODE:
	    owBuf[0] = OW_GET_MODE;
	    owBuf[1] = runMode;
	    
	    owState = OW_WRITE;
	    owBufPointer = owBuf;
	    owWriteBytes(2);
	break;
	
	case OW_SET_MODE:
	    if (owLastCommand != owBuf[0]) {
		owLastCommand = owBuf[0];
		// read an additional 1 byte
		owBufPointer = &owBuf[1];
		owReadBytes(1);
	    }
	    else {
		owLastCommand = 0x00;
		if (owBuf[1] >= 0 && owBuf[1] < NUM_RUN_MODES) {
		    runMode = owBuf[1];

		    owState = OW_WRITE;
		    owBufPointer = owBuf;
		    owWriteBytes(2);
		}
	    }
	    ;
	break;
    }
}

void owWriteComplete(void) {
    // we're finished with the transaction
    owTimeoutIsr(0);
}

void owByteReceived(void) {
    *(owBufPointer++) = owByteValue;

    owTimeout(300);

    if (--owReadNum > 0)
	owReadBytes(owReadNum);
    else
	owReadComplete();
}

void owByteSent(void) {
    owBufPointer++;

    owTimeout(300);

    if (--owWriteNum > 0)
	owWriteBytes(owWriteNum);
    else
	owWriteComplete();
}

void owEdgeDetect(uint8_t edge) {
    if (edge == 0) {
	switch (owState) {
	    case OW_READ:
		// read bit in 15us
		pwmIsrAllOff();
		owRelease();
		timerSetAlarm3(15*TIMER_MULT, owReadBitIsr, 0);
	    break;

	    case OW_WRITE:
		// write bit
		owRelease();
		owWriteBitIsr(0);
	    break;
	}
    }
}

void owResetIsr(int state) {
    switch (state) {
	case OW_RESET_STATE_0:
	    // wait 15us
	    timerSetAlarm3(15*TIMER_MULT, owResetIsr, OW_RESET_STATE_1);
	break;

	case OW_RESET_STATE_1:
	    // signal presence for 70us
	    owPullLow();
	    timerSetAlarm3(70*TIMER_MULT, owResetIsr, OW_RESET_STATE_2);
	break;

	case OW_RESET_STATE_2:
	    // release bus and wait for byte
	    owRelease();

	    // light status LED
	    digitalLo(statusLed);

	    // prepare to read a byte
	    owState = OW_READ;
	    owBufPointer = owBuf;
	    owLastCommand = 0x00;
	    owReadBytes(1);

	    // 15ms timeout
	    owTimeout(15000);
	break;
    }
}

void owReadBitIsr(int unused) {
    // sample bus
    owByteValue |= (PWM_SAMPLE_LEVEL<<owBit);

    // wait for next write slot
    pwmIsrRunOn();

    owTimeout(150);

    // is byte complete?
    if (++owBit > 7)
	owByteReceived();
}

void owWriteBitIsr(int state) {
    // write bit
    if (state == 0) {
	// only 0 bits need action, 1 bits let the bus rise on its own
	if (!(owByteValue & (0x01<<owBit++))) {
	    owPullLow();
	    // schedule release
	    timerSetAlarm3(30*TIMER_MULT, owWriteBitIsr, 1);
	}
	else {
	    owTimeout(150);

	    // finished with this byte
	    if (owBit > 7)
		    owByteSent();
	}
    }
    // release bus
    else {
	owRelease();

	owTimeout(150);

	// finished with this byte
	if (owBit > 7)
		owByteSent();
    }
}

void owTimeoutIsr(int unused) {
    timerCancelAlarm3();
    // revert to listening for PWM & OW
    owRelease();
    pwmIsrAllOn();
    // turn off status LED
    digitalHi(statusLed);
    inputMode = ESC_INPUT_PWM;
}