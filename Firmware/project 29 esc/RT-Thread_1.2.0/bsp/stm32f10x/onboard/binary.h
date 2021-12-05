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

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _BINARY_H
#define _BINARY_H

#include "main.h"

enum binaryParseStates {
    BINARY_STATE_SYNC1 = 0,
    BINARY_STATE_SYNC2,
    BINARY_STATE_SIZE,
    BINARY_STATE_PAYLOAD,
    BINARY_STATE_CHKA,
    BINARY_STATE_CHKB
};

enum binaryCommands {
    BINARY_COMMAND_NOP = 0,
    BINARY_COMMAND_ARM,
    BINARY_COMMAND_CLI,
    BINARY_COMMAND_CONFIG,
    BINARY_COMMAND_DISARM,
    BINARY_COMMAND_DUTY,
    BINARY_COMMAND_PWM,
    BINARY_COMMAND_RPM,
    BINARY_COMMAND_SET,
    BINARY_COMMAND_START,
    BINARY_COMMAND_STATUS,
    BINARY_COMMAND_STOP,
    BINARY_COMMAND_TELEM_RATE,
    BINARY_COMMAND_VERSION,
    BINARY_COMMAND_TELEM_VALUE,
    BINARY_COMMAND_ACK = 250,
    BINARY_COMMAND_NACK
};

enum binaryValues {
    BINARY_VALUE_NONE = 0,
    BINARY_VALUE_AMPS,
    BINARY_VALUE_VOLTS_BAT,
    BINARY_VALUE_VOLTS_MOTOR,
    BINARY_VALUE_RPM,
    BINARY_VALUE_DUTY,
    BINARY_VALUE_COMM_PERIOD,
    BINARY_VALUE_BAD_DETECTS,
    BINARY_VALUE_ADC_WINDOW,
    BINARY_VALUE_IDLE_PERCENT,
    BINARY_VALUE_STATE,
    BINARY_VALUE_AVGA,
    BINARY_VALUE_AVGB,
    BINARY_VALUE_AVGC,
    BINARY_VALUE_AVGCOMP,
    BINARY_VALUE_FETSTEP,
    BINARY_VALUE_NUM
};

typedef struct {
    uint8_t command;
    uint16_t seqId;
    float params[2];
} __attribute__((packed)) binaryCommandStruct_t;

extern void binaryCheck(void);
extern void configRecalcConst(void);

#endif