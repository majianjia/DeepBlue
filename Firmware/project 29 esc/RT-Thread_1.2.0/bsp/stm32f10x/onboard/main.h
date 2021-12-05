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

#ifndef _MAIN_H
#define _MAIN_H

#define VERSION			"2.0.0"

#include "digital.h"

#define ESC_DEBUG				// uncomment to include debugging code

#define GPIO_ERROR_LED_PORT	GPIOB
#define GPIO_ERROR_LED_PIN	GPIO_Pin_4
#define GPIO_STATUS_LED_PORT	GPIOB
#define GPIO_STATUS_LED_PIN	GPIO_Pin_3
#define GPIO_TP_PORT		GPIOB
#define GPIO_TP_PIN		GPIO_Pin_15

#define NOP			{__asm volatile ("nop\n\t");}
#define NOPS_4			{NOP; NOP; NOP; NOP;}

enum escStates {
    ESC_STATE_DISARMED = 0,
    ESC_STATE_STOPPED,
    ESC_STATE_NOCOMM,
    ESC_STATE_STARTING,
    ESC_STATE_RUNNING
};

enum escInputModes {
    ESC_INPUT_PWM = 0,
    ESC_INPUT_UART,
    ESC_INPUT_I2C,
    ESC_INPUT_CAN,
    ESC_INPUT_OW,
    ESC_INPUT_MAX
};

enum escDisarmReasons {
    REASON_STARTUP = 0,
    REASON_BAD_DETECTS,
    REASON_CROSSING_TIMEOUT,
    REASON_PWM_TIMEOUT,
    REASON_LOW_VOLTAGE,
    REASON_CLI_USER,
    REASON_BINARY_USER,
    REASON_CAN_USER,
    REASON_CAN_TIMEOUT
};

extern digitalPin *errorLed, *statusLed, *tp;
extern volatile uint32_t minCycles, idleCounter, totalCycles;
extern volatile uint8_t state, inputMode;

extern void escRun(void);

#endif
