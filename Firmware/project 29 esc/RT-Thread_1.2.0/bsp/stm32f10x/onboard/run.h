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

#ifndef _RUN_H
#define _RUN_H

#include "misc.h"

#define RUN_FREQ		2000		    // Hz
#define RUN_ARM_COUNT		20		    // number of valid PWM signals seen before arming
#define RUN_MIN_MAX_CURRENT	0.0		    // Amps
#define RUN_MAX_MAX_CURRENT	75.0		    // Amps

//#define RUN_ENABLE_IWDG
#define RUN_LSI_FREQ		40000		    // 40 KHz LSI for IWDG

enum runCommandModes {
    CLI_MODE = 0,
    BINARY_MODE
};

enum runModes {
    OPEN_LOOP = 0,
    CLOSED_LOOP_RPM,
    CLOSED_LOOP_THRUST,
    SERVO_MODE,
    NUM_RUN_MODES
};

extern volatile uint32_t runCount;
extern float idlePercent;
extern float avgAmps, maxAmps;
extern float avgVolts;
extern float rpm;
extern float targetRpm;
extern float runRPMFactor;
extern uint8_t disarmReason;
extern uint8_t commandMode;
extern uint8_t escId;
volatile extern uint8_t runMode;

extern void runInit(void);
extern void runNewInput(uint16_t setpoint);
extern void runDisarm(int reason);
extern void runArm(void);
extern void runStart(void);
extern void runStop(void);
extern uint8_t runDuty(float duty);
extern void runRpmPIDReset(void);
extern void runSetConstants(void);
extern uint16_t runIWDGInit(int ms);
extern void runFeedIWDG(void);
extern void runSetpoint(uint16_t val);

#endif
