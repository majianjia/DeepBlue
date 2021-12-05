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

#ifndef _FET_H
#define _FET_H

#include "stm32f10x_gpio.h"

#define FET_A_L_PORT		GPIOA
#define FET_B_L_PORT		GPIOB
#define FET_C_L_PORT		GPIOB
#define FET_A_H_PORT		GPIOB
#define FET_B_H_PORT		GPIOB
#define FET_C_H_PORT		GPIOB

#define FET_A_L_PIN		GPIO_Pin_7
#define FET_B_L_PIN		GPIO_Pin_0
#define FET_C_L_PIN		GPIO_Pin_1
#define FET_A_H_PIN		GPIO_Pin_6
#define FET_B_H_PIN		GPIO_Pin_7
#define FET_C_H_PIN		GPIO_Pin_8

#define AL_ON			FET_A_L_PIN
#define BL_ON			FET_B_L_PIN
#define CL_ON			FET_C_L_PIN
#define AL_OFF			(FET_A_L_PIN<<16)
#define BL_OFF			(FET_B_L_PIN<<16)
#define CL_OFF			(FET_C_L_PIN<<16)

#define AH_ON			FET_A_H_PIN
#define BH_ON			FET_B_H_PIN
#define CH_ON			FET_C_H_PIN
#define AH_OFF			(FET_A_H_PIN<<16)
#define BH_OFF			(FET_B_H_PIN<<16)
#define CH_OFF			(FET_C_H_PIN<<16)


#define FET_A_L_OFF FET_A_L_PORT->BSRR = AL_OFF
#define FET_B_L_OFF FET_B_L_PORT->BSRR = BL_OFF
#define FET_C_L_OFF FET_C_L_PORT->BSRR = CL_OFF

#define FET_A_H_OFF FET_A_H_PORT->BSRR = AH_OFF
#define FET_B_H_OFF FET_B_H_PORT->BSRR = BH_OFF
#define FET_C_H_OFF FET_C_H_PORT->BSRR = CH_OFF

#define FET_A_L_ON FET_A_L_PORT->BSRR = AL_ON
#define FET_B_L_ON FET_B_L_PORT->BSRR = BL_ON
#define FET_C_L_ON FET_C_L_PORT->BSRR = CL_ON

#define FET_A_H_ON FET_A_H_PORT->BSRR = AH_ON
#define FET_B_H_ON FET_B_H_PORT->BSRR = BH_ON
#define FET_C_H_ON FET_C_H_PORT->BSRR = CH_ON

// bit band address turn switch on or off PWM output
// HI side
#define AH_BITBAND		((uint32_t *)(0x42000000 + (0x10C00*32) + (27*4)))
#define BH_BITBAND		((uint32_t *)(0x42000000 + (0x10C00*32) + (31*4)))
#define CH_BITBAND		((uint32_t *)(0x42000000 + (0x10C04*32) + (3*4)))
// LO side
#define AL_BITBAND		((uint32_t *)(0x42000000 + (0x10800*32) + (31*4)))
#define BL_BITBAND		((uint32_t *)(0x42000000 + (0x10C00*32) + (3*4)))
#define CL_BITBAND		((uint32_t *)(0x42000000 + (0x10C00*32) + (7*4)))

#define FET_MASTER_TIMER        TIM3
#define FET_MASTER_DBGMCU_STOP  DBGMCU_TIM3_STOP

#define FET_H_TIMER		TIM4
#define FET_H_TIMER_REMAP
#define FET_H_TIMER_MASTER      TIM_TS_ITR2			    // TIM3
#define FET_DBGMCU_STOP		DBGMCU_TIM4_STOP
#define FET_AHB_FREQ		(SystemCoreClock/2)		    // 36Mhz

// HI side timer channels
#define FET_A_H_CHANNEL		CCR1
#define FET_B_H_CHANNEL		CCR2
#define FET_C_H_CHANNEL		CCR3

// LO side timer channels
#define FET_A_L_CHANNEL		CCR2
#define FET_B_L_CHANNEL		CCR3
#define FET_C_L_CHANNEL		CCR4

// Servo stuff
#define FET_DEADTIME		18				    // 36Mhz clock ticks
#define FET_SERVO_RESOLUTION	1024
#ifndef M_PI
#define M_PI			3.14159265f
#endif

#define FET_MIN_SWITCH_FREQ	4				    // KHz
#define FET_MAX_SWITCH_FREQ	64				    // KHz
#define FET_MIN_START_VOLTAGE	0.1				    // %
#define FET_MAX_START_VOLTAGE	3.0				    // %
#define FET_MIN_START_DETECTS	1
#define FET_MAX_START_DETECTS	512
#define FET_MIN_DISARM_DETECTS	1
#define FET_MAX_DISARM_DETECTS	512
#define FET_MIN_LIMIT_STEP	0.1				    // %
#define FET_MAX_LIMIT_STEP	100.0				    // %

#define FET_PANIC { \
    *AH_BITBAND = 0; \
    *BH_BITBAND = 0; \
    *CH_BITBAND = 0; \
    FET_A_L_PORT->BSRR = AL_OFF; \
    FET_B_L_PORT->BSRR = BL_OFF; \
    FET_C_L_PORT->BSRR = CL_OFF; \
}

enum fetSelfTestResults {
    FET_TEST_NOT_RUN = 0,
    FET_TEST_PASSED,
    FET_TEST_A_LO_FAIL,
    FET_TEST_B_LO_FAIL,
    FET_TEST_C_LO_FAIL,
    FET_TEST_A_HI_FAIL,
    FET_TEST_B_HI_FAIL,
    FET_TEST_C_HI_FAIL
};

extern int32_t fetSwitchFreq;
extern int32_t fetStartDuty;
extern int16_t fetStartDetects;
extern int16_t fetDisarmDetects;

extern volatile uint8_t fetStep;
extern volatile int8_t fetNextStep;
extern volatile uint32_t fetBadDetects;
extern volatile uint32_t fetGoodDetects;
extern volatile uint32_t fetTotalBadDetects;
extern int32_t fetActualDutyCycle;
extern volatile int32_t fetDutyCycle;
extern int32_t fetPeriod;
extern volatile uint32_t fetCommutationMicros;
extern int8_t fetBrakingEnabled;
extern int8_t fetBraking;
extern int8_t fetStepDir;
extern float servoAngle;

extern void fetInit(void);
extern uint8_t fetSelfTest(void);
extern void fetBeep(uint16_t freq, uint16_t duration);
extern void fetCommutate(int unused);
extern void fetSetStep(int n);
extern void fetSetDutyCycle(int32_t dutyCycle);
extern void motorStartSeqInit (void);
extern void motorStartSeq (int period);
extern void fetStartCommutation(uint8_t startStep);
extern void fetSetConstants(void);
extern void fetSetBraking(int8_t value);
extern void _fetSetDutyCycle(int32_t dutyCycle);
extern void fetSetAngleFromPwm(int32_t pwm);
extern void fetSetAngle(float angle);
extern void fetUpdateServo(void);

#endif
