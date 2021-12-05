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

#ifndef _TIMER_H
#define _TIMER_H

#include "stm32f10x_tim.h"

#define TIMER_TIM	    TIM2
#define TIMER_IRQ_CH	    TIM2_IRQn
#define TIMER_ISR	    TIM2_IRQHandler
#define TIMER_MULT	    2//4		    // 0.5 us resolution
#define TIMER_MASK	    0xFFFFFFFF	    // for testing timer roll-over

typedef void timerCallback_t(int);

typedef struct {
    timerCallback_t *alarm1Callback;
    int alarm1Parameter;

    timerCallback_t *alarm2Callback;
    int alarm2Parameter;

    timerCallback_t *alarm3Callback;
    int alarm3Parameter;
} timerStruct_t;

extern volatile uint32_t timerMicros;

extern void timerInit(void);
extern void timerDelay(uint16_t us);
extern void timerSetAlarm1(int32_t us, timerCallback_t *callback, int parameter);
extern void timerSetAlarm2(int32_t us, timerCallback_t *callback, int parameter);
extern void timerSetAlarm3(int32_t us, timerCallback_t *callback, int parameter);
extern void timerCancelAlarm1(void);
extern void timerCancelAlarm2(void);
extern void timerCancelAlarm3(void);
extern uint8_t timerAlarmActive3(void);
extern uint32_t timerGetMicros(void);

#endif
