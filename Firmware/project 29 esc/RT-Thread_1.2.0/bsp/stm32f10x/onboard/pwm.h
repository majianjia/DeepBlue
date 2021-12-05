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

#ifndef _PWM_H
#define _PWM_H

#include "config.h"
#include "stm32f10x_tim.h"

#define PWM_PORT	    GPIOA
#define PWM_PIN		    GPIO_Pin_8

#define PWM_OUTPUT	    {PWM_PORT->CRH = (PWM_PORT->CRH & ~0x0f) | 0x03;}
#define PWM_INPUT	    {PWM_PORT->CRH = (PWM_PORT->CRH & ~0x0f) | 0x04;}
#define PWM_SAMPLE_LEVEL    ((PWM_PORT->IDR & (0x01<<8))>>8)

#define PWM_TIM		    TIM1
#define PWM_CHANNEL	    TIM_Channel_1
#define PWM_IRQ		    TIM1_CC_IRQn
#define PWM_IRQ_HANDLER	    TIM1_CC_IRQHandler
#define PWM_CLK_DIVISOR	    72

#define PWM_TIMEOUT	    (200000*TIMER_MULT)	    // micros that the last received PWM signal is valid for (0.2 seconds)

#define	PWM_RPM_SCALE_MIN   1000.0f
#define	PWM_RPM_SCALE_MAX   20000.0f

extern int16_t pwmMinPeriod;
extern int16_t pwmMaxPeriod;
extern int16_t pwmMinValue;
extern int16_t pwmLoValue;
extern int16_t pwmHiValue;
extern int16_t pwmMaxValue;
extern int16_t pwmMinStart;
extern volatile uint32_t pwmValidMicros;
extern volatile uint16_t pwmValue;

extern void pwmInit(void);
extern void pwmSetConstants(void);
extern void pwmIsrAllOff(void);
extern void pwmIsrAllOn(void);
extern void pwmIsrRunOn(void);

#endif
