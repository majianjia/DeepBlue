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

#include "pwm.h"
#include "timer.h"
#include "run.h"
#include "main.h"
#include "ow.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "misc.h"

int16_t pwmMinPeriod;
int16_t pwmMaxPeriod;
int16_t pwmMinValue;
int16_t pwmLoValue;
int16_t pwmHiValue;
int16_t pwmMaxValue;
int16_t pwmMinStart;
volatile uint32_t pwmValidMicros;

inline void pwmIsrAllOff(void) {
    PWM_TIM->DIER &= (uint16_t)~(TIM_IT_CC1 | TIM_IT_CC2);
}

inline void pwmIsrAllOn(void) {
    PWM_TIM->CCR1;
    PWM_TIM->CCR2;
    PWM_TIM->DIER |= (TIM_IT_CC1 | TIM_IT_CC2);
}

inline void pwmIsrRunOn(void) {
    uint16_t dier = PWM_TIM->DIER;

    dier &= (uint16_t)~(TIM_IT_CC1 | TIM_IT_CC2);
    dier |= TIM_IT_CC2;

    PWM_TIM->CCR1;
    PWM_TIM->CCR2;
    PWM_TIM->DIER = dier;
}

void pwmInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    pwmSetConstants();

    // TIM1 channel 1 pin (PA.08) configuration
    GPIO_InitStructure.GPIO_Pin = PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_PORT, &GPIO_InitStructure);

    // Enable the TIM1 global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = PWM_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (PWM_CLK_DIVISOR-1);
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = PWM_CHANNEL;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x08;
    TIM_PWMIConfig(PWM_TIM, &TIM_ICInitStructure);

    // Select the TIM Input Trigger: TI1FP1
    TIM_SelectInputTrigger(PWM_TIM, TIM_TS_TI1FP1);

    // Select the slave Mode: Reset Mode
    TIM_SelectSlaveMode(PWM_TIM, TIM_SlaveMode_Reset);

    // Enable the Master/Slave Mode
    TIM_SelectMasterSlaveMode(PWM_TIM, TIM_MasterSlaveMode_Enable);

    // TIM enable counter
    TIM_Cmd(PWM_TIM, ENABLE);

    pwmIsrAllOn();
}

void PWM_IRQ_HANDLER(void) {
    uint16_t pwmValue;
    uint16_t periodValue;
    uint8_t edge;

    edge = !(PWM_TIM->SR & TIM_IT_CC2);

    periodValue = PWM_TIM->CCR1;
    pwmValue = PWM_TIM->CCR2;

    // is this an OW reset pulse?
    if (state == ESC_STATE_DISARMED && edge == 1 && (periodValue - pwmValue) > OW_RESET_MIN && (periodValue - pwmValue) < OW_RESET_MAX) {
	owReset();
    }
    // look for good RC PWM input
    else if (inputMode == ESC_INPUT_PWM && periodValue >= pwmMinPeriod && periodValue <= pwmMaxPeriod && pwmValue >= pwmMinValue && pwmValue <= pwmMaxValue) {
	if (edge == 0) {
	    pwmValidMicros = timerMicros;
	    runNewInput(pwmValue);
	}
    }
    // otherwise if already in OW mode, pass control to OW
    else if (inputMode == ESC_INPUT_OW) {
	owEdgeDetect(edge);
    }
}

void pwmSetConstants(void) {
    float rpmScale = p[PWM_RPM_SCALE];

    pwmMinPeriod = p[PWM_MIN_PERIOD] = (int)p[PWM_MIN_PERIOD];
    pwmMaxPeriod = p[PWM_MAX_PERIOD] = (int)p[PWM_MAX_PERIOD];
    pwmMinValue = p[PWM_MIN_VALUE] = (int)p[PWM_MIN_VALUE];
    pwmLoValue = p[PWM_LO_VALUE] = (int)p[PWM_LO_VALUE];
    pwmHiValue = p[PWM_HI_VALUE] = (int)p[PWM_HI_VALUE];
    pwmMaxValue = p[PWM_MAX_VALUE] = (int)p[PWM_MAX_VALUE];
    pwmMinStart = p[PWM_MIN_START] = (int)p[PWM_MIN_START];

    if (rpmScale < PWM_RPM_SCALE_MIN)
	rpmScale = PWM_RPM_SCALE_MIN;
    else if (rpmScale > PWM_RPM_SCALE_MAX)
	rpmScale = PWM_RPM_SCALE_MAX;

    p[PWM_RPM_SCALE] = rpmScale;
}