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

#include "timer.h"
#include "stm32f10x_tim.h"
#include "misc.h"

timerStruct_t timerData;
volatile uint32_t timerMicros;

// must be called at least once every 65536 ticks in order for this strategy to work
// TODO - consider interrupt interference
uint32_t timerGetMicros(void) {
    static uint16_t timerLast;
    static uint16_t hiBits;
    register uint16_t tmp;

    tmp = TIMER_TIM->CNT;

    if (tmp < timerLast)
	    hiBits++;

    timerLast = tmp;

    timerMicros = (hiBits<<16 | tmp) & TIMER_MASK;

    return timerMicros;
}

//使用timer2做延时
void timerDelay(uint16_t us) {
	uint16_t cnt = TIMER_TIM->CNT;
	uint16_t targetTimerVal = cnt + us*TIMER_MULT;

	if (targetTimerVal < cnt)
		// wait till timer rolls over
		while (TIMER_TIM->CNT > targetTimerVal)
			;

	while (TIMER_TIM->CNT < targetTimerVal)
		;
}

//取消通道1,比较中断
void timerCancelAlarm1(void) 
{
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC1;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC1);
}
//取消通道2,比较中断
void timerCancelAlarm2(void) 
{
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC2;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC2);
}
//取消通道3,比较中断
void timerCancelAlarm3(void) 
{
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC3;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC3);
}

uint8_t timerAlarmActive3(void) {
    return (TIMER_TIM->DIER & TIM_IT_CC3);
}

// Create a timer  初始化TIMER2
void timerInit(void) 
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // Enable the TIMER_TIM global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Prescaler = (72/TIMER_MULT) - 1;   //分频
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;                   //自动重载寄存器
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                 //分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
    TIM_TimeBaseInit(TIMER_TIM, &TIM_TimeBaseStructure);

	//取消1 2 3路 比较中断
    timerCancelAlarm1();
    timerCancelAlarm2();
    timerCancelAlarm3();

    // Output Compare for alarm
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    TIM_OC2Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    TIM_OC3Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    TIM_ARRPreloadConfig(TIMER_TIM, ENABLE);//ARR有缓冲器

    // go...
    TIM_Cmd(TIMER_TIM, ENABLE);//开启计数器
}




// TODO: worry about 32 bit rollover
//参数：
// ticks：要调用callback函数的相对时间
// callback：要调用的函数
// parameter：要调用函数的参数
//被adc.c使用 adc采样
void timerSetAlarm1(int32_t ticks, timerCallback_t *callback, int parameter) {
    // do it now
    if (ticks <= TIMER_MULT) 
	{
		//小于2个ticks.直接调用callback传递来的函数
		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC1;

		callback(parameter);
    }
    // otherwise, schedule it
    else {
		//否则注册.在定时器中断里面调用
		timerData.alarm1Callback = callback;
		timerData.alarm1Parameter = parameter;

		TIMER_TIM->CCR1 = TIMER_TIM->CNT + ticks;//设备捕获比较中断寄存器
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC1;

		TIMER_TIM->DIER |= TIM_IT_CC1;//开中断
    }
}
//被fet.c使用 电调的主要控制逻辑部分
void timerSetAlarm2(int32_t ticks, timerCallback_t *callback, int parameter) {
    // do it now
    if (ticks <= TIMER_MULT) {
		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC2;

		callback(parameter);
    }
    // otherwise, schedule it
    else {
		timerData.alarm2Callback = callback;
		timerData.alarm2Parameter = parameter;

		TIMER_TIM->CCR2 = TIMER_TIM->CNT + ticks;
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC2;
		TIMER_TIM->DIER |= TIM_IT_CC2;
    }
}
//被ow.c使用.主要用于1wire协议
void timerSetAlarm3(int32_t ticks, timerCallback_t *callback, int parameter) {
    // do it now
    if (ticks <= TIMER_MULT) {
		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC3;

		callback(parameter);
    }
    // otherwise, schedule it
    else {
		timerData.alarm3Callback = callback;
		timerData.alarm3Parameter = parameter;

		TIMER_TIM->CCR3 = TIMER_TIM->CNT + ticks;
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC3;
		TIMER_TIM->DIER |= TIM_IT_CC3;
    }
}

//timer2 中断
void TIMER_ISR(void) 
{
    if (TIM_GetITStatus(TIMER_TIM, TIM_IT_CC1) != RESET) 
	{
		//判断第1路比较中断
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC1;

		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC1;

		timerData.alarm1Callback(timerData.alarm1Parameter);
    }
    else if (TIM_GetITStatus(TIMER_TIM, TIM_IT_CC2) != RESET) 
	{
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC2;

		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC2;

		timerData.alarm2Callback(timerData.alarm2Parameter);
    }
    else if (TIM_GetITStatus(TIMER_TIM, TIM_IT_CC3) != RESET) 
	{
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC3;

		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC3;

		timerData.alarm3Callback(timerData.alarm3Parameter);
    }
}
