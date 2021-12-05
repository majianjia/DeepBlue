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

#include "rcc.h"
#include "adc.h"
#include "main.h"
#include "digital.h"
#include "stm32f10x_rcc.h"

//从备份寄存器中读取数据
static uint32_t rccReadBkpDr(void) 
{
    return *((uint16_t *)BKP_BASE + 0x04) | *((uint16_t *)BKP_BASE + 0x08)<<16;
}

void rccWriteBkpDr(uint32_t value) 
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR->CR |= PWR_CR_DBP;

    *((uint16_t *)BKP_BASE + 0x04) = value & 0xffff;
    *((uint16_t *)BKP_BASE + 0x08) = (value & 0xffff0000)>>16;
}

static __asm void rcc_asm(void)
{
	PUSH {lr}
	LDR     R0, =0x1FFFF000
	LDR     SP,[R0, #0]
	LDR     R0,[R0, #4]
	BX      R0
	;POP {PC}
}

static void rccBootLoader(void) 
{
    // check for magic cookie
    if (rccReadBkpDr() == 0xDECEA5ED)
	{
		digitalPin *statusLed, *errorLed;

		rccWriteBkpDr(0); // reset flag

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

		//设置两个LED并输出低电平
		statusLed = digitalInit(GPIO_STATUS_LED_PORT, GPIO_STATUS_LED_PIN);
		digitalLo(statusLed);
		errorLed = digitalInit(GPIO_ERROR_LED_PORT, GPIO_ERROR_LED_PIN);
		digitalLo(errorLed);

		// jump to boot loader ROM
#if 0
		__asm volatile ("LDR     R0, =0x1FFFF000\n"
			"LDR     SP,[R0, #0]\n"
			"LDR     R0,[R0, #4]\n"
			"BX      R0\n");
#endif
		rcc_asm();
    }
}

void rccReset(void) 
{
    // set magic cookie
    rccWriteBkpDr(0xDECEA5ED);

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

void rccInit(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;

    rccBootLoader();

    // turn on fault interrupts
    SCB->SHCSR |= (0x01<<SCB_SHCSR_USGFAULTENA_Pos);//开启 用法故障使能位
    SCB->SHCSR |= (0x01<<SCB_SHCSR_BUSFAULTENA_Pos);//开启 总线故障使能位
    SCB->SHCSR |= (0x01<<SCB_SHCSR_MEMFAULTENA_Pos);//开启 存储器故障使能位


	//开GPIO TIMER ADC AFIO DMA UART1 PWR BKP的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // used for low power detection
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);


	//所有的GPIO设置为模拟输入
    /*
	    Configure all unused GPIO port pins in Analog Input mode (floating input
	    trigger OFF), this will reduce the power consumption and increase the device
	    immunity against EMI/EMC

	    NOTE: the ADC code assumes this has been done
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    // Disable JTAG-DP
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	//配置ADC的采样时钟
    RCC_ADCCLKConfig(ADC_CLOCK);

    // clear reset flags
    RCC_ClearFlag();

    // Shutdown HSI clock
    RCC_HSICmd(DISABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == SET)
		;
}
