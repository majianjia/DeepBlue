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

    Copyright 漏 2011, 2012  Bill Nesbitt
*/

#include "rtthread.h"

#include "main.h"
#include "rcc.h"
#include "config.h"
#include "timer.h"
#include "fet.h"
#include "serial.h"
#include "pwm.h"
#include "adc.h"
#include "run.h"
#include "cli.h"
#include "ow.h"

digitalPin *errorLed, *statusLed;
#ifdef ESC_DEBUG
digitalPin *tp;
#endif

volatile uint32_t minCycles, idleCounter,      totalCycles;
//                最小周期   main循环运行次数  总共周期     这几个参数用来计算空闲时间百分比(也就是没有中断,在死循环里什么事情也不做)
volatile uint8_t state,    inputMode;
//               运行状态  输入模式(控制方式 串口 iic can pwm 1wire)
__asm void nop(void);


struct rt_semaphore sem;
void main_timeout(void* p)
{
	rt_sem_release(&sem);
}

int thread_main(void) 
{
	//this function is the systick_interrupt_handle, now drive by soft timer
	extern void period_1ms(void);
	
	//THIS TWO FUNCTIONS are ctrlling the period of period_1ms
	rt_sem_init(&sem, "1ms out", RT_NULL, RT_IPC_FLAG_FIFO);
	rt_timer_create("1ms", main_timeout, RT_NULL, 1, RT_IPC_FLAG_FIFO);
	
	rccInit();     //开启CPU默认的一些时钟等

	statusLed = digitalInit(GPIO_STATUS_LED_PORT, GPIO_STATUS_LED_PIN);
	errorLed = digitalInit(GPIO_ERROR_LED_PORT, GPIO_ERROR_LED_PIN);
#ifdef ESC_DEBUG
	tp = digitalInit(GPIO_TP_PORT, GPIO_TP_PIN);
	digitalLo(tp);
#endif

	timerInit();   //timer2内部计数器
	configInit();  //加载默认的配置

	adcInit();     //ADC采样配置
	fetInit();
	serialInit();  //串口初始化
	runInit();
	cliInit();     //向串口发送一些版本信息,串口数据处理也在这里面实现
	owInit();      //初始化1wire协议

	runDisarm(REASON_STARTUP);
	inputMode = ESC_INPUT_PWM;

	fetSetDutyCycle(0);
	fetBeep(200, 100);
	fetBeep(300, 100);
	fetBeep(400, 100);
	fetBeep(500, 100);
	fetBeep(400, 100);
	fetBeep(300, 100);
	fetBeep(200, 100);

	pwmInit(); //PWM IN初始化

	//LED初始化
	digitalHi(statusLed);
	digitalHi(errorLed);

	// self calibrating idle timer loop
	{
//		volatile unsigned long cycles;
//		volatile unsigned int *DWT_CYCCNT  = (volatile unsigned int *)0xE0001004;//当前PC采样器周期计数寄存器
//		volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000;
//		volatile unsigned int *SCB_DEMCR   = (volatile unsigned int *)0xE000EDFC;

//		*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
//		*DWT_CONTROL = *DWT_CONTROL | 1;	// enable the counter

		minCycles = 0xffff;

		while (1) 
		{
			rt_sem_take(&sem, 5);
			period_1ms();				//replace the system tick interrupt
			
			
//			idleCounter++;

//			//NOPS_4;
//			nop();

//			cycles = *DWT_CYCCNT;
//			*DWT_CYCCNT = 0;		    // reset the counter

//			//用来计算空闲时间百分比
//			// record shortest number of instructions for loop
//			totalCycles += cycles;
//			if (cycles < minCycles)
//				minCycles = cycles;
		}
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1) {
    NOP;
  }
}
#endif

//
void hardfault_esc32(void)
{
	FET_PANIC;
}

//void HardFault_Handler(void) {
//    FET_PANIC;
//    while (1)
//	;
//}

void MemManage_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void BusFault_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void UsageFault_Handler(void) {
    FET_PANIC;
    while (1)
	;
}

void reset_wait(void) {
    FET_PANIC;
    while (1)
	;
}

#if 0
//__disable_irq()
__asm void CPSID_I(void)
{
	PUSH {lr}
	cpsid i
	POP {PC}
}

//__disable_fault_irq()
__asm void CPSID_F(void)
{
	PUSH {lr}
	cpsid f
	POP {PC}
}

//__enable_irq()
__asm void CPSIE_I(void)
{
	PUSH {lr}
	cpsie i 
	POP {PC}
}
#endif

__asm void nop(void)
{
	PUSH {lr}
	nop
	nop
	nop
	nop
	POP {PC}
}
