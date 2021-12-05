/*
 * File      : thread_ppm.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-04    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include "math.h"

#include "struct_all.h"
#include "thread_ppm.h"
#include "time_measure.h"

//INPUT pin
#define PPM_INPUT_BIT (GPIOA->IDR & GPIO_Pin_12)

int volatile ppm_raw[16]; 
unsigned int volatile ppm_count = 0;			//到第几个通道了
unsigned int volatile ppm_count_valid= 0;		//PPM通道有效值
int ppm_offset[16];

static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//sem
static struct rt_semaphore sem;

/* 定时器的控制块*/
static struct rt_timer tim_ppm;

/* 定时器超时函数*///即失控函数
static void timer_ppm_timeout(void* parameter)
{
	static unsigned int ppm_count_bef = 0;
	if(ppm.count == ppm_count_bef)
	{
		ppm.input.ch3 = -500;//PPM无效后关闭油门
		ppm.valid_count = 0;	 //有效数等于0
		ppm.flag = 0;
	}
	ppm_count_bef = ppm.count;
}

//used for canbus decode update flag
void ppm_can_update(void)
{
	//wait until one canbus PPM frame update
	rt_sem_release(&sem);
}


//interrupt
void ppm_interrupt_handle(void)
{
	static unsigned int temp;
	
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
		
		//LOW LEVEL
		if(!(PPM_INPUT_BIT))
		{
			//disable the measurement channel
			TIM_Cmd(TIM7, DISABLE);	
			temp = TIM7->CNT;
			
			//if the length longer then 3000us that isn't the valid channel 
			if(temp >= 3000)
			{
				ppm_count_valid = ppm_count;//mark the new valid channel count
				ppm_count = 0;
				
				if(temp >= 50000)		//if longer then 50ms,than the signal isn't valid
					ppm.input.flag = 0;
				else
					ppm.input.flag = 1;
				rt_sem_release(&sem);	//release one semaphone for ppm_thread to update a new data
			}
			else
			{
				ppm_raw[ppm_count] = temp;
				ppm_count ++;				//set the point to next channel
				if(ppm_count >= 16)
					ppm_count= 0;
			}
			
			TIM_TimeBaseStructure.TIM_Period = 0xFFFF;				//定时器周期为最大值，65536us = 65ms	
			TIM_TimeBaseStructure.TIM_Prescaler = 84-1;				//计时精度为 us
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
			TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
			TIM_Cmd(TIM7, ENABLE);									//开始计时
			
		}
		//HIGH level
		else
		{

		}
		
	}
}

//timer interrupt
void ppm_timer_handle(void)
{


}

//hardware initialization
void ppm_hw_init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;


    //GPIO Periph clock enable 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	//PPM PO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  //PULL UP
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//设定定时器
	TIM_DeInit(TIM7);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;				//定时器周期为最大值，65536us = 65ms	
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;				//计时精度为 us
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	//timer not started now
	
	/* Connect EXTI Line12 to Pa12 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);
	
	//clear it
	EXTI_ClearITPendingBit(EXTI_Line12);

	/* Configure EXTI Line12 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  //both rising and falling will put into interrupt
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
// 	/* Enable and set TIM7 ,that is used to judge if the timer is overflow */
// 	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);

}



//初始化信号量 和中立点
void ppm_sw_init(void)
{
	char count = 15;
	//初始化信号量
	rt_sem_init(&sem,"ppm",0 ,RT_IPC_FLAG_FIFO);
	
	/* 创建超时定时器 */
	rt_timer_init(&tim_ppm,
					"ppm", 				/* 定时器名字 */
					timer_ppm_timeout, 				/* 超时时回调的处理函数*/
					RT_NULL,				/* 超时函数的入口参数*/
					3000, 					/* 定时长度，以OS Tick为单位*/
					RT_TIMER_FLAG_PERIODIC);/* 周期性定时器*/	
	/* 启动定时器*/
	rt_timer_start(&tim_ppm);
	
	//set the default value of the offset 
	while(count)
	{
		ppm_offset[count--] = 0;
	}
	ppm_offset[0] = 0;
	
	//set the ppm high level length to 1500us
	ppm.ppm_zero = 1500;
	
	//throttle to zero
	ppm.input.ch3 = -500;

}

//ppm hw and sw initialization
void ppm_init(void)
{
	ppm_sw_init();
	ppm_hw_init();
}


//thread ppm main body
void thread_ppm(void)
{
	//initialization
	ppm_init();
	
	//main circle
	while(1)
	{
		//wait until one PPM frame update
		//2second for timeout
		rt_sem_take(&sem, RT_TICK_PER_SECOND *2);
		
		//if  ppm can message is valid
		if(ppm.can.flag != 0)
		{
			ppm.input.ch1 = ppm.can.ch1 - ppm.ppm_zero;
			ppm.input.ch2 = ppm.can.ch2 - ppm.ppm_zero;
			ppm.input.ch3 = ppm.can.ch3 - ppm.ppm_zero;
			ppm.input.ch4 = ppm.can.ch4 - ppm.ppm_zero;
			ppm.input.ch5 = ppm.can.ch5 - ppm.ppm_zero;
			ppm.input.ch6 = ppm.can.ch6 - ppm.ppm_zero;
			ppm.input.ch7 = ppm.can.ch7 - ppm.ppm_zero;
			ppm.input.ch8 = ppm.can.ch8 - ppm.ppm_zero;
	
			ppm.valid_count = 8; //how many channels are valid
			ppm.count ++;
			ppm.flag = 1; //flag in the interrupt
		}	
		//if ppm input is valid
		else if(ppm.input.flag != 0)
		{
			ppm.input.ch1 = ppm_raw[0] - ppm.ppm_zero;
			ppm.input.ch2 = ppm_raw[1] - ppm.ppm_zero;
			ppm.input.ch3 = ppm_raw[2] - ppm.ppm_zero;
			ppm.input.ch4 = ppm_raw[3] - ppm.ppm_zero;
			ppm.input.ch5 = ppm_raw[4] - ppm.ppm_zero;
			ppm.input.ch6 = ppm_raw[5] - ppm.ppm_zero;
			ppm.input.ch7 = ppm_raw[6] - ppm.ppm_zero;
			ppm.input.ch8 = ppm_raw[7] - ppm.ppm_zero;
			ppm.input.ch9 = ppm_raw[8] - ppm.ppm_zero;	
			ppm.input.ch10 = ppm_raw[9] - ppm.ppm_zero;
			ppm.input.ch11 = ppm_raw[10] - ppm.ppm_zero;
			ppm.input.ch12 = ppm_raw[11] - ppm.ppm_zero;
			ppm.input.ch13 = ppm_raw[12] - ppm.ppm_zero;
			ppm.input.ch14 = ppm_raw[13] - ppm.ppm_zero;
			ppm.input.ch15 = ppm_raw[14] - ppm.ppm_zero;
			ppm.input.ch16 = ppm_raw[15] - ppm.ppm_zero;
			
			ppm.valid_count = ppm_count_valid; //how many channels are valid
			
			//标记数据可用 数据更新
			ppm.count ++;
			ppm.flag = 1; //flag in the interrupt
		}
		//no PPM can used(if there are 2 seconds for no ppm input )
		else
		{
			//ppm.input.ch3 = -500;//PPM无效后关闭油门
			ppm.valid_count = 0;	 //有效数等于0
			ppm.flag = 0;
		}

	}

}
