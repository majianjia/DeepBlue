/*
 * File      : pwm.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014.3.15    majianjia   the first version
 */

#include "stm32f10x.h"
#include "pwm.h"
#include "rtthread.h"
#include "db_can_message.h"

//LED管脚定义
#define LED(value) 						\
{										\
	if (value)							\
		GPIOA->BSRR = GPIO_Pin_15;		\
	else								\
		GPIOA->BRR  = GPIO_Pin_15;		\
}

//LED管脚定义
#define PPM(value) 						\
{										\
	if (value)							\
		GPIOA->BSRR = GPIO_Pin_7;		\
	else								\
		GPIOA->BRR  = GPIO_Pin_7;		\
}


volatile unsigned long long ms50_count = 0;
volatile unsigned long long second_ms = {0};
volatile unsigned short pwm_us[8] = {0};
volatile unsigned int pwm_in[8] = {0};
volatile unsigned int pwm_update_count = 0;
int pwm_good_flag = 0;

//sem
static struct rt_semaphore sem;

//定时器控制块
static struct rt_timer tim_pwm;


static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructpwme;

//
void pwm_hw_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;      	

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	// 打开所有GPIO口时钟 端口复用时钟（外部中断）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
			
	//设定定时器
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	  		//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseStructpwme.TIM_Period = 50000;
	TIM_TimeBaseStructpwme.TIM_Prescaler = 64 -1 ;	
	TIM_TimeBaseStructpwme.TIM_ClockDivision =  0x0000;
	TIM_TimeBaseStructpwme.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructpwme);
	TIM_Cmd(TIM3, ENABLE);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
	
	//tim4 for ppm output
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	  		//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseStructpwme.TIM_Period = 0xffff;
	TIM_TimeBaseStructpwme.TIM_Prescaler = 64 -1 ;	
	TIM_TimeBaseStructpwme.TIM_ClockDivision =  0x0000;
	TIM_TimeBaseStructpwme.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructpwme);
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 

	
	//捕获
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6
									 | GPIO_Pin_7 ;
	//GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING ;//IPD;   //pull down
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	//PPM output
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	
	//清空中断标志
	EXTI_ClearITPendingBit(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);

	//接受口
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4 | EXTI_Line5
									| EXTI_Line6;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    	//第1通道上升和下降
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		

	//
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	  		//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);	
}

void tim_update_handle(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
		//50ms ++
		ms50_count++;
		if(ms50_count%20 == 0)
			second_ms++;
	}


}

void TIM4_IRQHandler(void)
{
	static unsigned int ppm_out_count = 0;
	static unsigned int ppm_value = 0;
	static unsigned int next_time = 100;
	#define HIGH_TIME 100 //us
	
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		
		
		
		
		
		
		
	}
}


//中断函数
void EXTI0_IRQHandler(void)
{
	static unsigned int ms_50;
	static unsigned int us;
	
	//clear interrupt flag
	EXTI_ClearITPendingBit(EXTI_Line0);
	
	//HIGHT
	if (GPIOA->IDR & GPIO_Pin_0)
	{
		//mark the start time point
		ms_50 = ms50_count;
		us = TIM3->CNT;
		
	}
	//LOW
	else
	{
		int temp;
		int tim_counter = TIM3->CNT; 
		//get pwm form history
		temp = (ms50_count - ms_50)*50000 + tim_counter - us;
		pwm_in[5] = temp;
		
		pwm_update_count++;

	}


}

void EXTI1_IRQHandler(void)
{
	static unsigned int ms_50;
	static unsigned int us;
	
	//clear interrupt flag
	EXTI_ClearITPendingBit(EXTI_Line1);
	
	//HIGHT
	if (GPIOA->IDR & GPIO_Pin_1)
	{
		//mark the start time point
		ms_50 = ms50_count;
		us = TIM3->CNT;
		
	}
	//LOW
	else
	{
		int temp;
		int tim_counter = TIM3->CNT; 
		//get pwm form history
		temp = (ms50_count - ms_50)*50000 + tim_counter - us;
		pwm_in[4] = temp;
		
		pwm_update_count++;
		
//		//if in one conter period
//		if(tim_counter > pwm_us[6] && pwm_ms50[6] >= ms50_count)	
//		{			
//			temp = tim_counter - pwm_us[6];
//			pwm_in[6] = temp;
//		}	
//		else if	(pwm_ms50[6] < ms50_count)
//		{
//			temp = (ms50_count - pwm_ms50[6])*50000 + tim_counter - pwm_us[6];
//			pwm_in[6] = temp;
//		}

	}

}

void EXTI2_IRQHandler(void)
{
	static unsigned int ms_50;
	static unsigned int us;
	
	//clear interrupt flag
	EXTI_ClearITPendingBit(EXTI_Line2);
	
	if (GPIOA->IDR & GPIO_Pin_2)
	{
		//mark the start time point
		ms_50 = ms50_count;
		us = TIM3->CNT;		
	}
	//LOW
	else
	{
		int temp;
		int tim_counter = TIM3->CNT; 
		//get pwm form history
		temp = (ms50_count - ms_50)*50000 + tim_counter - us;
		pwm_in[3] = temp;
		
		pwm_update_count++;
	}

}

void EXTI3_IRQHandler(void)
{
		static unsigned int ms_50;
	static unsigned int us;
	
	//clear interrupt flag
	EXTI_ClearITPendingBit(EXTI_Line3);
	
	//HIGHT
	if (GPIOA->IDR & GPIO_Pin_3)
	{
		//mark the start time point
		ms_50 = ms50_count;
		us = TIM3->CNT;		
	}
	//LOW
	else
	{
		int temp;
		int tim_counter = TIM3->CNT; 
		//get pwm form history
		temp = (ms50_count - ms_50)*50000 + tim_counter - us;
		pwm_in[2] = temp;
		
		pwm_update_count++;
	}

}

void EXTI4_IRQHandler(void)
{
		static unsigned int ms_50;
	static unsigned int us;
	
	//clear interrupt flag
	EXTI_ClearITPendingBit(EXTI_Line4);
	
	//HIGHT
	if (GPIOA->IDR & GPIO_Pin_4)
	{
		//mark the start time point
		ms_50 = ms50_count;
		us = TIM3->CNT;		
	}
	//LOW
	else
	{
		int temp;
		int tim_counter = TIM3->CNT; 
		//get pwm form history
		temp = (ms50_count - ms_50)*50000 + tim_counter - us;
		pwm_in[1] = temp;
		
		pwm_update_count++;
	}


}

void EXTI9_5_IRQHandler(void)
{	
	
	//exit5
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		static unsigned int ms_50;
		static unsigned int us;
		
		EXTI_ClearITPendingBit(EXTI_Line5);
		
		//HIGHT
		if (GPIOA->IDR & GPIO_Pin_5)
		{
			//mark the start time point
			ms_50 = ms50_count;
			us = TIM3->CNT;		
		}
		//LOW
		else
		{
			int temp;
			int tim_counter = TIM3->CNT; 
			//get pwm form history
			temp = (ms50_count - ms_50)*50000 + tim_counter - us;
			pwm_in[0] = temp;
			
		
			pwm_update_count++;
		}
	
	
	}
	
	//exit6
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		static unsigned int ms_50;
		static unsigned int us;		

		EXTI_ClearITPendingBit(EXTI_Line6);
		
		//HIGHT
		if (GPIOA->IDR & GPIO_Pin_6)
		{
			//mark the start time point
			ms_50 = ms50_count;
			us = TIM3->CNT;		
		}
		//LOW
		else
		{
			int temp;
			int tim_counter = TIM3->CNT; 
			//get pwm form history
			//temp = (ms50_count - ms_50)*50000 + tim_counter - us;
			//pwm_in[1] = temp;
			
			//pwm_update_count++;
		}
	
	}
//	
//	//exit7
//	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
//	{
//		static unsigned int ms_50;
//		static unsigned int us;		

//		EXTI_ClearITPendingBit(EXTI_Line7);
//		
//		//HIGHT
//		if (GPIOA->IDR & GPIO_Pin_7)
//		{
//			//mark the start time point
//			ms_50 = ms50_count;
//			us = TIM3->CNT;		
//		}
//		//LOW
//		else
//		{
//			int temp;
//			int tim_counter = TIM3->CNT; 
//			//get pwm form history
//			temp = (ms50_count - ms_50)*50000 + tim_counter - us;
//			pwm_in[0] = temp;
//		}		
//	
//	}

}




//every 200 ms to see if pwm signal is good
void timer_pwm_timeout(void *p)
{
	static unsigned int pwm_update_count_bef;
	
	if(pwm_update_count_bef == pwm_update_count)
	{
		static char led_value = 0;
		pwm_good_flag = 0;
		
		//flash led
		if(led_value)
			led_value = 0;
		else
			led_value = 1;
		LED(led_value);
	}
	else
	{
		pwm_good_flag = 1;
		LED(1);
	}
	pwm_update_count_bef = 	pwm_update_count;
}

void pwm_sw_init(void)
{
	rt_sem_init(&sem, "pwm",0 ,RT_IPC_FLAG_FIFO);
	
	rt_timer_init(&tim_pwm,
					"pwm", 				/*name*/
					timer_pwm_timeout, 				/* 超时函数*/
					RT_NULL,				/* 参数 */
					100, 					/* 定时长度 */
					RT_TIMER_FLAG_PERIODIC);/* 周期定时器*/	
	rt_timer_start(&tim_pwm);
}

//
void thread_pwm(void)
{	
	extern int can_message_send(CanTxMsg *TxMessage);
	float rang_bef = 0;
	CanTxMsg TxMessage;
	
	pwm_sw_init();
	pwm_hw_init();
	
	while(1)
	{	
		rt_thread_delay(10);
		
		//send to can bus only when PWM signal is good
		if(pwm_good_flag)
		{	
			//send to canbus message 1, voltage for each battery
			TxMessage.StdId = CAN_TYPE_MASTER | CAN_PPM | CAN_MESSAGE_INDEX0;
			TxMessage.ExtId = 0x00;
			TxMessage.RTR = CAN_RTR_DATA;
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.DLC = 8;
			
			//the first 4 channels
			TxMessage.Data[0] = pwm_in[0]>>8;
			TxMessage.Data[1] = pwm_in[0];

			TxMessage.Data[2] = pwm_in[1]>>8;
			TxMessage.Data[3] = pwm_in[1];

			TxMessage.Data[4] = pwm_in[2]>>8;
			TxMessage.Data[5] = pwm_in[2];

			TxMessage.Data[6] = pwm_in[3]>>8;
			TxMessage.Data[7] = pwm_in[3];
			
			//send to bus
			can_message_send(&TxMessage);	
			
			
			//send to canbus message 1, voltage for each battery
			TxMessage.StdId = CAN_TYPE_MASTER | CAN_PPM | CAN_MESSAGE_INDEX1;
			TxMessage.ExtId = 0x00;
			TxMessage.RTR = CAN_RTR_DATA;
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.DLC = 8;
			
			//the first 4 channels
			TxMessage.Data[0] = pwm_in[4]>>8;
			TxMessage.Data[1] = pwm_in[4];

			TxMessage.Data[2] = pwm_in[5]>>8;
			TxMessage.Data[3] = pwm_in[5];

			TxMessage.Data[4] = pwm_in[6]>>8;
			TxMessage.Data[5] = pwm_in[6];

			TxMessage.Data[6] = pwm_in[7]>>8;
			TxMessage.Data[7] = pwm_in[7];
			
			//send to bus
			can_message_send(&TxMessage);
		}		

	}

}





