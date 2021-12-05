/*
 * File      : ultrasonic_ranging.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.15    majianjia   the first version
 */

#include "stm32f10x.h"
#include "ultrasonic_ranging.h"
#include "rtthread.h"

unsigned int ur_time;	//ʱ��
float rang;		//����
float speed;

//sem
static struct rt_semaphore sem;

//��ʱ�����ƿ�
static struct rt_timer tim_ur;


static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//
void ur_hw_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;      	

	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	// ������GPIO��ʱ�� �˿ڸ���ʱ�ӣ��ⲿ�жϣ�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	//����
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;            
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;   //pull downIN_FLOATING ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	//�����ź�
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;            
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;   	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	//�趨��ʱ��
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;

	TIM_TimeBaseStructure.TIM_Prescaler = 64 -1 ;	
	TIM_TimeBaseStructure.TIM_ClockDivision =  0x0000;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//TIM_Cmd(TIM3, ENABLE);
	
	//����жϱ�־
	EXTI_ClearITPendingBit(EXTI_Line0);

	//���ܿ�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    	//��1ͨ���������½�
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		

	//
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	  		//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

//�жϺ���
void exit0_handle(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0); 
	
	//�ߵ�ƽ
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
	{
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		TIM_Cmd(TIM3, ENABLE);
	}
	else
	{
		//��¼���ݺ��ͷ��ź���
		ur_time = TIM3->CNT;
		rt_sem_release(&sem);
	}


}
//�жϺ���
void EXTI0_IRQHandler(void)
{
	exit0_handle();
}

static void delay( int t)
{
	while(t>0)
		t--;
}


//�����ʱ�����Ͳ����ź�
void timer_ur_timeout(void *p)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	delay(10*20); //��Լ20us
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
}

void ur_sw_init(void)
{
	rt_sem_init(&sem, "ur",0 ,RT_IPC_FLAG_FIFO);
	
	rt_timer_init(&tim_ur,
					"ur", 				/*name*/
					timer_ur_timeout, 				/* ��ʱ����*/
					RT_NULL,				/* ���� */
					50, 					/* ��ʱ���� */
					RT_TIMER_FLAG_PERIODIC);/* ���ڶ�ʱ��*/	
	rt_timer_start(&tim_ur);
}



//multiple axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    5	 //1s
#define  FILTER_CHANNEL		2

static void sliding_windows_filter(float in[FILTER_CHANNEL], float out[FILTER_CHANNEL])
{
	static float buf[FILTER_WINDOWS][FILTER_CHANNEL];
	static int buf_pointer = 0;
	int weight = FILTER_WINDOWS;
	int count = FILTER_WINDOWS;
	int ch_count = FILTER_CHANNEL;
	int index = buf_pointer;
	int total_weight = 0;
	float integral[FILTER_CHANNEL] = {0};
	
	//add the newest date to the buffer
	ch_count = FILTER_CHANNEL;
	while(ch_count>0)
	{
		ch_count--;
		buf[index][ch_count] = in[ch_count];
	}


	//filter
	while(count--)
	{
		ch_count = FILTER_CHANNEL;
		while(ch_count>0)
		{
			ch_count--;
			integral[ch_count] += buf[index][ch_count] * weight;
		}
		
		total_weight += weight; //the newest data have the most significant weight
		weight--;				
				
		index++;
		if(index >= FILTER_WINDOWS)
			index = 0;
	}
	buf_pointer ++;
	if(buf_pointer >= FILTER_WINDOWS)
		buf_pointer = 0;
	
	//update the newest date
	ch_count = FILTER_CHANNEL;
	while(ch_count>0)
	{
		ch_count--;
		out[ch_count] = (integral[ch_count] / total_weight);
	}
}

//
void thread_ur(void)
{	
	extern void can_message_send(void);

	float rang_bef = 0;
	float in[2];
	float out[2];
	//unsigned int tick;
	
	ur_sw_init();
	ur_hw_init();
	
	while(1)
	{	
		//start new measurement
		//timer_ur_timeout(RT_NULL);
		
		//if time out get new data
		rt_sem_take(&sem, 1000);
		
		//get lenght
		in[0] = (float)ur_time/1000000.f * 340 / 2;
		//get speed
		in[1] = (in[0] - rang_bef)/ 0.05f;
		
		//filter
		sliding_windows_filter(in, out);

		//get data after filter
		rang = out[0];
		speed = out[1];
		
		//mark last rang
		rang_bef = rang;
	
		//send to canbus
		can_message_send();
		

	}

}





