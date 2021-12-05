/*
 * File      : thread_can.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.15    majianjia   the first version
 */
 
#include <rtthread.h>
#include "thread_can.h"
#include "stm32f10x.h"
#include "string.h"

#include "db_can_message.h"

#include "run.h"
#include "main.h"
#include "timer.h"
#include "pwm.h"


//define which esc is it
#define MOTO_NUMBER  0

#define CAN_FLASH_PAGE_SIZE			((uint16_t)0x400)
// use the last KB for ESC32 storage 
// and the previous kb for CAN ID
#define CAN_FLASH_WRITE_ADDR		(0x08000000 + (uint32_t)CAN_FLASH_PAGE_SIZE * 62)    

int CAN_ID = MOTO_NUMBER;
int CAN_CURRENT_ID = 0;


/* tx message buffer */
static struct rt_messagequeue mq;
static char msg_pool[1024];

//
unsigned int volatile systick = 0;
unsigned int volatile seconds = 0;
float cpu_usage;


/* 邮箱控制块 */
static struct rt_semaphore sem_rx;

//
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

//can 总线初始化
void can_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	/* Disable the Serial Wire Jtag Debug Port SWJ-DP */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);
	
	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remapping_CAN , ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;			//时间触发模式
	CAN_InitStructure.CAN_ABOM = ENABLE;			//
	CAN_InitStructure.CAN_AWUM = DISABLE;			//睡眠模式？
	CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;			//报文只发送一次
	CAN_InitStructure.CAN_RFLM = DISABLE;			// 是否覆盖已满的报文
	CAN_InitStructure.CAN_TXFP = DISABLE;			//发送优先级由报文决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* CAN Baudrate = 0.5MBps*/
	//APB = 32M
	//Rate = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler; 
	// 0.5m = 36m / (1 + 3 + 2)/ 12 
	//that meas sample point is about 75% from the start
	//
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 12;
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* Enable CAN1 RX0 interrupt IRQ channel */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	//打开接口芯片
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

//interrupt handle
void canbus_rx_handle(void)
{
   if(SET == CAN_GetITStatus(CAN1, CAN_IT_FMP0))
   {
	    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		rt_sem_release(&sem_rx);
   }

}



//can id write to flash
int can_id_to_flash(void) 
{
    FLASH_Status FLASHStatus;
    uint32_t address;
    int ret = 0;

    // Startup HSI clock
    RCC_HSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != SET);

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	if ((FLASHStatus = FLASH_ErasePage(CAN_FLASH_WRITE_ADDR)) == FLASH_COMPLETE) 
	{
		FLASH_ProgramWord(CAN_FLASH_WRITE_ADDR, CAN_ID);
	}

//    if ((FLASHStatus = FLASH_ErasePage(CAN_FLASH_WRITE_ADDR)) == FLASH_COMPLETE) 
//	{
//		address = 0;
//		while (FLASHStatus == FLASH_COMPLETE && address < sizeof(CAN_ID)) 
//		{
//			if ((FLASHStatus = FLASH_ProgramWord(CAN_FLASH_WRITE_ADDR + address, *(uint32_t *)((char *)CAN_ID + address))) != FLASH_COMPLETE)
//				break;

//			address += 4;
//		}

//		ret = 1;
//    }

    FLASH_Lock();

    // Shutdown HSI clock
    RCC_HSICmd(DISABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == SET)
		;

    return ret;
}

int can_id_from_flash(void)
{
	//memcpy((char *)CAN_CURRENT_ID, (char *)CAN_FLASH_WRITE_ADDR, sizeof(CAN_CURRENT_ID));
	
	CAN_CURRENT_ID = *((uint32_t*)CAN_FLASH_WRITE_ADDR);
	
	//if there is not id in flash
	if(CAN_CURRENT_ID == 0x00 || CAN_CURRENT_ID == 0xff)
		CAN_CURRENT_ID = 0;
		
	
	return CAN_CURRENT_ID;
}






void update_pwm_from_canbus(CanRxMsg rx)
{
	unsigned short pwm;
	extern volatile uint8_t state, inputMode;
	
	//if id is invalid
	if(CAN_CURRENT_ID == 0)
		return;
	
	//is the message an ctrl?
	if((rx.StdId & CAN_FILTER) == CAN_CTRL_GROUP0)	
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				{
					//get the pwm base on the moto number
					pwm = rx.Data[0+ (CAN_CURRENT_ID-1) *2] << 8 | rx.Data[1+ (CAN_CURRENT_ID-1) *2];
					
					if(inputMode == ESC_INPUT_PWM)
					{
						pwmValidMicros = timerMicros;//what the fuck is that?
						runNewInput(pwm); //send pwm to ESC32
					}
					
					
					
					break;
				}
			
			default:break;
		}

	}
}



/* thread_can_rx */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_rx_stack[1024];
static struct rt_thread thread_can_rx_handle;

void entry_thread_can_rx(void* parameter)
{
// 	rt_thread_delay(RT_TICK_PER_SECOND);
	while(1)
	{
		//wait for a new message
		rt_sem_take(&sem_rx, RT_WAITING_FOREVER);
		
		//
		switch(RxMessage.StdId & CAN_FILTER)
		{
			case CAN_TIME_BROADCAST: 
				
				break;
			
			case CAN_CTRL_GROUP0:
				update_pwm_from_canbus(RxMessage);
				
				break;
			
			default: break;
		}

	}
}

/* 创建can_rx子线程 */
void thread_can_rx_init(void)
{
	
	
	/* 初始化一个信号量 */
	rt_sem_init(&sem_rx, "can rx",0 ,RT_IPC_FLAG_FIFO);
	
	//mpu
	rt_thread_init(&thread_can_rx_handle,
                   "can_rx",
                   entry_thread_can_rx,
                   RT_NULL,
                   &thread_can_rx_stack[0],
                   sizeof(thread_can_rx_stack),4,1);
    rt_thread_startup(&thread_can_rx_handle);
}

/* thread_can_tx */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_tx_stack[512];
static struct rt_thread thread_can_tx_handle;

void entry_thread_can_tx(void* parameter)
{
	//rt_thread_delay(RT_TICK_PER_SECOND);
	while(1)
	{
		//wait for new message
		rt_mq_recv(&mq, &TxMessage, sizeof(CanTxMsg), RT_WAITING_FOREVER);
		
		//if all mailbox are no available
		if(CAN_Transmit(CAN1, &TxMessage) == CAN_TxStatus_NoMailBox)
		{
			rt_thread_delay(1);
		}


	}

}

//send 
int can_message_send(CanTxMsg *TxMessage)
{
	return rt_mq_send(&mq, (void *)TxMessage,sizeof(CanTxMsg));
}
//urgent send
int can_message_urgent(CanTxMsg *TxMessage)
{
	return rt_mq_urgent(&mq, (void *)TxMessage,sizeof(CanTxMsg));
}


/* 创建can_tx子线程 */
void thread_can_tx_init(void)
{
	rt_mq_init(&mq,
			   "can tx", 
				msg_pool, 
				sizeof(CanTxMsg), 
				sizeof(msg_pool),
				RT_IPC_FLAG_FIFO);
	
	rt_thread_init(&thread_can_tx_handle,
                   "can_tx",
                   entry_thread_can_tx,
                   RT_NULL,
                   &thread_can_tx_stack[0],
                   sizeof(thread_can_tx_stack),4,1);
    rt_thread_startup(&thread_can_tx_handle);
}


//硬件初始化
static void hw_init(void)
{
	can_init();
}


//软件初始化
static void sw_init(void)
{
	thread_can_rx_init();
	thread_can_tx_init();
}

void show_id_from_led(void)
{
	#define TIME_PART 100 //ms
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_3);
	GPIO_SetBits(GPIOB, GPIO_Pin_4);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	if(CAN_CURRENT_ID > 0 && CAN_CURRENT_ID*TIME_PART < RT_TICK_PER_SECOND)
	{
		//off
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
		//on
		while(rt_tick_get() < TIME_PART * CAN_CURRENT_ID+1) ;
		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
		//off
		while(rt_tick_get() < TIME_PART * (CAN_CURRENT_ID+2));
		GPIO_SetBits(GPIOB, GPIO_Pin_3);
	}
	
	//wait till 1s
	while(rt_tick_get() < RT_TICK_PER_SECOND)
		;

}


void thread_can(void)
{
	unsigned char major, minor;

	extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
	extern void cpu_usage_init(void);
	extern void esc32_start_up(void);
	extern void rccInit(void) ;
	
	cpu_usage_init();
	
	//set rcc used the function from ESC32
//	rccInit();
	
	//Write can ID to flash 1 time programed
//	can_id_to_flash();
	
	//read can id from flash, every time to setup
	can_id_from_flash();
	
	//wait for 1s then start up esc32
	show_id_from_led();
	esc32_start_up();
	
	//软件初始化
	sw_init();
	
	//初始化硬件
	hw_init();
	
	while(1)
	{
		rt_thread_delay(1);
		systick = rt_tick_get();
		if(systick % 1000 == 0)
		{
			seconds++;
			
			cpu_usage_get(&major, &minor);
			cpu_usage = major + (float)minor/100.f;
		}
		
		//report rpm
		if(systick % 5 == 0)
		{
//			CanTxMsg TxMessage;
//			unsigned short int temp16;
//					
//			//send to canbus message 0, general battery status
//			TxMessage.StdId = CAN_TYPE_MASTER | CAN_BATTERY | CAN_MESSAGE_INDEX0;
//			TxMessage.ExtId = 0x00;
//			TxMessage.RTR = CAN_RTR_DATA;
//			TxMessage.IDE = CAN_ID_STD;
//			TxMessage.DLC = 8;
//			
//			//the first battery
//			TxMessage.Data[0] = 1;

//			//send out
//			can_message_send(&TxMessage);	
		
		}
		
		
		
		
	}


}
