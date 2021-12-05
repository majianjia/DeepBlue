/*
 * File      : thread_canbus.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-16    majianjia   the first version
 */


#include "stm32f4xx.h"
#include <rtthread.h>

#include "struct_all.h"
#include "thread_canbus.h"
#include "time_measure.h"
#include "thread_expand.h"

#include "db_can_message.h"

/* 信号量控制块 */
static struct rt_semaphore sem_rx;

/* 消息队列 */
static struct rt_messagequeue mq;
static char msg_pool[1024];


//
CanTxMsg TxMessage;
CanRxMsg RxMessage;


//硬件
void canbus_hw_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN GPIOs configuration **************************************************/

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);

	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Configure CAN Enable pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//低电平，开启接口芯片
	GPIO_ResetBits(GPIOD, GPIO_Pin_2);
	
	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE; //
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;							//DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;			//CAN_Mode_LoopBack; //CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	/* CAN Baudrate = 0.5MBps*/
	//APB1 = 42M
	//Rate = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler; 
	// 0.5m = 42m / (1 + 10 + 3)/ 6 
	//that meas sample point is about 78% from the start
	//
	CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;//CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;//CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 6;//7;
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
	

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
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

/* thread_can_rx */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_rx_stack[1024];
static struct rt_thread thread_can_rx_handle;

void entry_thread_can_rx(void* parameter)
{
	while(1)
	{
		//wait for a new message
		rt_sem_take(&sem_rx, RT_WAITING_FOREVER);
		
		//
		switch(RxMessage.StdId & CAN_FILTER)
		{
			case CAN_AIR_SPEED: 
				air_speed_decode(RxMessage);
				break;
			
			case CAN_ULT_RANG: 
				ultrasonic_ranging_decode(RxMessage);
				break;
			
			case CAN_BATTERY:
				battery_decode(RxMessage);
				break;
			
			case CAN_PPM:
				ppm_decode(RxMessage);
				break;
			
			case CAN_OPT:
				opt_floaw_decode(RxMessage);
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
	
	//rx
	rt_thread_init(&thread_can_rx_handle,
                   "can_rx",
                   entry_thread_can_rx,
                   RT_NULL,
                   &thread_can_rx_stack[0],
                   sizeof(thread_can_rx_stack),10,1);
    rt_thread_startup(&thread_can_rx_handle);
}

/* thread_can_tx */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_tx_stack[1024];
static struct rt_thread thread_can_tx_handle;

void entry_thread_can_tx(void* parameter)
{
	while(1)
	{
		//等待发送新信息
		rt_mq_recv(&mq, &TxMessage, sizeof(CanTxMsg), RT_WAITING_FOREVER);
		
		//if all mailbox are no available
		while(CAN_Transmit(CAN1, &TxMessage) == CAN_TxStatus_NoMailBox)
		{
			rt_thread_delay(1);
		}

	}

}

//外部调用，发送信息
int can_message_send(CanTxMsg *TxMessage)
{
	return rt_mq_send(&mq, (void *)TxMessage,sizeof(CanTxMsg));
}
//紧急发送
int can_message_urgent(CanTxMsg *TxMessage)
{
	return rt_mq_urgent(&mq, (void *)TxMessage,sizeof(CanTxMsg));
}


/* 创建can_tx子线程 */
void thread_can_tx_init(void)
{
	/* 初始化消息队列 */
	rt_mq_init(&mq,
			   "can tx", 
				msg_pool, 
				sizeof(CanTxMsg), 
				sizeof(msg_pool),
				RT_IPC_FLAG_FIFO);
	
	//tx
	rt_thread_init(&thread_can_tx_handle,
                   "can_tx",
                   entry_thread_can_tx,
                   RT_NULL,
                   &thread_can_tx_stack[0],
                   sizeof(thread_can_tx_stack),10,1);
    rt_thread_startup(&thread_can_tx_handle);
}


//软件
void canbus_sw_init(void)
{
	thread_can_rx_init();
	thread_can_tx_init();
}

//初始化
void canbus_init(void)
{
	canbus_sw_init();
	canbus_hw_init();
}

//canbus 线程主体
void thread_canbus(void)
{
	//CanTxMsg TxMessage;
	canbus_init();
	
	while(1)
	{	
		rt_thread_delay(10);
	}
}
