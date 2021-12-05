/*
 * File      : thread_main.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.15    majianjia   the first version
 */
 
#include <rtthread.h>
#include "thread_main.h"
#include "stm32f10x.h"
#include "string.h"

#include "db_can_message.h"

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

	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;			//时间触发模式
	CAN_InitStructure.CAN_ABOM = DISABLE;			//
	CAN_InitStructure.CAN_AWUM = DISABLE;			//睡眠模式？
	CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;			//报文只发送一次
	CAN_InitStructure.CAN_RFLM = DISABLE;			// 是否覆盖已满的报文
	CAN_InitStructure.CAN_TXFP = DISABLE;			//发送优先级由报文决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	/* CAN Baudrate = 0.5MBps*/
	//APB = 36M
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
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_TIME_BROADCAST << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_FILTER << 5;//左对齐
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
static char thread_can_tx_stack[1024];
static struct rt_thread thread_can_tx_handle;

void entry_thread_can_tx(void* parameter)
{
	rt_thread_delay(RT_TICK_PER_SECOND);
	
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


//
struct rt_timer timer;
struct rt_semaphore sem_beep;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
 GPIO_InitTypeDef GPIO_InitStructure;

void beep_timeout(void *p)
{
	TIM_Cmd(TIM3, DISABLE);  
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	//off
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	
	rt_sem_release(&sem_beep);
}

void beep_setup(void)
{
   

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);        
	  
	/* Disable the Serial Wire Jtag Debug Port SWJ-DP */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_4);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	
	        
	/* Time base configuration */                 
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); 
	   
	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;            //
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;        
	TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period/2;           
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);         
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//TIM_Cmd(TIM3, ENABLE);      
	
	//time out timer init
	rt_sem_init(&sem_beep, "beep", 1, RT_IPC_FLAG_FIFO);
	rt_timer_init(&timer, "beep", beep_timeout, RT_NULL, 1, RT_TIMER_FLAG_ONE_SHOT);

}

//tune 			1 		2		3		4		5		6		7		
//frequency		1046	1175	1319	1397	1568	1760	1975
//period		956		851		758		715		637		568		506
void beep(unsigned int tune, unsigned int time)
{
	unsigned int period;
	
	//if take timer failure
	if(rt_sem_take(&sem_beep, RT_TICK_PER_SECOND) == -RT_ETIMEOUT)
		return;
	
	//set timeout 
	rt_timer_control(&timer, RT_TIMER_CTRL_SET_TIME, &time);
	rt_timer_start(&timer);
	
	//set tune
	switch(tune)
	{
		case 1: period = 956; break;
		case 2: period = 851; break;
		case 3: period = 758; break;
		case 4: period = 715; break;
		case 5: period = 637; break;
		case 6: period = 568; break;
		case 7: period = 506; break;
		default:period = 2000; break;
	}
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//init pwm
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_OCInitStructure.TIM_Pulse = period/2;  
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//start
	TIM_Cmd(TIM3, ENABLE);      
	
}

//硬件初始化
static void hw_init(void)
{
	can_init();
	beep_setup();
}


//软件初始化
static void sw_init(void)
{
	thread_can_rx_init();
	thread_can_tx_init();
}


void thread_main(void)
{
	unsigned char major, minor;

	extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
	
	//软件初始化
	sw_init();
	
	//初始化硬件
	hw_init();
	//
	beep(1, 100);
	beep(2, 100);
	beep(3, 100);
	beep(2, 100);
	beep(1, 100);
	
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
	}


}
