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
#include "opticalflow.h"

#include "db_can_message.h"

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

//LED管脚定义
#define LED(value) 						\
{										\
	if (value)							\
		GPIOA->BSRR = GPIO_Pin_15;		\
	else								\
		GPIOA->BRR  = GPIO_Pin_15;		\
}


//时钟初始化 内部时钟 倍频到 64M
static void rcc_init(void)
{
  	//将外设 RCC寄存器重设为缺省值
  	RCC_DeInit();

	//设置时钟为HSI
	SystemInit();

	RCC_HSICmd(ENABLE);
  
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

  	if(1)
  	{
    	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    	FLASH_SetLatency(FLASH_Latency_2);
   
    	RCC_HCLKConfig(RCC_SYSCLK_Div1);
   
    	RCC_PCLK2Config(RCC_HCLK_Div1);
  
    	RCC_PCLK1Config(RCC_HCLK_Div2);
   
    	//设置 PLL 时钟源及倍频系数		64MHZ
    	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
              
    	//使能或者失能 PLL,这个参数可以取：ENABLE或者DISABLE
    	RCC_PLLCmd(ENABLE);
    	//等待指定的 RCC 标志位设置成功 等待PLL初始化成功
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    	//设置系统时钟（SYSCLK） 设置PLL为系统时钟源
    	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
    	//等待PLL成功用作于系统时钟的时钟源
    	//  0x00：HSI 作为系统时钟
    	//  0x04：HSE作为系统时钟
    	//  0x08：PLL作为系统时钟  
    	while(RCC_GetSYSCLKSource() != 0x08);
  	}

	//更新内核时钟
	SystemCoreClockUpdate();
}

//GPIOinit
static void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;      		

	// 打开所有GPIO口时钟 端口复用时钟（外部中断）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	//关闭JTAG 打开SWD
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	//LED
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15 ;            
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);       		

}
/* 中断控制器初始化 */
void nvic_init(void)
{ 
	//2bit 抢占优先级 2bit副优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}


//can 总线初始化
void can_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
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

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
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
	//APB = 32M
	//Rate = RCC_APB1PeriphClock/CAN_SJW+CAN_BS1+CAN_BS2/CAN_Prescaler; 
	// 0.5m = 32m / (1 + 11 + 4)/ 4 
	//that meas sample point is about 75% from the start
	//
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
	CAN_InitStructure.CAN_Prescaler = 4;
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
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

//interrupt handle
void canbus_rx_handle(void)
{
   if(SET == CAN_GetITStatus(CAN1, CAN_IT_FMP0))
   {
        //CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	   
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
	while(1)
	{
		//之前校准，不发送数据
		if(seconds < 5)
		{
			rt_thread_delay(500);
			continue;
		}
			
// 		//发送空速，压差值
 		rt_thread_delay(5);
// 		TxMessage.StdId = CAN_TYPE_SLAVER | CAN_AIR_SPEED | CAN_MESSAGE_INDEX0;
// 		TxMessage.ExtId = 0x00;
// 		TxMessage.RTR = CAN_RTR_DATA;
// 		TxMessage.IDE = CAN_ID_STD;
// 		TxMessage.DLC = 8;
// 		
// 		memcpy(&TxMessage.Data[0], &p_diff, 4);
// 		memcpy(&TxMessage.Data[4], &air_speed, 4);
// 		CAN_Transmit(CAN1, &TxMessage);
// 		
// 		
// 		//发送测试数据
// 		rt_thread_delay(5);	
// 		TxMessage.StdId = CAN_TYPE_SLAVER | CAN_AIR_SPEED | CAN_MESSAGE_INDEX1;
// 		TxMessage.ExtId = 0x00;
// 		TxMessage.RTR = CAN_RTR_DATA;
// 		TxMessage.IDE = CAN_ID_STD;
// 		TxMessage.DLC = 4;		

// 		memcpy(TxMessage.Data, &rt_air_speed, 4);
// 		CAN_Transmit(CAN1, &TxMessage);
	}

}


/* 创建can_tx子线程 */
void thread_can_tx_init(void)
{
	rt_thread_init(&thread_can_tx_handle,
                   "can_tx",
                   entry_thread_can_tx,
                   RT_NULL,
                   &thread_can_tx_stack[0],
                   sizeof(thread_can_tx_stack),4,1);
    rt_thread_startup(&thread_can_tx_handle);
}





//功能线程初始化

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t op_stack[4096];
static struct rt_thread op_thread;
static void op_thread_entry(void* parameter)
{
	thread_op();
}
void thread_as_init(void)
{
    /* init main thread */
	rt_thread_init(&op_thread,
		"optical flow",
		op_thread_entry, 
		RT_NULL,
		(rt_uint8_t*)&op_stack[0], 
		sizeof(op_stack), 6, 1);
    rt_thread_startup(&op_thread);
}


//硬件初始化
static void hw_init(void)
{
	nvic_init();
	rcc_init();
	gpio_init();
	can_init();
}


//硬件初始化
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
	
	//初始化功能线程
	thread_as_init();
	
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
