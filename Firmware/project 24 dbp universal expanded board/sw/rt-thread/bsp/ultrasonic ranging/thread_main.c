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
#include "ultrasonic_ranging.h"
#include "db_can_message.h"
#include "string.h"
//
unsigned int volatile systick = 0;
unsigned int volatile seconds = 0;


/* ������ƿ� */
static struct rt_semaphore sem_rx;
static struct rt_semaphore sem_tx;

//
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

//LED�ܽŶ���
#define LED(value) 						\
{										\
	if (value)							\
		GPIOA->BSRR = GPIO_Pin_15;		\
	else								\
		GPIOA->BRR  = GPIO_Pin_15;		\
}


//ʱ�ӳ�ʼ�� �ڲ�ʱ�� ��Ƶ�� 64M
static void rcc_init(void)
{
  	//������ RCC�Ĵ�������Ϊȱʡֵ
  	RCC_DeInit();

	//����ʱ��ΪHSI
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
   
    	//���� PLL ʱ��Դ����Ƶϵ��		64MHZ
    	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
              
    	//ʹ�ܻ���ʧ�� PLL,�����������ȡ��ENABLE����DISABLE
    	RCC_PLLCmd(ENABLE);
    	//�ȴ�ָ���� RCC ��־λ���óɹ� �ȴ�PLL��ʼ���ɹ�
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

    	//����ϵͳʱ�ӣ�SYSCLK�� ����PLLΪϵͳʱ��Դ
    	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  
    	//�ȴ�PLL�ɹ�������ϵͳʱ�ӵ�ʱ��Դ
    	//  0x00��HSI ��Ϊϵͳʱ��
    	//  0x04��HSE��Ϊϵͳʱ��
    	//  0x08��PLL��Ϊϵͳʱ��  
    	while(RCC_GetSYSCLKSource() != 0x08);
  	}

	//�����ں�ʱ��
	SystemCoreClockUpdate();
}

//GPIOinit
static void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;      		

	// ������GPIO��ʱ�� �˿ڸ���ʱ�ӣ��ⲿ�жϣ�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	//�ر�JTAG ��SWD
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	//LED
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15 ;            
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);       		

}
/* �жϿ�������ʼ�� */
void nvic_init(void)
{ 
	//2bit ��ռ���ȼ� 2bit�����ȼ�
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}


//can ���߳�ʼ��
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
	CAN_InitStructure.CAN_TTCM = DISABLE;			//ʱ�䴥��ģʽ
	CAN_InitStructure.CAN_ABOM = DISABLE;			//
	CAN_InitStructure.CAN_AWUM = DISABLE;			//˯��ģʽ��
	CAN_InitStructure.CAN_NART = ENABLE;//DISABLE;			//����ֻ����һ��
	CAN_InitStructure.CAN_RFLM = DISABLE;			// �Ƿ񸲸������ı���
	CAN_InitStructure.CAN_TXFP = DISABLE;			//�������ȼ��ɱ��ľ���
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
	CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_TIME_BROADCAST << 5;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CAN_FILTER << 5;//�����
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

	//�򿪽ӿ�оƬ
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
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
			case CAN_TIME_BROADCAST: 
				
				break;
			
			default: break;
		}

	}
}

/* ����can_rx���߳� */
void thread_can_rx_init(void)
{
	/* ��ʼ��һ���ź��� */
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
		rt_sem_take(&sem_tx, RT_WAITING_FOREVER);
		
		TxMessage.StdId = CAN_TYPE_SLAVER | CAN_ULT_RANG | CAN_MESSAGE_INDEX0;
		TxMessage.ExtId = 0x00;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.DLC = 8;		
		
		memcpy(&TxMessage.Data[0], &rang, 4);
		memcpy(&TxMessage.Data[4], &speed, 4);
		CAN_Transmit(CAN1, &TxMessage);
	}

}
void can_message_send(void)
{
	rt_sem_release(&sem_tx);
}


/* ����can_tx���߳� */
void thread_can_tx_init(void)
{
	rt_sem_init(&sem_tx, "tx", 0,RT_IPC_FLAG_FIFO);
	
	rt_thread_init(&thread_can_tx_handle,
                   "can_tx",
                   entry_thread_can_tx,
                   RT_NULL,
                   &thread_can_tx_stack[0],
                   sizeof(thread_can_tx_stack),4,1);
    rt_thread_startup(&thread_can_tx_handle);
}

//��ʼ��
static void sw_init(void)
{
	thread_can_rx_init();
	thread_can_tx_init();
	
}


//Ӳ����ʼ��
static void hw_init(void)
{
	nvic_init();
	rcc_init();
	gpio_init();
	can_init();
}


//�����̳߳�ʼ��

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t ur_stack[1024];
static struct rt_thread ur_thread;
static void ur_thread_entry(void* parameter)
{
	thread_ur();
}
void thread_ur_init(void)
{
    /* init main thread */
	rt_thread_init(&ur_thread,
		"ult rang",
		ur_thread_entry, 
		RT_NULL,
		(rt_uint8_t*)&ur_stack[0], 
		sizeof(ur_stack), 6, 1);
    rt_thread_startup(&ur_thread);
}



void thread_main(void)
{
	sw_init();
	
	//��ʼ��Ӳ��
	hw_init();
	
	//��ʼ�������߳�
	thread_ur_init();
	
	while(1)
	{
		rt_thread_delay(1);
		systick = rt_tick_get();
		if(systick % 1000 == 0)
			seconds++;
	}


}