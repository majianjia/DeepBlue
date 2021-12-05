
/*
 * File      : opticalflow.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.9.7    majianjia   the first version
 */
#include <rtthread.h>
#include "thread_main.h"
#include "stm32f10x.h"
#include "opticalflow.h"
#include "string.h"

#include "db_can_message.h"


#define SPI_WRITE	  0x00
#define SPI_READ	  0x80


#define SS_Pin		GPIO_Pin_4
#define SS_IO			GPIOA

#define CS(value) 					\
{										\
	if (value)							\
		GPIOC->BSRR = SS_Pin;		\
	else								\
		GPIOC->BRR  = SS_Pin;		\
}


void SPI1_send(unsigned char addr, unsigned char data)
{
	CS(0);

    SPI_I2S_SendData(SPI1, addr);	
	while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)) == RESET);

    SPI_I2S_SendData(SPI1, data);
	while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)) == RESET);

	SPI_I2S_ReceiveData(SPI1);
	while((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)) == RESET);

	CS(1);
	SPI_I2S_ReceiveData(SPI1);
}								   

unsigned char spi1_re(unsigned char addr)
{	
	addr = addr & 0xBF;
	
	CS(0);

    SPI_I2S_SendData(SPI1, addr);										

	while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)) == RESET);
	SPI_I2S_SendData(SPI1, 0x00);

	while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE))== RESET);
	SPI_I2S_ReceiveData(SPI1);

	while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE))== RESET);
   
	CS(1);
	return SPI_I2S_ReceiveData(SPI1) & 0xff;
}


void op_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	SPI_Cmd(SPI1, DISABLE);
	SPI_I2S_DeInit(SPI1);  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1| RCC_APB2Periph_GPIOA, ENABLE);
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;// 36M/8=4.00M 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, &SPI_InitStructure);
	SPI_CalculateCRC(SPI1, DISABLE);

    SPI_Cmd(SPI1, ENABLE);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void thread_op(void)
{
	op_init();
	
	while(1)
	{
		rt_thread_delay(1);
		SPI1_send(0x5A, 0X5A);
		
		
	}

}


