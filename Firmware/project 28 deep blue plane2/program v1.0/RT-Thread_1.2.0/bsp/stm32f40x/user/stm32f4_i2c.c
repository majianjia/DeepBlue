/*=====================================================================================================*/
/*=====================================================================================================*/
#include "stm32f4xx.h"
#include "stm32f4_i2c.h"
#include "rtthread.h"

//RTT mutex surpported
#ifdef RT_USING_MUTEX

static struct rt_mutex i2c1_mutex;

void i2c1_mutex_init(void)
{
	//initial the mutex with priority
	rt_mutex_init(&i2c1_mutex,"i2c1", RT_IPC_FLAG_PRIO);//PRIO);
}

rt_err_t i2c1_take(int flag)
{
	//flag means waiting time
	//RT_WAITTING_FOREVER
	return rt_mutex_take(&i2c1_mutex, flag);
}

rt_err_t i2c1_release(void)
{
	return rt_mutex_release(&i2c1_mutex);
}

#endif



/*=====================================================================================================*/
/*=====================================================================================================*/
#define I2C1_DR_Address	((u32)0x40005410)
/*=====================================================================================================*/
/*=====================================================================================================*/
vu8* I2C_ReadPtr;
vu8* I2C_WritePtr;
vu32 I2C_TimeCnt = I2C_TIME;

unsigned int volatile i2c_time_out_count = 0;
/*=====================================================================================================*/
/*=====================================================================================================*/
DMA_InitTypeDef DMA_InitStruct;
/*=====================================================================================================*/
/*=====================================================================================================*/
void I2C_Config( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);  

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_TEIF6 |
															DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6);
															
// 	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 |
// 															DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
	
	DMA_Cmd(DMA1_Stream6, DISABLE);
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStruct.DMA_Channel = DMA_Channel_1;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)0;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = 0xFFFF;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);

	//DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_FEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_TEIF0 |
	//														DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
	
	DMA_Cmd(DMA1_Stream0, DISABLE);
	DMA_DeInit(DMA1_Stream0);
	DMA_InitStruct.DMA_Channel = DMA_Channel_1;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)0;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = 0xFFFF;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream0, &DMA_InitStruct);

	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);

	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_16_9;//I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x55;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = I2C1_SPEED;
	I2C_Init(I2C1, &I2C_InitStruct);
	
	//Çå³ýÃ¦±êÖ¾
// 	I2C1->CR1|=0x8000;             // reset i2c lines,disable i2c
// 	I2C1->CR1&=~0x8000;             // reset i2c lines,enable i2c
	
	I2C_Cmd(I2C1, ENABLE);
	I2C_DMACmd(I2C1, ENABLE);
	
}
/*=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_Read( u8* ReadBuf, u8 SlaveAddr, u8 ReadAddr, u8* NumByte )
{
	I2C_ReadPtr = NumByte;

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_SendData(I2C1, ReadAddr);

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF) == RESET)
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);

	if((u16)(*NumByte) < 2) {
		I2C_TimeCnt = I2C_TIME;
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		I2C_AcknowledgeConfig(I2C1, DISABLE);
		(void)I2C1->SR2;

		I2C_GenerateSTOP(I2C1, ENABLE);

		I2C_TimeCnt = I2C_TIME;
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		*ReadBuf = I2C_ReceiveData(I2C1);

		(u16)(*NumByte)--;

		I2C_TimeCnt = I2C_TIME;
		while(I2C1->CR1 & I2C_CR1_STOP)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		I2C_AcknowledgeConfig(I2C1, ENABLE);
	}
	else {
		I2C_TimeCnt = I2C_TIME;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		DMA_InitStruct.DMA_Channel = DMA_Channel_1;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
		DMA_InitStruct.DMA_Memory0BaseAddr = (u32)ReadBuf;
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStruct.DMA_BufferSize = (u32)(*NumByte);
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream0, &DMA_InitStruct);

		I2C_DMALastTransferCmd(I2C1, ENABLE);

		DMA_Cmd(DMA1_Stream0, ENABLE);
	}

	I2C_TimeCnt = I2C_TIME;
	while(*NumByte > 0)
	{
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();
		rt_thread_delay(1);
	}

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_ReadReg( u8* ReadBuf, u8 SlaveAddr, u8 ReadAddr, u8 NumByte )
{
	I2C_DMA_Read(ReadBuf, SlaveAddr, ReadAddr, (u8*)(&NumByte));

	I2C_TimeCnt = I2C_TIME;
	while(NumByte > 0)
	{
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();
		rt_thread_delay(1);
	}

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_Write( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8* NumByte )
{
	I2C_WritePtr = NumByte;

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	I2C_TimeCnt = I2C_TIME;
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();

	I2C_SendData(I2C1, WriteAddr);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
		if((I2C_TimeCnt--) == 0) return I2C_TimeOut();
	
	//if send byte = 0 then return and finished the transfer
	if((* NumByte)<1)
	{
		I2C_GenerateSTOP(I2C1, ENABLE);
		return 0;
	}

	DMA_InitStruct.DMA_Channel = DMA_Channel_1;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)WriteBuf;
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStruct.DMA_BufferSize = (u32)(*NumByte);
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);

	DMA_Cmd(DMA1_Stream6, ENABLE);

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_WriteReg( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8 NumByte )
{
	I2C_DMA_Write(WriteBuf, SlaveAddr, WriteAddr, (u8*)(&NumByte));

	I2C_TimeCnt = I2C_TIME;
	while(NumByte > 0)
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void I2C1_Send_DMA_IRQ( void )
{
	if(DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) != RESET) {
		DMA_Cmd(DMA1_Stream6, DISABLE);
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);

		I2C_TimeCnt = I2C_TIME;
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF))
			if((I2C_TimeCnt--) == 0) I2C_TimeOut();

		I2C_GenerateSTOP(I2C1, ENABLE);
		*I2C_WritePtr = 0;
	}
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void I2C1_Recv_DMA_IRQ( void )
{
	if(DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0) != RESET) {
		I2C_GenerateSTOP(I2C1, ENABLE);
		DMA_Cmd(DMA1_Stream0, DISABLE);
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
		*I2C_ReadPtr = 0;
	}
}
/*=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_TimeOut( void )
{
	//send 9 clock on the bus
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	//restart the bus
	I2C_Cmd(I2C1, DISABLE);
	//reconfig
	I2C_Config();

	//to mark the time out times
	i2c_time_out_count++;
	return 0;
}
/*=====================================================================================================*/
/*=====================================================================================================*/









