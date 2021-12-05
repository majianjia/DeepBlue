/* #include "stm32f4_i2c.h" */

#ifndef __STM32F4_I2C_H
#define __STM32F4_I2C_H


#define u32 unsigned int
#define u8 unsigned char

/*=====================================================================================================*/
/*=====================================================================================================*/
#define I2C_TIME		((u32)65535)
#define I2C1_SPEED	    ((u32)400000)
/*=====================================================================================================*/
/*=====================================================================================================*/

#include "rtthread.h"
#ifdef RT_USING_MUTEX
extern rt_err_t i2c1_take(int flag);
extern rt_err_t i2c1_release(void);
extern void i2c1_mutex_init(void);
#endif

extern void I2C_Config(void);
extern u32 I2C_DMA_Read( u8* ReadBuf, u8 SlaveAddr, u8 ReadAddr, u8* NumByte );
extern u32 I2C_DMA_ReadReg( u8* ReadBuf, u8 SlaveAddr, u8 ReadAddr, u8 NumByte );
extern u32 I2C_DMA_Write( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8* NumByte );
extern u32 I2C_DMA_WriteReg( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8 NumByte );
extern void I2C1_Send_DMA_IRQ( void );
extern void I2C1_Recv_DMA_IRQ( void );
extern u32 I2C_TimeOut( void );
/*=====================================================================================================*/
/*=====================================================================================================*/

#endif
