
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
#include "math.h"

extern unsigned char ADNS_SROM[];

float H = 0 ;
float pitch;
float roll;

#define SPI_WRITE	  0x00
#define SPI_READ	  0x80


#define SS_Pin		GPIO_Pin_4
#define SS_IO			GPIOA

#define CS(value) 					\
{										\
	if (value)							\
		SS_IO->BSRR = SS_Pin;		\
	else								\
		SS_IO->BRR  = SS_Pin;		\
}

void delay_us(unsigned int us)
{
	int cl = 72;
	while(us--)
	{
		while((cl--)> 0);
	}

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

char SPI_SendReceive(char data)     //SPI1的收发
{		  
  	  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);  //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET); 
      SPI_I2S_SendData(SPI1, data);	
	  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	  return  SPI_I2S_ReceiveData(SPI1);

} 

unsigned char read_busy(void)//写帧率的判忙  ==1忙
{
	unsigned char temp;
	CS(0);
	temp=SPI_SendReceive(Extended_Config+0x00);
	delay_us(75);
	temp=SPI_SendReceive(0xff);
	temp&=0x80;
	CS(1);
	return temp;
}

void write_register(u8 adress,u8 vlue)
{
	CS(0);
	SPI_SendReceive(adress+0x80);
	SPI_SendReceive(vlue);
	CS(1);
	delay_us(75);
}
unsigned char read_register(u8 adress)
{
	unsigned char temp;
	CS(0);
	temp = SPI_SendReceive(adress+0x00);	
	delay_us(75);
	temp = SPI_SendReceive(0xff);	
	CS(1);
	return temp;
}

void clear_motion(void)
{
	CS(0);
	SPI_SendReceive(Motion_Clear+0x80);
	SPI_SendReceive(0xff);	//清除X Y数据
	CS(1);
}

void write_srom(void)
{
	int i;
	CS(0); 
	write_register(0x20,0x44);
	delay_us(51);
	write_register(0x23,0x07);
	delay_us(51);
	write_register(0x24,0x88);
	delay_us(51);
	CS(1);  //突发_写模式
	delay_us(340);//等待大于1帧时间
	CS(0); 
	write_register(SROM_Enable,0x18);
	CS(1);  //突发_写模式
	delay_us(41);//  >40us
	CS(0);
	for(i=0;i<=1985;i++)
	{
		write_register(0x60,ADNS_SROM[i]);
		delay_us(11);// >10us
	}
	CS(0);
	delay_us(105);	//>104us
}

void ADNS_Configuration(void)
{
	 CS(0); 
	 write_register(Configuration_bits,0x10);		//设置分辨率 1600	 //若Bit 4为0，则为400点每英寸
	 rt_thread_delay(3);
	 write_register(Extended_Config,0x01);
	 rt_thread_delay(3);
	if(read_busy()!=1)
	{  							      //设为3000帧每秒
		CS(1);  //突发_写模式
		rt_thread_delay(2);
		CS(0);	
//		SPI_SendReceive(Frame_Period_Max_Bound_Lower+0x80);	//设置帧率 //先写低位再写高位
//		SPI_SendReceive(0x40); //   C0 5000帧率	   
//		SPI_SendReceive(Frame_Period_Max_Bound_Upper+0x80);
//		SPI_SendReceive(0x1f);	 // 12
		SPI_SendReceive(Frame_Period_Max_Bound_Lower+0x80);	//设置帧率 //先写低位再写高位
		SPI_SendReceive(0xE0); //   C0 2000帧率	   
		SPI_SendReceive(Frame_Period_Max_Bound_Upper+0x80);
		SPI_SendReceive(0x2e);	 // 12

	} 
	clear_motion();
	CS(1);
}

float SumX;
float SumY;
float sum_x,sum_y;
void Read_Data_burst(void)
{
	unsigned char move=0;
	int  x=0;
	int  y=0;
	//burst读。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。
	CS(0);
	SPI_SendReceive(0x50);   
	delay_us(75);
	move=SPI_SendReceive(0xFF);             
	x=SPI_SendReceive(0xFF);
	y=SPI_SendReceive(0xFF);
	
	if(x&0x80)
	{
		//x的二补码转换	
		x -= 1;
		x = ~x;	
		x=(-1)*x;
		x-=256;
	}
	if(y&0x80)
	{
		//y的二补码转换	
		y -= 1;
		y = ~y;	
		y=(-1)*y;
		y-=256;
	} 
//	SumX=SumX+x;             //累加X读入的移动数据
//	SumY=SumY+y;			 //累加Y读入的移动数据
	
	CS(1);
	delay_us(4);
	CS(1);
//	sum_x=(25.4*(float)SumX *H)/(12*1600);//距离=d_x*(25.4/1600)*n   其中n=像高:物高=8毫米:物长
//	sum_y=(25.4*(float)SumY *H)/(12*1600);
	
	if(H > 0.2)
	{
		#define VIEW_ANGLE 70
		#define RES_SCALAR (1)
		float delta_x = 0;
		float delta_y = 0;
		float exp_x = 0;
		float exp_y = 0;
		static float pre_roll = 0;
		static float pre_pitch = 0;
		
//		exp_x = (roll-pre_roll)/(VIEW_ANGLE) * (x); 
//		exp_y = (pitch-pre_pitch)/(VIEW_ANGLE) * (y); 
//		pre_roll = roll;
//		pre_pitch = pitch;
		
//		delta_x = ((x-exp_x) * H)/(1.f) * 2.0 * 0.5447358;
//		delta_y = ((y-exp_y) * H)/(1.f) * 2.0 * 0.5447358;
		
//		exp_x = (roll)/(VIEW_ANGLE) * (1); 
//		exp_y = (pitch)/(VIEW_ANGLE) * (1); 
//		pre_roll = roll;
//		pre_pitch = pitch;

//		delta_x = (x) * H;
//		delta_y = (y) * H;
//		
//		SumX += delta_x;
//		SumY += delta_y;
//		
//		sum_x = SumX - roll*H;
//		sum_y = SumY - (-pitch*H);


		delta_x = ((x * H) / (float)RES_SCALAR) * 2.f *  tan((double)(VIEW_ANGLE/180.0*3.14159/2.0));
		delta_y = ((y * H) / (float)RES_SCALAR) * 2.f *  tan((double)(VIEW_ANGLE/180.0*3.14159/2.0));
		
		SumX += delta_x;
		SumY += delta_y;
		
//		sum_x = SumX - sin(roll/180.0 * 3.14159)*(float)RES_SCALAR * H ;
//		sum_y = SumY - sin(pitch/180.0 * 3.14159)*(float)RES_SCALAR * H;
		
		sum_x = SumX - roll*(float)RES_SCALAR * H * 8;
		sum_y = SumY - (-pitch)*(float)RES_SCALAR * H *8;
		
	}

}

void ultrasonic_ranging_decode(CanRxMsg rx)
{
	//is the message an ult rang ?
	if((rx.StdId & CAN_FILTER) == CAN_ULT_RANG)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				rt_memcpy(&(H), rx.Data, sizeof(H));

			break;

			
			default :break;			
		}
	}
	else
	{
	}

}

void altitude_angle_decode(CanRxMsg rx)
{
	//is the message an ult rang ?
	if((rx.StdId & CAN_FILTER) == CAN_ALT_ANGLE)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				rt_memcpy(&(pitch), rx.Data, sizeof(pitch));
				rt_memcpy(&(roll), &(rx.Data[4]), sizeof(roll));
			break;
			
			case CAN_MESSAGE_INDEX1:
				
			break;
			
			default :break;			
		}
	}
	else
	{
	}


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
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;// 36M/8=4.00M 
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI1, &SPI_InitStructure);
	SPI_CalculateCRC(SPI1, DISABLE);

    SPI_Cmd(SPI1, ENABLE);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//NPD
	GPIO_SetBits(GPIOA, GPIO_Pin_0);
	rt_thread_delay(50);
	
	//reset
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	rt_thread_delay(5);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	rt_thread_delay(5);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	rt_thread_delay(10);
	
	//
	write_srom();
	rt_thread_delay(10);
	
	ADNS_Configuration();
	rt_thread_delay(20);
	

}

void thread_op(void)
{
	extern void can_message_send(void);
	op_init();
	
	while(1)
	{
		rt_thread_delay(10);
		Read_Data_burst();
		
		
		
		can_message_send();
		
	}

}


