/*
 * File      : thread_gps.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-22    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>

#include "stdlib.h"

#include "thread_gps.h"
#include "struct_all.h"

/* GPS 数据的环形缓冲区 */
#define GPS_RX_CIRCLE_SIZE 256
static char volatile gps_rx_circle[GPS_RX_CIRCLE_SIZE];
static char volatile *p_host = gps_rx_circle;
static char volatile *p_tail = gps_rx_circle;
static unsigned int volatile gps_rx_count = 0;
static unsigned int volatile gps_rx_circle_busy = 0;
unsigned int gps_rx_count_max = 0;

//加入数据到GPS缓冲区里面
int gps_data_to_circle(char *str, unsigned int count)
{
	unsigned int temp = 0;
	
	//正在操作缓冲区
	gps_rx_circle_busy = 1;
		
	while(count --)
	{
		//溢出后返回错误		
		if(gps_rx_count > GPS_RX_CIRCLE_SIZE)
		{
			gps_rx_circle_busy = 0;
			return 1;	 
		}
		
		*p_host = *(str + temp);
		p_host ++;
		temp ++;		
		gps_rx_count ++;

		//地址溢出后切换回头指针
		if(p_host >= &gps_rx_circle[GPS_RX_CIRCLE_SIZE])
			p_host = gps_rx_circle;	
				
	}

	//操作缓冲区完毕
	gps_rx_circle_busy = 0;
	
	return 0;
}

/*
 *从缓冲区里面读取数据
*/
unsigned int gps_data_from_circle(char* str)
{
	unsigned int temp = 0;
	unsigned int count = 0;
	unsigned int pre_count = 0;
	//缓冲区忙
	if(gps_rx_circle_busy)return 0;

	if(gps_rx_count > gps_rx_count_max)
		gps_rx_count_max = gps_rx_count;
	
	//记录数据量
	count = gps_rx_count;
	pre_count = count;
		
	while(count--)
	{
		*(str + temp) =	*p_tail;
		p_tail ++;
		temp ++;	

		//地址溢出后切换回头指针
		if(p_tail >= &gps_rx_circle[GPS_RX_CIRCLE_SIZE])
			p_tail = gps_rx_circle;
	}
	
	//更新数据量，避免冲突
	gps_rx_count -= pre_count; 
	
	//试验，加入反斜杠0做字符串结尾
	if(temp)
		*(str + temp) = '\0';
	
	return temp;
}

//GPS中断函数
void uart4_interrupt_handler(void)
{
	char temp;
	
	/* USART in mode Sender   --------------------------------------------------*/
	if(USART_GetITStatus(UART4, USART_IT_TXE) == SET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_TXE);
	}
	
	/* USART in mode Receiver --------------------------------------------------*/
	if(USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		temp =  USART_ReceiveData(UART4);
		gps_data_to_circle(&temp, 1);
				
	}  
}
//GPS发送字符串
int gps_send_cmd(char *s, int len)
{
	unsigned int count = 0;
	
	for(; len > 0; count++)
	{
		//等待发送空
		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) != SET);
		USART_SendData(UART4, (*(s+count)));
		len--;
	}
	
	return 0;
}

//hard ware initailization
void gps_hw_init(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

    //GPIO Periph clock enable 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE);
	
	// Enable USART4 clock 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	

	//USART 4
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Enable the USART4 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
// 	USART_ClockStructInit(&USART_ClockInitStruct);
// 	USART_ClockInit(USART2, &USART_ClockInitStruct);
	
	//USART2 Configaration
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);	
	USART_Cmd(UART4, ENABLE);

	//enable USART4 RXNE
	USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}



//GPS 命令标志
#define GPGGA ('G'+'P'+'G'+'G'+'A')
#define GPRMC ('G'+'P'+'R'+'M'+'C')

//GPGGA原始数据
struct _gpgga{
	char time[13];         //UTC时间
	char lat[11];         //纬度 latitiude
	char n_s;			  //南北半球
	char lon[12];         //经度 longitude
	char e_w;			  //东西经
	char warn;            //定位警告
	char quality;         //定位质量
	char alt[8];          //海拔
	char sv[3];           //使用卫星

}gpgga;

//GPRMC原始数据
struct _gprmc{
	char time[13];         //UTC时间
	char status;          //定位状态
	char lat[11];         //纬度 latitiude
	char n_s;			  //南北半球
	char lon[12];         //经度 longitude
	char e_w;			  //东西经
	char speed[7];        //速率 节
	char course[7];        //航向 角度
	char date[9];          //UTC日期
}gprmc;

//GPGGA 翻译函数
int gps_translate_gpgga(char c)
{
	static unsigned int segment;  //逗号计数
	static unsigned int byte_count; //字节计数
	
// 	char i;
// 	char la[11] = {"2517.34240"};
// 	char lo[12] = {"11019.93579"};

	//判断逗号
	if(c == ',')
	{
		byte_count = 0;
		segment++;
		return 0;
	}
	
	//传输完成后，转换并输出
	if(c == '$')
	{
		segment = 0;
		byte_count = 0;
		
		//转换纬度 北半球正
		if(gpgga.n_s == 'S')
			gps.lat = -1.0 * (atof(&gpgga.lat[2])/ 60.0 + (gpgga.lat[0]-'0')* 10 + (gpgga.lat[1]-'0'));
		else if(gpgga.n_s == 'N')
			gps.lat = atof(&gpgga.lat[2]) / 60.0 + (gpgga.lat[0]-'0')* 10 + (gpgga.lat[1]-'0');
		
		
		//经度 东半球正
		if(gpgga.e_w == 'W')
			gps.lon = -1.0 * (atof(&gpgga.lon[3]) /60.0 + (gpgga.lon[0]-'0')*100 + (gpgga.lon[1]-'0')*10 + (gpgga.lon[2]-'0'));
		else if(gpgga.e_w == 'E')
			gps.lon = atof(&gpgga.lon[3]) /60.0 + (gpgga.lon[0]-'0')*100 + (gpgga.lon[1]-'0')*10 + (gpgga.lon[2]-'0');
		
		//高度
		gps.altitude = atof(gpgga.alt);
		
		//输出定位状态
		gps.status = gpgga.quality;
		
		//卫星数量
		gps.stars = atof(gpgga.sv);
						
		return 0;
	}
	
	//根据逗号判断信息
	switch(segment)
    {
        case 1:    if(byte_count == 2 || byte_count == 5)    //$GPGGA段1，UTC时间，hhmmss（时分秒）格式,转换为HH:MM:SS.sss格式
                    {
                        gpgga.time[byte_count] = ':';
                        byte_count++;
                    }
                    if(byte_count<11)
                        gpgga.time[byte_count] = c;
					else
						gpgga.time[11] = '\0';
                    break;    

        case 2:    //$GPGGA 段2处理 纬度ddmm.mmmm（度分）格式
                    gpgga.lat[byte_count] = c; 
					gpgga.lat[10] = '\0';					
                    break;

       case 3:    gpgga.n_s = c;                     //$GPGGA第3段处理 纬度半球N（北半球）或S（南半球）
                    break;

       case 4:    //$GPGGA 段4处理 经度dddmm.mmmm（度分）格式
                    gpgga.lon[byte_count] = c;   
					gpgga.lon[11] = '\0';					
                    break;
        

       case 5:    gpgga.e_w = c;                  //$GPGGA第5段处理 经度半球E（东经）或W（西经）
                    break;
                        
       case 6:    gpgga.quality = c;			 //定位状态
                    break;
                                                                    
       case 7:if(byte_count<2)                    //$GPGGA第7段处理  正在使用解算位置的卫星数量（00~12）（前面的0也将被传输）
					gpgga.sv[byte_count] = c;
                  gpgga.sv[2] = '\0';     
                  break;
                                            
       case 9:if(byte_count<7)                     //$GPGGA第9段处理 海拔高度（-9999.9~99999.9）
              {
				gpgga.alt[byte_count] = c;
				gpgga.alt[byte_count+1] = '\0';
              }
              break;

       default:break;
    }
	byte_count ++;

	return 0;
}


//GPRMC 翻译函数
int gps_translate_gprmc(char c)
{
	static unsigned int segment;  //逗号计数
	static unsigned int byte_count; //字节计数
	
	//判断逗号
	if(c == ',')
	{
		byte_count = 0;
		segment++;
		return 0;
	}
	
	//判断到帧头之后，转换数值复位数值
	if(c == '$')
	{
		//复位数字
		segment = 0;
		byte_count = 0;
		
		//解析GPS数据
		//时间 年数不为0 ASCII 字符，有效
		if(gprmc.date[0] + gprmc.date[1]) 
		{
			gps.time.date  = (gprmc.date[6]-'0')*10 + (gprmc.date[7]-'0');
			gps.time.month = (gprmc.date[3]-'0')*10 + (gprmc.date[4]-'0');
			gps.time.year  = (gprmc.date[0]-'0')*10 + (gprmc.date[1]-'0');

			gps.time.milisecond = (gprmc.time[9]-'0')*100 + (gprmc.time[10]-'0')*10;
			gps.time.second = (gprmc.time[6]-'0')*10 + (gprmc.time[7]-'0');
			gps.time.minute = (gprmc.time[3]-'0')*10 + (gprmc.time[4]-'0');
			gps.time.hour   = (gprmc.time[0]-'0')*10 + (gprmc.time[1]-'0');
			
			//时间数据有效
			gps.time.flag = 1;
		}
		else
		{
			gps.time.flag = 0;
		}
	
		//速度信息
		gps.speed = atof(gprmc.speed);
		
		//转换坐标信息	
		gps.course = atof(gprmc.course);
		gps.speed  = atof(gprmc.speed);
		
		//最后一个命令，标志数据加1
		gps.count ++;
			
		return 0;
	}
	
	//根据逗号判断信息
	switch(segment)
    {
        case 1: if(byte_count == 2 || byte_count == 5)    //$GPRMC段1，UTC时间，hhmmss（时分秒）格式,转换为HH:MM:SS.sss格式
				{
					gprmc.time[byte_count] = ':';
					++byte_count;
				}
				if(byte_count<11)
					gprmc.time[byte_count] = c;
				else
					gprmc.time[11] = '\0';
				break; 
						
		case 2:gprmc.status = c;
			   break;

        case 3:             //$GPRMC 段2处理 纬度ddmm.mmmmm（度分）格式
				gprmc.lat[byte_count] = c;  
				gprmc.lat[10] = '\0';				
				break;

        case 4: gprmc.n_s = c;                     //$gprmc第3段处理 纬度半球N（北半球）或S（南半球）
                break;

		case 5:                //$gprmc 段4处理 经度dddmm.mmmmm（度分）格式
				gprmc.lon[byte_count] = c;
				gprmc.lon[11] = '\0';		
				break;
        case 6: gprmc.e_w = c;                     //$gprmc第5段处理 经度半球E（东经）或W（西经）
                break;
                        
        case 7: if(byte_count <5)                  //速度
				{
					gprmc.speed[byte_count] = c;
				}
				gprmc.speed[6] = '\0';
                break;
				
		case 8: if(byte_count <5)					//航向
				{
					gprmc.course[byte_count] = c;
				}
				gprmc.course[6] = '\0';
                break;
                                                                    
        case 9: if(byte_count<2)                    //$GPRMC第9段处理 UTC日期，ddmmyy（日月年）格式转换为yy-mm-dd
                {
                    gprmc.date[6+byte_count] = c;
                }
                if(byte_count>1 && byte_count<4)//月
                {
                    gprmc.date[1+byte_count] = c;
                    gprmc.date[5] = '-';
                }
                if(byte_count>3 && byte_count<6)//年
                {
                    gprmc.date[byte_count-4] = c;
                    gprmc.date[2] = '-';
                    gprmc.date[8] = '\0';
                }

       default:break;
    }
	byte_count ++;

	return 0;
}



//解释GPS数据
int gps_translater(char *s)
{
	unsigned int count = 0;
	static unsigned int command = 0;
	static unsigned int state = 0;
	static unsigned int byte_count = 0;
	
	//一直循环处理完字节
	for(; *(s+count)!= '\0'; count++)
	{
		//送去对应的函数处理
		if(state == 2)
		{
			switch(command)
			{
				case GPGGA :gps_translate_gpgga(*(s+count));break;
				case GPRMC :gps_translate_gprmc(*(s+count));break;
				default: break;
			}
		}
		
		//判断6个命令字节，然后选择对应的处理函数
		if(state == 1)
		{
			command += *(s+count);
			
			byte_count ++;			
			if(byte_count >= 5)
			{
				state = 2;
			}				
		}
		
		//起始命令
		if(*(s+count) == '$')
		{
			state = 1;
			byte_count = 0;
			command = 0;
		}		
	}
	return 0;
}

//ublox 命令

//GPS模块初始化
int gps_init(void)
{
	char ublox_command[32]={0xb5,0x62,0x06,0x01,0x03,0x00,0xf0,0x00};
	unsigned char count;

	//HARD ware initialization
	gps_hw_init();
	rt_thread_delay(100);

	//disable GPGLL
	count = 7;
	ublox_command[count++] = 0x01;
	ublox_command[count++] = 0x00;
	ublox_command[count++] = 0xfb;
	ublox_command[count++] = 0x11;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);

	//disable GPGSA
	count = 7;
	ublox_command[count++] = 0x02;
	ublox_command[count++] = 0x00;
	ublox_command[count++] = 0xfc;
	ublox_command[count++] = 0x13;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);
	
	//disable GPGSV
	count = 7;
	ublox_command[count++] = 0x03;
	ublox_command[count++] = 0x00;
	ublox_command[count++] = 0xfd;
	ublox_command[count++] = 0x15;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);
	
	//disable GPGLL
	count = 7;
	ublox_command[count++] = 0x01;
	ublox_command[count++] = 0x00;
	ublox_command[count++] = 0xfb;
	ublox_command[count++] = 0x11;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);
	
	//disable GPVTG
	count=7;
	ublox_command[count++]=0x05; 
	ublox_command[count++]=0x00;
	ublox_command[count++]=0xFF;
	ublox_command[count++]=0x19;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);
	
	//4HZ
	count = 3;
	ublox_command[count++]=0x08;
	ublox_command[count++]=0x06;
	ublox_command[count++]=0x00;
	ublox_command[count++]=0xC8;
	ublox_command[count++]=0x00; 
	ublox_command[count++]=0x01;
	ublox_command[count++]=0x00;
	ublox_command[count++]=0x01;
	ublox_command[count++]=0x00;
	ublox_command[count++]=0xDE; 
	ublox_command[count++]=0x6A;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);
	
	//波特率115200bps one time setting
	//B5 62 06 41 09 00 01 01 30 81 00 00 00 00 F8 FB 1C
	//B5 62 06 41 09 00 01 01 30 81 00 00 00 00 F8 FB 1C
	
	count = 0;
	ublox_command[count++]=0xB5;
	ublox_command[count++]=0x62;
	ublox_command[count++]=0x06;
	ublox_command[count++]=0x41;
	ublox_command[count++]=0x09; 
	ublox_command[count++]=0x00;
	ublox_command[count++]=0x01;
	ublox_command[count++]=0x01;
	ublox_command[count++]=0x30;
	ublox_command[count++]=0x81; 
	ublox_command[count++]=0x00;
	ublox_command[count++]=0x00;
	ublox_command[count++]=0x00;
	ublox_command[count++]=0x00;
	ublox_command[count++]=0xF8;
	ublox_command[count++]=0xFB;
	ublox_command[count++]=0x1C;
	gps_send_cmd(ublox_command, count);
	rt_thread_delay(5);
	
// 	index=3;
// 	ublox_cmd[index++]=0x00; 
// 	ublox_cmd[index++]=0x14;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x01;
// 	ublox_cmd[index++]=0x00; 
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0xD0;
// 	ublox_cmd[index++]=0x08;
// 	ublox_cmd[index++]=0x00; 
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0xC2;
// 	ublox_cmd[index++]=0x01;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x07;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x07;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0x00;
// 	ublox_cmd[index++]=0xC4;
// 	ublox_cmd[index++]=0x96;
// 	UART3_Send_Buf(ublox_cmd,index);
// 	delay_ms(10);
		
	return 0;
}



//GPS线程主体
void thread_gps(void *parameter)
{
	static char rx[GPS_RX_CIRCLE_SIZE];

	gps_init();
	
	//主循环
	while(1)
	{
		if(gps_rx_count != 0)
		{
			gps_data_from_circle(rx);
			//rt_kprintf("%s", rx);
			gps_translater(rx);
			//rt_kprintf("最大值 %d \n", gps_rx_count_max);
			
			if((gps.status != '0')&&(gps.status != 0))//字符
				gps.flag = 1;
			else
				gps.flag = 0;
		}
			
		rt_thread_delay(1);
	}
	
}


