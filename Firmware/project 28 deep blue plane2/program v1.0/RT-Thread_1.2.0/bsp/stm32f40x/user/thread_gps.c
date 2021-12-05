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

/* GPS ���ݵĻ��λ����� */
#define GPS_RX_CIRCLE_SIZE 256
static char volatile gps_rx_circle[GPS_RX_CIRCLE_SIZE];
static char volatile *p_host = gps_rx_circle;
static char volatile *p_tail = gps_rx_circle;
static unsigned int volatile gps_rx_count = 0;
static unsigned int volatile gps_rx_circle_busy = 0;
unsigned int gps_rx_count_max = 0;

//�������ݵ�GPS����������
int gps_data_to_circle(char *str, unsigned int count)
{
	unsigned int temp = 0;
	
	//���ڲ���������
	gps_rx_circle_busy = 1;
		
	while(count --)
	{
		//����󷵻ش���		
		if(gps_rx_count > GPS_RX_CIRCLE_SIZE)
		{
			gps_rx_circle_busy = 0;
			return 1;	 
		}
		
		*p_host = *(str + temp);
		p_host ++;
		temp ++;		
		gps_rx_count ++;

		//��ַ������л���ͷָ��
		if(p_host >= &gps_rx_circle[GPS_RX_CIRCLE_SIZE])
			p_host = gps_rx_circle;	
				
	}

	//�������������
	gps_rx_circle_busy = 0;
	
	return 0;
}

/*
 *�ӻ����������ȡ����
*/
unsigned int gps_data_from_circle(char* str)
{
	unsigned int temp = 0;
	unsigned int count = 0;
	unsigned int pre_count = 0;
	//������æ
	if(gps_rx_circle_busy)return 0;

	if(gps_rx_count > gps_rx_count_max)
		gps_rx_count_max = gps_rx_count;
	
	//��¼������
	count = gps_rx_count;
	pre_count = count;
		
	while(count--)
	{
		*(str + temp) =	*p_tail;
		p_tail ++;
		temp ++;	

		//��ַ������л���ͷָ��
		if(p_tail >= &gps_rx_circle[GPS_RX_CIRCLE_SIZE])
			p_tail = gps_rx_circle;
	}
	
	//�����������������ͻ
	gps_rx_count -= pre_count; 
	
	//���飬���뷴б��0���ַ�����β
	if(temp)
		*(str + temp) = '\0';
	
	return temp;
}

//GPS�жϺ���
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
//GPS�����ַ���
int gps_send_cmd(char *s, int len)
{
	unsigned int count = 0;
	
	for(; len > 0; count++)
	{
		//�ȴ����Ϳ�
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



//GPS �����־
#define GPGGA ('G'+'P'+'G'+'G'+'A')
#define GPRMC ('G'+'P'+'R'+'M'+'C')

//GPGGAԭʼ����
struct _gpgga{
	char time[13];         //UTCʱ��
	char lat[11];         //γ�� latitiude
	char n_s;			  //�ϱ�����
	char lon[12];         //���� longitude
	char e_w;			  //������
	char warn;            //��λ����
	char quality;         //��λ����
	char alt[8];          //����
	char sv[3];           //ʹ������

}gpgga;

//GPRMCԭʼ����
struct _gprmc{
	char time[13];         //UTCʱ��
	char status;          //��λ״̬
	char lat[11];         //γ�� latitiude
	char n_s;			  //�ϱ�����
	char lon[12];         //���� longitude
	char e_w;			  //������
	char speed[7];        //���� ��
	char course[7];        //���� �Ƕ�
	char date[9];          //UTC����
}gprmc;

//GPGGA ���뺯��
int gps_translate_gpgga(char c)
{
	static unsigned int segment;  //���ż���
	static unsigned int byte_count; //�ֽڼ���
	
// 	char i;
// 	char la[11] = {"2517.34240"};
// 	char lo[12] = {"11019.93579"};

	//�ж϶���
	if(c == ',')
	{
		byte_count = 0;
		segment++;
		return 0;
	}
	
	//������ɺ�ת�������
	if(c == '$')
	{
		segment = 0;
		byte_count = 0;
		
		//ת��γ�� ��������
		if(gpgga.n_s == 'S')
			gps.lat = -1.0 * (atof(&gpgga.lat[2])/ 60.0 + (gpgga.lat[0]-'0')* 10 + (gpgga.lat[1]-'0'));
		else if(gpgga.n_s == 'N')
			gps.lat = atof(&gpgga.lat[2]) / 60.0 + (gpgga.lat[0]-'0')* 10 + (gpgga.lat[1]-'0');
		
		
		//���� ��������
		if(gpgga.e_w == 'W')
			gps.lon = -1.0 * (atof(&gpgga.lon[3]) /60.0 + (gpgga.lon[0]-'0')*100 + (gpgga.lon[1]-'0')*10 + (gpgga.lon[2]-'0'));
		else if(gpgga.e_w == 'E')
			gps.lon = atof(&gpgga.lon[3]) /60.0 + (gpgga.lon[0]-'0')*100 + (gpgga.lon[1]-'0')*10 + (gpgga.lon[2]-'0');
		
		//�߶�
		gps.altitude = atof(gpgga.alt);
		
		//�����λ״̬
		gps.status = gpgga.quality;
		
		//��������
		gps.stars = atof(gpgga.sv);
						
		return 0;
	}
	
	//���ݶ����ж���Ϣ
	switch(segment)
    {
        case 1:    if(byte_count == 2 || byte_count == 5)    //$GPGGA��1��UTCʱ�䣬hhmmss��ʱ���룩��ʽ,ת��ΪHH:MM:SS.sss��ʽ
                    {
                        gpgga.time[byte_count] = ':';
                        byte_count++;
                    }
                    if(byte_count<11)
                        gpgga.time[byte_count] = c;
					else
						gpgga.time[11] = '\0';
                    break;    

        case 2:    //$GPGGA ��2���� γ��ddmm.mmmm���ȷ֣���ʽ
                    gpgga.lat[byte_count] = c; 
					gpgga.lat[10] = '\0';					
                    break;

       case 3:    gpgga.n_s = c;                     //$GPGGA��3�δ��� γ�Ȱ���N�������򣩻�S���ϰ���
                    break;

       case 4:    //$GPGGA ��4���� ����dddmm.mmmm���ȷ֣���ʽ
                    gpgga.lon[byte_count] = c;   
					gpgga.lon[11] = '\0';					
                    break;
        

       case 5:    gpgga.e_w = c;                  //$GPGGA��5�δ��� ���Ȱ���E����������W��������
                    break;
                        
       case 6:    gpgga.quality = c;			 //��λ״̬
                    break;
                                                                    
       case 7:if(byte_count<2)                    //$GPGGA��7�δ���  ����ʹ�ý���λ�õ�����������00~12����ǰ���0Ҳ�������䣩
					gpgga.sv[byte_count] = c;
                  gpgga.sv[2] = '\0';     
                  break;
                                            
       case 9:if(byte_count<7)                     //$GPGGA��9�δ��� ���θ߶ȣ�-9999.9~99999.9��
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


//GPRMC ���뺯��
int gps_translate_gprmc(char c)
{
	static unsigned int segment;  //���ż���
	static unsigned int byte_count; //�ֽڼ���
	
	//�ж϶���
	if(c == ',')
	{
		byte_count = 0;
		segment++;
		return 0;
	}
	
	//�жϵ�֡ͷ֮��ת����ֵ��λ��ֵ
	if(c == '$')
	{
		//��λ����
		segment = 0;
		byte_count = 0;
		
		//����GPS����
		//ʱ�� ������Ϊ0 ASCII �ַ�����Ч
		if(gprmc.date[0] + gprmc.date[1]) 
		{
			gps.time.date  = (gprmc.date[6]-'0')*10 + (gprmc.date[7]-'0');
			gps.time.month = (gprmc.date[3]-'0')*10 + (gprmc.date[4]-'0');
			gps.time.year  = (gprmc.date[0]-'0')*10 + (gprmc.date[1]-'0');

			gps.time.milisecond = (gprmc.time[9]-'0')*100 + (gprmc.time[10]-'0')*10;
			gps.time.second = (gprmc.time[6]-'0')*10 + (gprmc.time[7]-'0');
			gps.time.minute = (gprmc.time[3]-'0')*10 + (gprmc.time[4]-'0');
			gps.time.hour   = (gprmc.time[0]-'0')*10 + (gprmc.time[1]-'0');
			
			//ʱ��������Ч
			gps.time.flag = 1;
		}
		else
		{
			gps.time.flag = 0;
		}
	
		//�ٶ���Ϣ
		gps.speed = atof(gprmc.speed);
		
		//ת��������Ϣ	
		gps.course = atof(gprmc.course);
		gps.speed  = atof(gprmc.speed);
		
		//���һ�������־���ݼ�1
		gps.count ++;
			
		return 0;
	}
	
	//���ݶ����ж���Ϣ
	switch(segment)
    {
        case 1: if(byte_count == 2 || byte_count == 5)    //$GPRMC��1��UTCʱ�䣬hhmmss��ʱ���룩��ʽ,ת��ΪHH:MM:SS.sss��ʽ
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

        case 3:             //$GPRMC ��2���� γ��ddmm.mmmmm���ȷ֣���ʽ
				gprmc.lat[byte_count] = c;  
				gprmc.lat[10] = '\0';				
				break;

        case 4: gprmc.n_s = c;                     //$gprmc��3�δ��� γ�Ȱ���N�������򣩻�S���ϰ���
                break;

		case 5:                //$gprmc ��4���� ����dddmm.mmmmm���ȷ֣���ʽ
				gprmc.lon[byte_count] = c;
				gprmc.lon[11] = '\0';		
				break;
        case 6: gprmc.e_w = c;                     //$gprmc��5�δ��� ���Ȱ���E����������W��������
                break;
                        
        case 7: if(byte_count <5)                  //�ٶ�
				{
					gprmc.speed[byte_count] = c;
				}
				gprmc.speed[6] = '\0';
                break;
				
		case 8: if(byte_count <5)					//����
				{
					gprmc.course[byte_count] = c;
				}
				gprmc.course[6] = '\0';
                break;
                                                                    
        case 9: if(byte_count<2)                    //$GPRMC��9�δ��� UTC���ڣ�ddmmyy�������꣩��ʽת��Ϊyy-mm-dd
                {
                    gprmc.date[6+byte_count] = c;
                }
                if(byte_count>1 && byte_count<4)//��
                {
                    gprmc.date[1+byte_count] = c;
                    gprmc.date[5] = '-';
                }
                if(byte_count>3 && byte_count<6)//��
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



//����GPS����
int gps_translater(char *s)
{
	unsigned int count = 0;
	static unsigned int command = 0;
	static unsigned int state = 0;
	static unsigned int byte_count = 0;
	
	//һֱѭ���������ֽ�
	for(; *(s+count)!= '\0'; count++)
	{
		//��ȥ��Ӧ�ĺ�������
		if(state == 2)
		{
			switch(command)
			{
				case GPGGA :gps_translate_gpgga(*(s+count));break;
				case GPRMC :gps_translate_gprmc(*(s+count));break;
				default: break;
			}
		}
		
		//�ж�6�������ֽڣ�Ȼ��ѡ���Ӧ�Ĵ�����
		if(state == 1)
		{
			command += *(s+count);
			
			byte_count ++;			
			if(byte_count >= 5)
			{
				state = 2;
			}				
		}
		
		//��ʼ����
		if(*(s+count) == '$')
		{
			state = 1;
			byte_count = 0;
			command = 0;
		}		
	}
	return 0;
}

//ublox ����

//GPSģ���ʼ��
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
	
	//������115200bps one time setting
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



//GPS�߳�����
void thread_gps(void *parameter)
{
	static char rx[GPS_RX_CIRCLE_SIZE];

	gps_init();
	
	//��ѭ��
	while(1)
	{
		if(gps_rx_count != 0)
		{
			gps_data_from_circle(rx);
			//rt_kprintf("%s", rx);
			gps_translater(rx);
			//rt_kprintf("���ֵ %d \n", gps_rx_count_max);
			
			if((gps.status != '0')&&(gps.status != 0))//�ַ�
				gps.flag = 1;
			else
				gps.flag = 0;
		}
			
		rt_thread_delay(1);
	}
	
}


