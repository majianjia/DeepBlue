/*
 * File      : thread_gyro.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-02    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include <drivers/spi.h>
#include "math.h"

#include "struct_all.h"
#include "time_measure.h"

#include "thread_position.h"

#define ADXL_CMD_READ  0x0B
#define ADXL_CMD_WRITE 0x0A

static struct rt_spi_device * spi_device; 
static struct rt_semaphore sem;

unsigned char adxl_recv_buf[10];
struct
{
	float x;
	float y;
	float z;
	float temp;
	short int range;

}adxl_raw = {0,0,0,0,4};//

struct
{
	short int x;
	short int y;
	short int z;
	short int temp;
}adxl_offset_2g, adxl_offset_4g, adxl_offset_8g;






static void interrupt_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//GPIO INPUT PIN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);
	
	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// Enable the USART2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void adxl_isr_handle(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
		rt_sem_release(&sem);		
	}	
}

void adxl_set_offset(void)
{	
	adxl_offset_2g.x = 6;
	adxl_offset_2g.y = -33;
	adxl_offset_2g.z = 130;
	adxl_offset_2g.temp = 0;
	
	adxl_offset_4g.x = 40;
	adxl_offset_4g.y = -7;
	adxl_offset_4g.z = 120;
	adxl_offset_4g.temp = 0;
	
	adxl_offset_8g.x = 64;
	adxl_offset_8g.y = 28;
	adxl_offset_8g.z = 106;
	adxl_offset_8g.temp = 0;
}


void adxl_set_range(void)
{
	//setting measurment range
	//adxl_raw.range = 8;
}

void adxl_get_data(void)
{
	static rt_uint8_t send_buffer[10];
	short int x, y, z, t;

    RT_ASSERT(spi_device != RT_NULL);

    send_buffer[0] = ADXL_CMD_READ;
    send_buffer[1] = 0x0E;

	//get data
    rt_spi_transfer(spi_device, send_buffer, adxl_recv_buf, 10);
	
	x = adxl_recv_buf[2] | ((short)adxl_recv_buf[3] << 8);
	y = adxl_recv_buf[4] | ((short)adxl_recv_buf[5] << 8);
	z = adxl_recv_buf[6] | ((short)adxl_recv_buf[7] << 8);
	t = adxl_recv_buf[8] | ((short)adxl_recv_buf[9] << 8);
		
	
	//offset
	if(adxl_raw.range == 2)
	{
		x -= adxl_offset_2g.x;
		y -= adxl_offset_2g.y;
		z -= adxl_offset_2g.z;
	}
	else if(adxl_raw.range == 4)
	{
		x -= adxl_offset_4g.x;
		y -= adxl_offset_4g.y;
		z -= adxl_offset_4g.z;
	}
	else if(adxl_raw.range == 8)
	{
		x -= adxl_offset_8g.x;
		y -= adxl_offset_8g.y;
		z -= adxl_offset_8g.z;
	}
	
	adxl_raw.x = x / (1000.f * 2.f/ adxl_raw.range);
	adxl_raw.y = y / (1000.f * 2.f/ adxl_raw.range);
	adxl_raw.z = z / (1000.f * 2.f/ adxl_raw.range);
	adxl_raw.temp = t * 0.065f;
	
	//update range
	adxl_set_range();
}

//register	
static unsigned char  adxl_send(unsigned char cmd, unsigned char reg, unsigned char data)
{
    rt_uint8_t send_buffer[3];
	rt_uint8_t recv_buffer[3];

    RT_ASSERT(spi_device != RT_NULL);

    send_buffer[0] = cmd;
    send_buffer[1] = reg;
	send_buffer[2] = data;

    rt_spi_transfer(spi_device, send_buffer, recv_buffer, 3);
	
	return recv_buffer[2];
}



rt_err_t adxl362_init(const char * spi_device_name)
{	
	//find spi bus device
	while(1)
	{
		int i = 0;
		spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
		if(spi_device != RT_NULL || i>RT_TICK_PER_SECOND) //wait
			break;
		rt_thread_delay(1);
	}
	if(spi_device == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }

	{
		unsigned char config[][2]=
		{
			{0x1F,0x52},//soft reset
			{0x2A,0x01},//Data ready interrupt enable 
			//{0x2C,0x15},//rang = 2G, ODR = 400Hz 1/4 filter
			{0x2C,0x55},//rang = 4G, ODR = 400Hz 1/4 filter
			//{0x2C,0x95},//rang = 8G, ODR = 400Hz 1/4 filter
			{0x2D,0x22} //Ultralow noise mode.Measurement mode.
		};
		unsigned char index = 0;
		unsigned char temp;
		
		temp = adxl_send(ADXL_CMD_READ, 0x00, 0xFF);
		//id register
		rt_kprintf("\nADXL362 has been identified: 0x%02x",temp);
		
		temp = adxl_send(ADXL_CMD_READ, 0x01, 0xFF);
		rt_kprintf("\nAnalog Devices MEMS device ID:0x%02x",temp);
		temp = adxl_send(ADXL_CMD_READ, 0x02, 0xFF);
		rt_kprintf("\nDevice ID: 0x%02x (362 octal)",temp);
		temp = adxl_send(ADXL_CMD_READ, 0x03, 0xFF);
		rt_kprintf("\nProduct revision ID: 0x%02x",temp);
		
		//config registers
		rt_kprintf("\nSeting ADXL362...");
		while(index < sizeof(config)/2)
		{		
			temp = adxl_send(ADXL_CMD_WRITE, config[index][0], config[index][1]);
			temp = adxl_send(ADXL_CMD_READ, config[index][0], 0xFF);
			rt_kprintf("\nregister 0x%2x has been set to 0x%02x",config[index][0],temp);	
			
			index++;
		}	
		
	}
	
	//init offset
	adxl_set_offset();
	
	
	return 0;
}
int rt_adxl362_init(void)
{
	rt_sem_init(&sem, "adxl", 0, RT_IPC_FLAG_FIFO);
	
	interrupt_init();
	
	adxl362_init("spi31");
	
	return 0 ;
}
INIT_APP_EXPORT(rt_adxl362_init);

//3 axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    (400/10)	// 1/10S

static struct _coordinate sliding_windows_filter_3(struct _coordinate in)
{
	static struct _coordinate buf[FILTER_WINDOWS];
	static int buf_pointer = 0;
	int weight = FILTER_WINDOWS;
	int count = FILTER_WINDOWS;
	int index = buf_pointer;
	int total_weight = 0;	
	struct _coordinate integral = {0};
	struct _coordinate out;
	
	//add the newest date to the buffer
	buf[index].x = in.x;
	buf[index].y = in.y;
	buf[index].z = in.z;
			
	//filter
	while(count--)
	{
		integral.x += buf[index].x * weight;
		integral.y += buf[index].y * weight;
		integral.z += buf[index].z * weight;
				
		total_weight += weight; //the newest data have the most significant weight
		weight--;				
				
		index++;
		if(index >= FILTER_WINDOWS)
			index = 0;
	}
	buf_pointer ++;
	if(buf_pointer >= FILTER_WINDOWS)
		buf_pointer = 0;
			
	//update the newest date
	out.x  = integral.x / total_weight;
	out.y  = integral.y / total_weight;
	out.z  = integral.z / total_weight;
	
	return out;
}

//acc
void thread_acc(void *parameter)
{	
	struct _coordinate filter_in;
	struct _coordinate filter_out;
	
	while(1)
	{
		//wait until sensor data ready
		rt_sem_take(&sem, 5);
		adxl_get_data();
		
		//get filter in put data
		filter_in.x = adxl_raw.x;
		filter_in.y = adxl_raw.y;
		filter_in.z = adxl_raw.z;
		
		//filtering
		filter_out = sliding_windows_filter_3(filter_in);
		
		//update the newest date
		acc.axis.x = filter_out.x;
		acc.axis.y = filter_out.y;
		acc.axis.z = filter_out.z;
		
		acc.axis_rt.x = adxl_raw.x;
		acc.axis_rt.y = adxl_raw.y;
		acc.axis_rt.z = adxl_raw.z;
		acc.temperature.temp = adxl_raw.temp;

		acc.count ++;	
	}

}




