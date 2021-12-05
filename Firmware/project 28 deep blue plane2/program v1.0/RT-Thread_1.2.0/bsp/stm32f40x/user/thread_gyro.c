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

//(1) use sensor's DR signal (0) use mcu's timer
#define USING_DATA_READY 1

static struct rt_spi_device * spi_device; 
static struct rt_semaphore sem;

unsigned char max_recv_buf[10];
struct
{
	float x;
	float y;
	float z;
	short int temp;
	short int range;
}max_raw = {0,0,0,0,250};

struct
{
	short int x;
	short int y;
	short int z;
	short int temp;
}max_offset;

#if USING_DATA_READY
static void interrupt_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//GPIO INPUT PIN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	
	/* Configure EXTI Line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// Enable the Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#else

static void timer_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//设定定时器
	TIM_DeInit(TIM7);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	/*在system_stm32f4xx.c中设置的APB1 Prescaler = 4 ,可知
	APB1时钟为168M/4*2,因为如果APB1分频不为1，则定时时钟*2*/
	TIM_TimeBaseStructure.TIM_Period = 500;				//us
	TIM_TimeBaseStructure.TIM_Prescaler = 84  -1 ;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
		
	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update, ENABLE);  
	
	TIM_Cmd(TIM7, ENABLE);
}

#endif

 void gyro_isr_handle(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);	
		rt_sem_release(&sem);
	}	
	
}

void tim7_isr_handle(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		rt_sem_release(&sem);
	}
}

static unsigned char  max_send(unsigned char reg, unsigned char data)
{
    rt_uint8_t send_buffer[2];
	rt_uint8_t recv_buffer[2];

    RT_ASSERT(spi_device != RT_NULL);

    send_buffer[0] = reg;
    send_buffer[1] = data;

    rt_spi_transfer(spi_device, send_buffer, recv_buffer, 2);
	
	return recv_buffer[1];
}

void max_set_offset(void)
{	
	max_offset.x = -142;
	max_offset.y = 37;
	max_offset.z = -71;
	max_offset.temp = 0;
}

void max_set_range(short x, short y, short z)
{
	unsigned char reg;
	
	//setting measurment range
	//GYROranging threthold
	#define GYRO_AUTO_RANGE_MIN (0.3 * 30000)
	#define GYRO_AUTO_RANGE_MAX (0.8* 30000)
	
	//ranging too big
	if((x < GYRO_AUTO_RANGE_MIN && x > -GYRO_AUTO_RANGE_MIN) &&
	   (y < GYRO_AUTO_RANGE_MIN && y > -GYRO_AUTO_RANGE_MIN) &&
	   (z < GYRO_AUTO_RANGE_MIN && z > -GYRO_AUTO_RANGE_MIN) && max_raw.range != 250)
	{
		switch(max_raw.range)
		{
			case 2000: 	max_raw.range = 1000;
						reg = 0x01;//1000
						break;
			case 1000:	max_raw.range = 500; 
						reg = 0x02;//500
						break;
			case 500 : 	max_raw.range = 250;  
						reg = 0x03;//250
						break;
			default  : 	max_raw.range = 250;  
						reg = 0x03;
						break;
		}
		max_send(0x21, 0x00); //bank0
		max_send(0x00, reg<<6 | 0x0F); // degrees per second		
	}
	
	//gyro ranging too small
	if(((x > GYRO_AUTO_RANGE_MAX || x < -GYRO_AUTO_RANGE_MAX) ||
	   (y > GYRO_AUTO_RANGE_MAX || y < -GYRO_AUTO_RANGE_MAX) ||
	   (z > GYRO_AUTO_RANGE_MAX || z < -GYRO_AUTO_RANGE_MAX)) && max_raw.range != 2000)
	{
		switch(max_raw.range)
		{
			case 250  : max_raw.range  = 500;   
						reg = 0x02;//500
						break;
			case 500  : max_raw.range  = 1000;  
						reg = 0x01;//1000
						break;
	
			case 1000 : max_raw.range  = 2000;
						reg = 0x00;//2000
						break;
			default   : max_raw.range  = 2000;  
						reg = 0x00;//2000
						break;
		}	
		max_send(0x21, 0x00); //bank0
		max_send(0x00, reg<<6 | 0x0F); // degrees per second
	}	
	
}

void gyro_auto_offset(short x, short y, short z)
{
	static int steady_count = 0;
	static int avg_count = 0;
	static int x_int = 0;
	static int y_int = 0;
	static int z_int = 0;
	static float acc_x;
	static float acc_y;
	static float acc_g;
	static unsigned int steady_flag = 0;
	
	//if steady?
	if((acc.g - acc_g < 0.01 && acc.g - acc_g > -0.01) && (acc.g >0.97 && acc.g<1.03))
	{
		//first time enter this if
		if(steady_flag == 0)
		{
			steady_flag = 1;
			acc_x = acc.axis.x;
			acc_y = acc.axis.y;
		}
		//not first time 
		else
		{
			//not steady
			if((acc.axis_rt.x - acc_x > 0.03 || acc.axis_rt.x - acc_x < -0.03)
				||(acc.axis_rt.y - acc_y > 0.03 || acc.axis_rt.y - acc_y < -0.03))
			{
				steady_flag = 0;
			}
		}
	}
	else
	{
		acc_g = acc.g;
	}		
	
	
	//if steady	
	if(steady_flag)
	{
		steady_count++;
		
		//steady for 1s
		if(steady_count > 2000*4)
		{
			x_int += x;
			y_int += y;
			z_int += z;
			avg_count ++;
			
			//if integrate for 2s. just for first time when steady count == 2000;
			if(avg_count == 2000*1)
			{
				max_offset.x = x_int/avg_count;
				max_offset.y = y_int/avg_count;
				max_offset.z = z_int/avg_count;
			}

		}
	}
	//if not steady
	else
	{
		steady_count = 0;
		avg_count = 0;
		x_int = 0;
		y_int = 0;
		z_int = 0;
	}


}

void gyro_get_data(void)
{
	static rt_uint8_t send_buffer[10];
	static short int x,y,z,t;

    RT_ASSERT(spi_device != RT_NULL);

    send_buffer[0] = 0x22|0x80; //start with state register

	//get raw data form the sensor
    rt_spi_transfer(spi_device, send_buffer, max_recv_buf, 10);
	
	//get raw data
	x = (max_recv_buf[3] | ((short)max_recv_buf[2] << 8));
	y = (max_recv_buf[5] | ((short)max_recv_buf[4] << 8));
	z = (max_recv_buf[7] | ((short)max_recv_buf[6] << 8));
	t = max_recv_buf[9] | ((short)max_recv_buf[8] << 8);
	
	//if stop get offset
	gyro_auto_offset(x,y,z);
	
	//offset
	x -= max_offset.x / (max_raw.range / 250);
	y -= max_offset.y / (max_raw.range / 250);
	z -= max_offset.z / (max_raw.range / 250);
	
	//update to output data	
	max_raw.x = x * (max_raw.range / 250) /120.f;
	max_raw.y =	y * (max_raw.range / 250) /120.f;
	max_raw.z = z * (max_raw.range / 250) /120.f;
	max_raw.temp = (float)t / 256.f;
	 
	//update range
	max_set_range(x, y, z);
}







rt_err_t max21000_init(const char * spi_device_name)
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
		//setting sequency
		unsigned char config[][2]=
		{
			//{0x1F, 0xFF},	//soft reset
			
			{0x21, 0x00},	//bank0
			{0x00, 0xCF},   //POWER_CFG    250degree, XYZ enable, normal power mode
			{0x01, 0x3C},   //Bandwidth 400HZ, OIS off, self test of
			{0x02, 4   },	//ODR = 10k/(reg+1) 
			{0x03, 0x00},	//high pass filter off
			{0x13, 0x01},	//data ready clear when all reg has been read. temperature sensor enable
			{0x14, 0x03},	//I2C1 pin not pull up
			{0x15, 0x51},	//disable iic, 3ma I/O current
			{0x16, 0x00},	//default
			{0x17, 0x00},	//fifo threshold
			{0x18, 0x00},   //FIFO mode
			{0x1A, 0x00},	//DSYNC
			{0x1B, 0x00},	//DSYNC
			
			{0x21, 0x01},	//bank1
			{0x00, 0x00},
			{0x01, 0x00},
			{0x02, 0x00},
			{0x03, 0x00},
			{0x04, 0x00},
			{0x05, 0x00},
			{0x06, 0x00},
			{0x07, 0x00},
			{0x08, 0x00},
			{0x09, 0x00}, //INT_MASK_AO: Interrupt AND and Interrupt OR masks
			{0x0A, 0x00},
			{0x0B, 0x24}, //int1 int2 push pull, high active
			{0x0C, 0x00}, //not latched
			{0x10, 0x80}, // only dataready int is mask
			{0x11, 0x00}, //turn off the restart interrupt on int2
		};
		unsigned char temp;
		unsigned char index;
		
		//wait for restart
		max_send(0x1F, 0xFF); //reset
		max_send(0x21, 0x01); //bank1
		while(max_send(0x80|0x0D, 0xFF) & 2); 
		
		temp = max_send(0x00|0x21, 0x00); //bank0
		temp = max_send(0x00|0x15, 0x5D); //disable iic
		
		//read id register
		temp = max_send(0x80|0x20, 0xFF);
		rt_kprintf("\nMAX21000 has been identified: 0x%2x\n",temp);
		
		//read device ID
		temp = max_send(0x00|0x21, 0x01); //bank1
		
		temp = max_send(0xC0|0x1A, 0xFF); 
		rt_kprintf("Unique serial number:0x%02x",temp);
		temp = max_send(0xC0|0x1b, 0xFF); 
		rt_kprintf("%02x",temp);
		temp = max_send(0xC0|0x1c, 0xFF); 
		rt_kprintf("%02x",temp);
		temp = max_send(0xC0|0x1d, 0xFF); 
		rt_kprintf("%02x",temp);
		temp = max_send(0xC0|0x1e, 0xFF); 
		rt_kprintf("%02x",temp);
		temp = max_send(0xC0|0x1f, 0xFF); 
		rt_kprintf("%02x",temp);
		
		
		rt_kprintf("\nSeting MAX21000...");
		while(index < sizeof(config)/2)
		{				
			//print bank id
			if(config[index][0] == 0x21)
			{	
				rt_kprintf("\nSetting register bank %d ...",config[index][1]);		
			}
			
			// writing register			
			temp = max_send(config[index][0], config[index][1]);
			temp = max_send(0x80|config[index][0], 0xFF);
			rt_kprintf("\nregister 0x%02x has been set to 0x%02x",config[index][0],temp);	
			
			index++;
		}
		
	}
	
	//init offset
	max_set_offset();
	
	return 0;
}

int rt_max21000_init(void)
{
	rt_sem_init(&sem, "max", 100, RT_IPC_FLAG_FIFO);	
	
#if USING_DATA_READY
	interrupt_init();
#else
	timer_init();
#endif

	max21000_init("spi30");
	
	return 0;
}
INIT_APP_EXPORT(rt_max21000_init);


//3 axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    32	

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




//gyro
void thread_gyro(void *parameter)
{
	extern void position_release_start(void);
	struct _coordinate filter_in;
	struct _coordinate filter_out;
	
	while(1)
	{
		//wait for newest data
		rt_sem_take(&sem, 10);
		gyro_get_data();
		
		//get filter in put data
		filter_in.x = max_raw.x;
		filter_in.y = max_raw.y;
		filter_in.z = max_raw.z;
		
		//filtering
		filter_out = sliding_windows_filter_3(filter_in);
		
		//update newest data	
		gyro.rt_pitch = max_raw.x;
		gyro.rt_roll  = max_raw.y;
		gyro.rt_yaw   = -max_raw.z;
		gyro.pitch 	  = filter_out.x;
		gyro.roll     = filter_out.y;
		gyro.yaw      = -filter_out.z;
		gyro.rad_pitch= gyro.pitch 	/ 180.0 * PI; 
		gyro.rad_roll = gyro.roll 	/ 180.0 * PI; 
		gyro.rad_yaw  = gyro.yaw 	/ 180.0 * PI; 
		gyro.temperature.temp = max_raw.temp;
		gyro.count++;	

		// tell thread_position to calculate new position 1khz
		if(gyro.count % 2 ==0)
			position_release_start();

	}
}



