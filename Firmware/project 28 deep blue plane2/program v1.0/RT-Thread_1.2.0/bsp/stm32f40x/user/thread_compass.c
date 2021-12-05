/*
 * File      : thread_compass.c
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

#include "thread_compass.h"

static struct rt_spi_device * spi_device; 
static struct rt_semaphore sem;

unsigned char hmc_recv_buf[10];
struct
{
	float x;
	float y;
	float z;
	float temp;

}hmc_raw;

struct
{
	short x;
	short y;
	short z;
}hmc;

//offset struct
struct 
{
	float xsf;	   //Elliptical correction
	float ysf;	   //
	float zsf;
	int xoff;	   //zero point offset
	int yoff;	   //
	int zoff;
	
	int mea_range; //range
	float mea_lsb; //LSB	

	int xmax;
	int xmin;
	int ymax;
	int ymin;
	int zmax;
	int zmin;

	unsigned int max_value ;

}hmc_offset;

static void interrupt_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//GPIO INPUT PIN
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	
	// Enable the Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void hmc_isr_handle(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
		rt_sem_release(&sem);	
	}	
}
	
static unsigned char  hmc_send(unsigned char reg, unsigned char data)
{
    rt_uint8_t send_buffer[2];
	rt_uint8_t recv_buffer[2];

    RT_ASSERT(spi_device != RT_NULL);

    send_buffer[0] = reg;
    send_buffer[1] = data;

    rt_spi_transfer(spi_device, send_buffer, recv_buffer, 2);
	
	return recv_buffer[1];
}



void hmc_get_data(void)
{
	static rt_uint8_t send_buffer[10];
	static short int t;
	

    RT_ASSERT(spi_device != RT_NULL);

    send_buffer[0] = 0xC0|0x03; //start with mode register
    rt_spi_transfer(spi_device, send_buffer, hmc_recv_buf, 8);
	
	hmc.x = hmc_recv_buf[2] | ((short)hmc_recv_buf[1] << 8);
	hmc.z = hmc_recv_buf[4] | ((short)hmc_recv_buf[3] << 8);
	hmc.y = hmc_recv_buf[6] | ((short)hmc_recv_buf[5] << 8);
	
	send_buffer[0] = 0xC0|0x31; //temp register
	rt_spi_transfer(spi_device, send_buffer, &hmc_recv_buf[7], 3);
	t = hmc_recv_buf[9] | ((short)hmc_recv_buf[8] << 8);
	
	hmc_raw.x = ((float)hmc.x * hmc_offset.xsf + hmc_offset.xoff) / hmc_offset.mea_lsb;
	hmc_raw.y = ((float)hmc.y * hmc_offset.ysf + hmc_offset.yoff) / hmc_offset.mea_lsb;
	hmc_raw.z = ((float)hmc.z * hmc_offset.zsf + hmc_offset.zoff) / hmc_offset.mea_lsb;
	
	hmc_raw.temp = (float)t / 128.f + 25;
}

unsigned int hmc_factor_init(void)
{
	//offset init
	hmc_offset.xsf  = 1;
	hmc_offset.ysf  = 1;
	hmc_offset.zsf  = 1;
	hmc_offset.xoff = 0;
	hmc_offset.yoff = 0;
	hmc_offset.zoff = 0;

	hmc_offset.xmax = 0;
	hmc_offset.xmin = 0;
	hmc_offset.ymax = 0;
	hmc_offset.ymin = 0;
	hmc_offset.zmax = 0;
	hmc_offset.zmin = 0;
	hmc_offset.max_value = 100;


	hmc_offset.mea_range = 1090;
	hmc_offset.mea_lsb	 = 1090.0f;

//*****************(should be in flash)**************
	hmc_offset.xmax = 572;
	hmc_offset.xmin = -431;
	hmc_offset.ymax = 619;
	hmc_offset.ymin = -488;
	hmc_offset.zmax = 524;
	hmc_offset.zmin = -457;

//	hmc_offset.xmax = 624;
//	hmc_offset.xmin = -475;
//	hmc_offset.ymax = 386;
//	hmc_offset.ymin = -747;
//	hmc_offset.zmax = 592;
//	hmc_offset.zmin = -491;


//	hmc_offset.xmax = 667;
//	hmc_offset.xmin = -396;
//	hmc_offset.ymax = 362;
//	hmc_offset.ymin = -699;
//	hmc_offset.zmax = 522;
//	hmc_offset.zmin = -494;
	
//	hmc_offset.xmax = 715;
//	hmc_offset.xmin = -420;
//	hmc_offset.ymax = 519;
//	hmc_offset.ymin = -547;
//	hmc_offset.zmax = 417;
//	hmc_offset.zmin = -681;


	hmc_offset.xsf = 1.0;
	hmc_offset.ysf = (double)(hmc_offset.xmax - hmc_offset.xmin) / (double)(hmc_offset.ymax - hmc_offset.ymin);		
	hmc_offset.zsf = (double)(hmc_offset.xmax - hmc_offset.xmin) / (double)(hmc_offset.zmax - hmc_offset.zmin);		
	hmc_offset.xoff =((hmc_offset.xmax - hmc_offset.xmin) / 2.0 - hmc_offset.xmax) * hmc_offset.xsf;
	hmc_offset.yoff =((hmc_offset.ymax - hmc_offset.ymin) / 2.0 - hmc_offset.ymax) * hmc_offset.ysf;	
	hmc_offset.zoff =((hmc_offset.zmax - hmc_offset.zmin) / 2.0 - hmc_offset.zmax) * hmc_offset.zsf;
//***************************************************************

	return 0;
}

/* setting OFFSET */
unsigned int get_hmc_offset(void)
{
	static unsigned int count = 0 ; 
	static unsigned int count_check = 0;
	static unsigned int max_value_bef;
	unsigned int temp;

	//count time
	count ++;

	//calculate the Gain
	temp = sqrt(hmc.y * hmc.y + hmc.z * hmc.z + hmc.x * hmc.x);

	//if data is right, and bigger then the default value
	if(temp > hmc_offset.max_value &&
	   hmc.x <= 2047 && hmc.x >= -2048 &&
	   hmc.y <= 2047 && hmc.y >= -2048 &&
	   hmc.z <= 2047 && hmc.y >= -2048)
	{
		hmc_offset.max_value = temp;
	}

	//biggest and smollest
	if(hmc.x > hmc_offset.xmax && hmc.x <= 2047)
	{
		hmc_offset.xmax = hmc.x;	
	}
	else if(hmc.x < hmc_offset.xmin && hmc.x >= -2048)
	{
		hmc_offset.xmin = hmc.x;
	}
	if(hmc.y > hmc_offset.ymax && hmc.y <= 2047)
	{
		hmc_offset.ymax = hmc.y;	
	}
	else if(hmc.y < hmc_offset.ymin && hmc.y >= -2048)
	{
		hmc_offset.ymin = hmc.y;
	}
	if(hmc.z > hmc_offset.zmax && hmc.z <= 2047)
	{
		hmc_offset.zmax = hmc.z;	
	}
	else if(hmc.z < hmc_offset.zmin && hmc.z >= -2048)
	{
		hmc_offset.zmin = hmc.z;
	}

	//if the data is steady, then calculate the OFFSET
	if(hmc_offset.xmax - hmc_offset.xmin > hmc_offset.max_value * 0.8 &&
	   hmc_offset.ymax - hmc_offset.ymin > hmc_offset.max_value * 0.8 &&
	   hmc_offset.zmax - hmc_offset.zmin > hmc_offset.max_value * 0.8 &&
	   count > 200*5 )		 //about 5 seconds
	{
		hmc_offset.xsf = 1.0;
		hmc_offset.ysf = (double)(hmc_offset.xmax - hmc_offset.xmin) / (double)(hmc_offset.ymax - hmc_offset.ymin);		
		hmc_offset.zsf = (double)(hmc_offset.xmax - hmc_offset.xmin) / (double)(hmc_offset.zmax - hmc_offset.zmin);		
		hmc_offset.xoff =((hmc_offset.xmax - hmc_offset.xmin) / 2.0 - hmc_offset.xmax) * hmc_offset.xsf;
		hmc_offset.yoff =((hmc_offset.ymax - hmc_offset.ymin) / 2.0 - hmc_offset.ymax) * hmc_offset.ysf;	
		hmc_offset.zoff =((hmc_offset.zmax - hmc_offset.zmin) / 2.0 - hmc_offset.zmax) * hmc_offset.zsf;

		//check 50 times
		if(count_check < 50)
		{
			//if data is not steady
			if(hmc_offset.max_value > max_value_bef + 10 || hmc_offset.max_value < max_value_bef - 10)
			{
				count_check = 0;
			}
			max_value_bef = hmc_offset.max_value;			

			count_check	++;	
		}

	}

	return 0;
}


rt_err_t hmc5983_init(const char * spi_device_name)
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
			{0x00, 0x9C},	//temperature measurment on, ODR 220HZ 
			{0x01, 0x20},	//1.3G
			{0x02, 0x00}	//data ready int, 4wire spi, Continuous-Measurement Mode.
			
		};
		unsigned char index;
		unsigned char temp;
		
		//id register
		temp = hmc_send(0x80|0x0A, 0xFF);
		rt_kprintf("\nHMC5983L has been identified: 0x%2x",temp);
		temp = hmc_send(0x80|0x0B, 0xFF);
		rt_kprintf(" 0x%02x",temp);
		temp = hmc_send(0x80|0x0C, 0xFF);
		rt_kprintf(" 0x%02x",temp);
		
		//setting register
		rt_kprintf("\nSeting HMC5983L...");
		while(index < sizeof(config)/2)
		{		
			temp = hmc_send(config[index][0], config[index][1]);
			temp = hmc_send(0x80|config[index][0], 0xFF);
			rt_kprintf("\nregister 0x%02x has been set to 0x%02x",config[index][0],temp);	
			
			index++;
		}
	}
	
	
	return 0;
}
int rt_hmc5983_init(void)
{
	rt_sem_init(&sem, "cmps", 0, RT_IPC_FLAG_FIFO);
	
	hmc_factor_init();
	
	interrupt_init();
	
	return hmc5983_init("spi32");

}
INIT_APP_EXPORT(rt_hmc5983_init);


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


//compass
void thread_compass(void *parameter)
{
	struct _coordinate filter_in;
	struct _coordinate filter_out;
	
	float xh;
	float yh;
	float s_pitch;
	float s_roll;
	float c_pitch;
	float c_roll;
	float mag_x;
	float mag_y;
	float mag_z;
	float course;

	
	while(1)
	{
		//update new data
		rt_sem_take(&sem, 50);
		hmc_get_data();
		
		//input the newest data to filter
		filter_in.x = hmc_raw.x;
		filter_in.y = hmc_raw.y;
		filter_in.z = hmc_raw.z;
		
		//filter
		filter_out = sliding_windows_filter_3(filter_in);
		
		//update the newest data
		mag.x = filter_out.x;
		mag.y = filter_out.y;
		mag.z = filter_out.z;
		mag.temperature.temp = hmc_raw.temp;
		
		mag.count++;
		
		//course/heading
		s_pitch = sin(-out_angle.rad_pitch);
		s_roll  = sin(out_angle.rad_roll);
		c_pitch = cos(-out_angle.rad_pitch);
		c_roll  = cos(out_angle.rad_roll);
		
// 		s_pitch = sin(-acc.angle.pitch);
// 		s_roll  = sin(-acc.angle.roll);
// 		c_pitch = cos(-acc.angle.pitch);
// 		c_roll  = cos(-acc.angle.roll);
		
		mag_x = mag.x;
		mag_y = mag.y;
		mag_z = mag.z;
		
		xh = mag_x*c_pitch + mag_y*s_roll*s_pitch - mag_z*c_roll*s_pitch;//
		yh = mag_y*c_roll  + mag_z*s_roll;

		course = ((float)atan2(yh,xh)) * 180.0f / (float)PI ;	

		course = course - 180.0;
		
		mag.course = course;
		
		//get_hmc_offset();
	}
}



