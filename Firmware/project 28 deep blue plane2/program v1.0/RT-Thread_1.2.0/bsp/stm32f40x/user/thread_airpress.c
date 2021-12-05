/*
 * File      : thread_airpress.c
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
#include "thread_airpress.h"

static struct rt_spi_device * spi_device; 

//compute parameter 
static unsigned short c1,c2,c3,c4,c5,c6;
static unsigned int d1,d2;
static int dt,temperature;
static long long off, sens, p, off2, sens2, t2;

//get the parameter
void ms5611_parameter_get(void)
{
	unsigned char rx_buf[3] = {0x00,0x00,0x00};
	unsigned char tx_buf[3] = {0x00,0x00,0x00};

	//reset delay 2.8ms to reload
	tx_buf[0] = MS_COMMAND_RST;
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 1);
	rt_thread_delay(3);		

	//Read the prom value 
	tx_buf[0] = MS_COMMAND_PROM_READ|(1<<1);			//C1
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 3);
	c1 = ((unsigned short)rx_buf[1] << 8) | rx_buf[2];
	
	tx_buf[0] = MS_COMMAND_PROM_READ|(2<<1);			//C2
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 3);
	c2 = ((unsigned short)rx_buf[1] << 8) | rx_buf[2];
		
	tx_buf[0] = MS_COMMAND_PROM_READ|(3<<1);			//C3
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 3);
	c3 = ((unsigned short)rx_buf[1] << 8) | rx_buf[2];
	
	tx_buf[0] = MS_COMMAND_PROM_READ|(4<<1);			//C4
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 3);
	c4 = ((unsigned short)rx_buf[1] << 8) | rx_buf[2];
	
	tx_buf[0] = MS_COMMAND_PROM_READ |(5<<1);			//C5
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 3);
	c5 = ((unsigned short)rx_buf[1] << 8) | rx_buf[2];
	
	tx_buf[0] = MS_COMMAND_PROM_READ+(6<<1);			//C6
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 3);
	c6 = ((unsigned short)rx_buf[1] << 8) | rx_buf[2];
	
	rt_kprintf("\nReading PROM...");
	rt_kprintf("\nParameter C1 = %d",c1);
	rt_kprintf("\nParameter C2 = %d",c2);
	rt_kprintf("\nParameter C3 = %d",c3);
	rt_kprintf("\nParameter C4 = %d",c4);
	rt_kprintf("\nParameter C5 = %d",c5);
	rt_kprintf("\nParameter C6 = %d",c6);
}

//
void ms5611_get_press(void)
{
	unsigned char rx_buf[4] = {0};
	unsigned char tx_buf[4] = {0};

	//send command
	tx_buf[0] = MS_COMMAND_D1_4096;
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 1);

	//wait 9.04ms 
	rt_thread_delay(10);

	//take the result
	tx_buf[0] = MS_COMMAND_ADC_READ;
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 4);
	
	//get d1 
	d1 = ((unsigned int)rx_buf[1] << 16) | ((unsigned int)rx_buf[2] << 8) | rx_buf[3];
}

//temperature
void ms5611_get_temp(void)
{
	unsigned char rx_buf[4] = {0};
	unsigned char tx_buf[4] = {0};

	//send command
	tx_buf[0] = MS_COMMAND_D2_2048;
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 1);

	//wait 5ms 
	rt_thread_delay(5);

	//take the result
	tx_buf[0] = MS_COMMAND_ADC_READ;
	rt_spi_transfer(spi_device, tx_buf, rx_buf, 4);
	
	//get d1 
	d2 = ((unsigned int)rx_buf[1] << 16) | ((unsigned int)rx_buf[2] << 8) | rx_buf[3];

}

//calculate the temperature and air presure 
void ms5611_calculate(void)
{
// 	c1 = 40127;			//test data
// 	c2 = 36924;
// 	c3 = 23317;
// 	c4 = 23282;
// 	c5 = 33464;
// 	c6 = 28312;
// 	d1 = 9085466;
// 	d2 = 8569150;

	
	//temperature
	dt = d2 - (long long)c5 *256;
	temperature = 2000 + (long long)dt*c6 / 0x800000;
	
	//pressure
	if(temperature < 2000) //lower than 20
	{
		t2 = (long long)dt*dt/0x80000000;
		off2 = 5*(temperature - 2000)*(temperature - 2000)/2;
		sens2 = 5*(temperature - 2000)*(temperature - 2000)/4;
		
		if(temperature < -1500)//lower than -15
		{
			off2 = off2 + 7*(temperature +1500)*(temperature +1500);
			sens2 = sens2+ 11*(temperature +1500)*(temperature +1500)/2;
		}
	}
	else
	{
		t2 = 0;
		sens2 = 0;
		off2 = 0;
	}
	
	//
	temperature -= t2;
	
	//air pressure
	off = (long long)c2*65536 + ((long long)c4*dt)/128  - off2;
	sens = (long long)c1*32768 + ((long long)c3*dt)/256 - sens2;
	p = ((long long)d1*sens/0x200000 - off)/32768;
	
}

rt_err_t ms5611_init(const char * spi_device_name)
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

	rt_kprintf("\nMS5611 has been identified.");

	//get parameter
	ms5611_parameter_get();
	
	return 0;
}
int rt_ms5611_init(void)
{
	return ms5611_init("spi33");

}
INIT_APP_EXPORT(rt_ms5611_init);


//1 axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    32	

static double sliding_windows_filter(double in)
{
	static double buf[FILTER_WINDOWS];
	static int buf_pointer = 0;
	int weight = FILTER_WINDOWS;
	int count = FILTER_WINDOWS;
	int index = buf_pointer;
	int total_weight = 0;	
	double integral = {0};
	
	//add the newest date to the buffer
	buf[index] = in;

	//filter
	while(count--)
	{
		integral += buf[index] * weight;
				
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
	return (integral / total_weight);
}


//1 axis sliding windows filter for climb rate
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    32	

static double cl_sliding_windows_filter(double in)
{
	static double buf[FILTER_WINDOWS];
	static int buf_pointer = 0;
	int weight = FILTER_WINDOWS;
	int count = FILTER_WINDOWS;
	int index = buf_pointer;
	int total_weight = 0;	
	double integral = {0};
	
	//add the newest date to the buffer
	buf[index] = in;

	//filter
	while(count--)
	{
		integral += buf[index] * weight;
				
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
	return (integral / total_weight);
}



//airpress
void thread_airpress(void *parameter)
{
	double altitude;
	double climb;
	double altitude_bef = 0 ;
	unsigned int sys_tick_bef = 0;
	unsigned int seconde_bef = 0xffffffff;
	
	while(1)
	{
		ms5611_get_temp();
		ms5611_get_press();
		ms5611_calculate();	
		
		//press is wrong then return 
		if(p <= 0)
			continue;
		
		//Temperature 
		air_info.temperature.temp = (double)temperature / 100.0;
		//air press
		air_info.air_press 	 = (double)p / 100.0 *100.0;
		
		//altitude
		altitude = 44330.f * (1.f - pow(((float)air_info.air_press / 101325.f), 1.f / 5.225f));
		air_info.altitude = sliding_windows_filter(altitude);//sliding window filter
		air_info.rt_altitude = altitude;
		
		
		//climb rate
		climb = (air_info.altitude - altitude_bef ) / 
			 ((float)(rt_tick_get() - sys_tick_bef) / (float)RT_TICK_PER_SECOND);
		sys_tick_bef = rt_tick_get();	
		//copy the last altitude for next time
		altitude_bef = air_info.altitude ;
	
		air_info.climb = cl_sliding_windows_filter(climb); //a different filter function
		air_info.rt_climb = climb;
		
		//update data count
		air_info.count ++;	
	}


}


