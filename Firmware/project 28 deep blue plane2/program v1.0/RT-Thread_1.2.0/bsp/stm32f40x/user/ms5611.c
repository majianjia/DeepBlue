/*
 * File      : ms5611.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-05-28    majianjia   change for MS5611
 */


#include "ms5611.h"
#include "stm32f4_i2c.h"
#include "rtthread.h"
#include "thread_sensors.h"
#include "struct_all.h"
#include "math.h"


//compute parameter 
static unsigned short c1,c2,c3,c4,c5,c6;
static unsigned int d1,d2;
static int dt,temperature;
static long long off, sens, p, off2, sens2, t2;

//get the parameter
void ms5611_parameter_get(void)
{
	unsigned char rx_buf[3] = {0x00,0x00,0x00};
	unsigned char temp;

	//reset delay 2.8ms to reload
	I2C_DMA_WriteReg(&temp, MS5611_WR_ADDR, MS_COMMAND_RST, 0); //reset
	rt_thread_delay(5);		

	//Read the prom value 
	I2C_DMA_ReadReg(rx_buf, MS5611_RD_ADDR, MS_COMMAND_PROM_READ+(1<<1), 2); //C1
	c1 = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];

	I2C_DMA_ReadReg(rx_buf, MS5611_RD_ADDR, MS_COMMAND_PROM_READ+(2<<1), 2); //C2
	c2 = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];

	I2C_DMA_ReadReg(rx_buf, MS5611_RD_ADDR, MS_COMMAND_PROM_READ+(3<<1), 2); //C3
	c3 = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];

	I2C_DMA_ReadReg(rx_buf, MS5611_RD_ADDR, MS_COMMAND_PROM_READ+(4<<1), 2); //C4
	c4 = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];

	I2C_DMA_ReadReg(rx_buf, MS5611_RD_ADDR, MS_COMMAND_PROM_READ+(5<<1), 2); //C5
	c5 = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];

	I2C_DMA_ReadReg(rx_buf, MS5611_RD_ADDR, MS_COMMAND_PROM_READ+(6<<1), 2); //C6
	c6 = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];

}


//ȡ����ѹ��
void ms5611_get_press(void)
{
	unsigned char rx_buf[3] = {0};
	unsigned char temp = 0;

	//take the i2c bus
	i2c1_take(RT_WAITING_FOREVER);
	I2C_DMA_WriteReg(&temp, MS5611_WR_ADDR, MS_COMMAND_D1_4096, 0); //D1
	i2c1_release();

	//�ȴ�10ms
	rt_thread_delay(10);
	//take the i2c bus
	i2c1_take(RT_WAITING_FOREVER);
	I2C_DMA_ReadReg(rx_buf, MS5611_WR_ADDR, MS_COMMAND_ADC_READ, 3); //D1
	i2c1_release();
	
	//get d1 
	d1 = ((unsigned int)rx_buf[0] << 16) | ((unsigned int)rx_buf[1] << 8)|rx_buf[2];
}

//ȡ���¶Ȳ���ֵ
void ms5611_get_temp(void)
{
	unsigned char rx_buf[3] = {0};
	unsigned char temp = 0;

 	//I2C��������
	//take the i2c bus
	i2c1_take(RT_WAITING_FOREVER);
	I2C_DMA_WriteReg(&temp, MS5611_WR_ADDR, MS_COMMAND_D2_4096, 0); //D2
	i2c1_release();

	//�ȴ�10ms
	rt_thread_delay(10);

	//take the i2c bus
	i2c1_take(RT_WAITING_FOREVER);
	I2C_DMA_ReadReg(rx_buf, MS5611_WR_ADDR, MS_COMMAND_ADC_READ, 3); //D2
	i2c1_release();
	
	//get d2 
	d2 = ((unsigned int)rx_buf[0] << 16) | ((unsigned int)rx_buf[1] << 8)|rx_buf[2];

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
	dt = d2 - c5 *256;
	temperature = 2000 + (long long)dt*c6 / 0x800000;
	
	//pressure
	if(temperature < 2000) //lower than 20��C
	{
		t2 = (long long)dt*dt/0x80000000;
		off2 = 5*(temperature - 2000)*(temperature - 2000)/2;
		sens2 = 5*(temperature - 2000)*(temperature - 2000)/4;
		
		if(temperature < -1500)//lower than -15��C
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
	
	//�����¶�
	temperature -= t2;
	
	//air pressure
	off = (long long)c2*65536 + ((long long)c4*dt)/128  - off2;
	sens = (long long)c1*32768 + ((long long)c3*dt)/256 - sens2;
	p = ((long long)d1*sens/0x200000 - off)/32768;
	
}


//�˲�����
#define AIR_AVG_WINDOWS 64

//compute the temperature and the air pressure
void ms5611_press_temp_update(void)
{
	static float altitude_intergrate[AIR_AVG_WINDOWS] = {0};
	static float climb_intergrate[AIR_AVG_WINDOWS] = {0};
	static float altitude_bef = 0 ;
	static int air_avg_times = 0;
	static unsigned int sys_tick_bef = 0;
	static unsigned int seconde_bef = 0xffffffff;
	
	//temperature update rate 1hz is enough
	if(seconde_bef != system_info.startup_second)
	{
		seconde_bef = system_info.startup_second;
		ms5611_get_temp();
	}
	ms5611_get_press();
	ms5611_calculate();
	
	//����ѹ��Ϊ0����������Դ˴�����
	if(p <= 0)
		return;
	
	//������ѹ�� �¶�
	air_info.temperature = (double)temperature / 100.0;
	air_info.air_press 	 = (double)p / 100.0 *100.0;
	
	//����߶�
	altitude_intergrate[air_avg_times] = 
		44330.f * (1.f - pow(((float)air_info.air_press / 101325.f), 1.f / 5.225f));
	
	//����������
 	climb_intergrate[air_avg_times] = 
		(air_info.altitude - altitude_bef ) / ((float)(rt_tick_get() - sys_tick_bef) / (float)RT_TICK_PER_SECOND);
	//��¼ʱ��,�´���������
	sys_tick_bef = rt_tick_get();	
	
	//��¼ʵʱ�߶ȣ������ʣ�������
	air_info.rt_altitude = altitude_intergrate[air_avg_times];
	air_info.rt_climb = climb_intergrate[air_avg_times];
	
	//�˲���
	{
		int count = AIR_AVG_WINDOWS;	//��ѭ������
		int index = air_avg_times;		//�����ǵڼ���ֵ
		int weight = AIR_AVG_WINDOWS;	//Ȩֵ
		int total_weight = 0;			//��Ȩֵ
		double altitude = 0; 
		double climb = 0;
	
		while(count--)
		{
			//����Ȩֵ����
			altitude += altitude_intergrate[index] * weight;
			climb += climb_intergrate[index] * weight;
			
			total_weight += weight; //��¼Ȩֵ
			weight--;				//Ȩֵ�½�
			
			index++;
			if(index >= AIR_AVG_WINDOWS)
				index = 0;
		}
		//��¼��һ�εĸ߶ȣ����ڼ���������
		altitude_bef = air_info.altitude;
		
		//д���µĸ߶�
		air_info.altitude  = altitude / total_weight;
		
		//��¼������
		air_info.climb  = climb / total_weight;
	
		//��Ǹ���
		air_info.count ++;
	}
	//�л�����һ������
	air_avg_times ++;
	if(air_avg_times >= AIR_AVG_WINDOWS)
		air_avg_times = 0;
		
	
// 	air_info.temperature = (double)temperature / 100.0;
// 	air_info.air_press 	 = (double)p / 100.0 *100.0;
// 	
// 	altitude_intergrate += 44330.f * (1.f - pow(((float)air_info.air_press / 101325.f), 1.f / 5.225f));
// 	alt_int_times ++;
// 	
// 	//�߶�����8��ƽ���󣬼�¼��������������
// 	if(alt_int_times >= 32)
// 	{
// 		air_info.altitude = altitude_intergrate/(float)alt_int_times;
// 		
// 		//����������
// 		air_info.climb = (air_info.altitude - altitude_bef ) /
// 							((float)(rt_tick_get() - sys_tick_bef) / (float)RT_TICK_PER_SECOND);
// 		
// 		sys_tick_bef = rt_tick_get();		//��¼�˴�ʱ��
// 		altitude_bef = air_info.altitude;	//��¼���θ߶�
// 		altitude_intergrate = 0;            //�߶Ȼ�������
// 		alt_int_times = 0;					//���ִ�������
// 	}
// 	

	
}

