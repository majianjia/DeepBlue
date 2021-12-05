/*
 * File      : thread_sensors.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-10-24    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include "math.h"

#include "struct_all.h"
#include "thread_sensors.h"
#include "time_measure.h"


#include "mpu6050.h"
#include "hmc5883.h"
#include "ms5611.h"

#include "stm32f4_i2c.h"
#include "thread_position.h"


unsigned char HMC_ID;
unsigned char MPU_ID;

#define  MPU_UPDATE_PERIOD  (1000/AHRS_FREQUENCY) //ms
#define  ACC_BANDWIDTH  	20					  //HZ acc low pass filter
#define  ACC_AVG_WINDOWS    32					//64������

unsigned char mpu_raw[14];
unsigned char hmc_raw[6];

/* ��ʱ���Ŀ��ƿ�*/
static struct rt_timer tim_mpu;
static struct rt_timer tim_hmc;

/* ������ƿ� */
static struct rt_semaphore sem_mpu;
static struct rt_semaphore sem_hmc;

/* ��ʱ��mpu ��ʱ����*/
static void timer_mpu(void* parameter)
{
	rt_sem_release(&sem_mpu);
}

/* ��ʱ��hmc ��ʱ����*/
static void timer_hmc(void* parameter)
{
	rt_sem_release(&sem_hmc);
}


/* Ӳ����ʼ�� */
void sensors_hw_init(void)
{
	I2C_Config();
}


/* ��ʼ�����а��ش����� */
unsigned int sensors_init(void)
{	
	unsigned char temp;
	
	//first of all we should initialize the i2c mutex
	i2c1_mutex_init();
	
	rt_thread_delay(100);
	
	//initialization of the ms5611
	i2c1_take(RT_WAITING_FOREVER);
	ms5611_parameter_get();
	i2c1_release();
	
	//take the i2c bus
	i2c1_take(RT_WAITING_FOREVER);
	
	//HMC5883L initialization. ID should be 0x48
	I2C_DMA_ReadReg(&HMC_ID, HMC5883_RD_ADDR, HMC_ID_REG_A, 1 );
	rt_thread_delay(5);
	temp = 0x78;
	I2C_DMA_WriteReg(&temp, HMC5883_WR_ADDR, CFG_A , 1); //75HZ out put rate
	temp = 0x20;
	I2C_DMA_WriteReg(&temp, HMC5883_WR_ADDR, CFG_B , 1); //1.3GASS
	temp = 0x00;
	I2C_DMA_WriteReg(&temp, HMC5883_WR_ADDR, MODE , 1); //continue measurment mode	

	//MPU6050 initialization. ID should be 0x68
	I2C_DMA_ReadReg(&MPU_ID, MPU6050_RD_ADDRESS, MPU6050_WHO_AM_I, 1 );
	rt_thread_delay(5);
	
	temp = 0x80;
	I2C_DMA_WriteReg(&temp, MPU6050_WR_ADDRESS, 0x6B, 1); //PWR_MGMT_1    -- DEVICE_RESET 1
	temp = 0x00;
    I2C_DMA_WriteReg(&temp, MPU6050_WR_ADDRESS, 0x19, 1); //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	temp = 0x00;
 	I2C_DMA_WriteReg(&temp, MPU6050_WR_ADDRESS, 0x1A, 1); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	temp = 0x03;
	I2C_DMA_WriteReg(&temp, MPU6050_WR_ADDRESS, 0x6B, 1); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	temp = 0x00;
	I2C_DMA_WriteReg(&temp, MPU6050_WR_ADDRESS, 0x1B, 1); //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 250 deg/sec	
	temp = 0x00;
	I2C_DMA_WriteReg(&temp, MPU6050_WR_ADDRESS, 0x1C, 1); //ACC_CONFIG   -- FS_SEL = 3: Full scale set to 2G
	
	//release the i2c bus
	i2c1_release();
	
	return 0;
}

/* thread_sensors */
ALIGN(RT_ALIGN_SIZE)
static char thread_air_press_stack[1024];
static struct rt_thread thread_air_press_handle;

void entry_thread_air_press(void* parameter)
{		
	while(1)
	{			
		//update the new air pressure data
		ms5611_press_temp_update();
		
		//�ʵ���ʱ
		//rt_thread_delay(5);
	}
}

//MPU6050�޸����̺�����
//type ��ѡ MPU_ACC/MPU_GYRO
//range ��ѡ 2 4 8 16 , 250 500 1000 2000
#define MPU_ACC  0
#define MPU_GYRO 1

int mpu_change_range(unsigned int type, unsigned int range)
{
	unsigned char reg;
	unsigned char data;
	
	//���ٶȼ�
	if(type == MPU_ACC)
	{
		reg = 0x1c;
		switch(range)
		{
			case 2: data = 0x00; break;
			case 4: data = 0x08; break;
			case 8: data = 0x10; break;
			case 16:data = 0x18; break;
			default:return 1;
		}
	}
	//������
	else if(type == MPU_GYRO)
	{
		reg = 0x1b;
		switch(range)
		{
			case 250:  data = 0x00; break;
			case 500:  data = 0x08; break;
			case 1000: data = 0x10; break;
			case 2000: data = 0x18; break;
			default:return 1;
		}
	}
	
	//д��Ĵ���
	//take the i2c bus
	i2c1_take(RT_WAITING_FOREVER);
	I2C_DMA_WriteReg(&data, MPU6050_WR_ADDRESS, reg, 1);

	i2c1_release();
	
	return 0;
}


//mpu6050 ԭʼ����
struct _mpu{
	short int gyro_x;
	short int gyro_y;
	short int gyro_z;
	
	short int gyro_x_offset;
	short int gyro_y_offset;
	short int gyro_z_offset;
	
	short int acc_x;
	short int acc_y;
	short int acc_z;
	
	short int acc_x_offset;
	short int acc_y_offset;
	short int acc_z_offset;
	
	short int temp;
	float temperature;
	
	unsigned int gyro_range;	//���� factor = 65536 / (range * 2)
	unsigned int acc_range;
}mpu;

//thread_mpu �̳߳�ʼ��
void thread_mpu_timer_init(void)
{
	/* ��ʼ��һ���ź��� */
	rt_sem_init(&sem_mpu,"mpu",0 ,RT_IPC_FLAG_FIFO);
	
	/* ������ʱ��mpu */
	rt_timer_init(&tim_mpu,
					"mpu", 				/* ��ʱ������ */
					timer_mpu, 				/* ��ʱʱ�ص��Ĵ�����*/
					RT_NULL,				/* ��ʱ��������ڲ���*/
					MPU_UPDATE_PERIOD, 					/* ��ʱ���ȣ���OS TickΪ��λ*/
					RT_TIMER_FLAG_PERIODIC);/* �����Զ�ʱ��*/	
	/* ������ʱ��*/
	rt_timer_start(&tim_mpu);
}

/* thread_mpu */
ALIGN(RT_ALIGN_SIZE)
static char thread_mpu_stack[1024];
static struct rt_thread thread_mpu_handle;

void entry_thread_mpu(void* parameter)
{
	unsigned char count;
 	static float acc_x[ACC_AVG_WINDOWS];
	static float acc_y[ACC_AVG_WINDOWS];
	static float acc_z[ACC_AVG_WINDOWS];
	static int acc_avg_times = 0;
	
// 	static double gyro_x,gyro_y,gyro_z;
 	
	
		
	//��ʼ������
	mpu.gyro_range = 250;
	mpu.acc_range  = 2;
	
	//��ʼ��
	mpu.gyro_x_offset = -220;
	mpu.gyro_y_offset = 440;
	mpu.gyro_z_offset = -267;

	mpu.acc_x_offset = 450;
	mpu.acc_y_offset = 190;
	mpu.acc_z_offset = -1784;
	
	//��ʼ����ʱ��������
	thread_mpu_timer_init();	
	
	//��ѭ��
	while(1)
	{			
		//�ȴ���ʱ�������Ĺ̶�ʱ��Ķ�ʱ�ʼ�
		rt_sem_take(&sem_mpu, RT_WAITING_FOREVER);

		//��ȡmpu6050
		//take the i2c bus
		i2c1_take(RT_WAITING_FOREVER);
		count = 14;
		I2C_DMA_Read(mpu_raw, MPU6050_RD_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &count);
		i2c1_release();
	
		//���ݴ���
		mpu.acc_x = (short)mpu_raw[0] << 8 | (short)mpu_raw[1];
		mpu.acc_y = (short)mpu_raw[2] << 8 | (short)mpu_raw[3];
		mpu.acc_z = (short)mpu_raw[4] << 8 | (short)mpu_raw[5];
		
		mpu.temp  = (short)mpu_raw[6] << 8 | (short)mpu_raw[7];
		
		mpu.gyro_x = (short)mpu_raw[8] << 8  | (short)mpu_raw[9];
		mpu.gyro_y = (short)mpu_raw[10] << 8 | (short)mpu_raw[11];
		mpu.gyro_z = (short)mpu_raw[12] << 8 | (short)mpu_raw[13];
		
		//��������ȫ�����ݽṹ ����acc��x�������෴
		acc_x[acc_avg_times] = -((float)mpu.acc_x - mpu.acc_x_offset/ (int)(mpu.acc_range / 2)) 
				/(65536.f / 2.f / (float)mpu.acc_range);
		acc_y[acc_avg_times] = ((float)mpu.acc_y - mpu.acc_y_offset / (int)(mpu.acc_range / 2)) 
				/(65536.f / 2.f / (float)mpu.acc_range);
		acc_z[acc_avg_times] = ((float)mpu.acc_z - mpu.acc_z_offset / (int)(mpu.acc_range / 2)) 
				/(65536.f / 2.f / (float)mpu.acc_range);
		
		//�����˲�ǰ��ֵ�������ã�		
		acc.rt_x = acc_x[acc_avg_times];
		acc.rt_y = acc_y[acc_avg_times];
		acc.rt_z = acc_z[acc_avg_times];
						
		{
			int count = ACC_AVG_WINDOWS;	//��ѭ������
			int index = acc_avg_times;		//�����ǵڼ���ֵ
			int weight = ACC_AVG_WINDOWS;	//Ȩֵ
			int total_weight = 0;			//��Ȩֵ
			double x = 0; 
			double y = 0;
			double z = 0;
			
			while(count--)
			{
				//Ȩֵ
				x += acc_x[index] * weight;
				y += acc_y[index] * weight;
				z += acc_z[index] * weight;
				
				total_weight += weight; //��¼Ȩֵ
				weight--;				//Ȩֵ�½�
				
				index++;
				if(index >= ACC_AVG_WINDOWS)
					index = 0;
			}
			
			//д���µ�accֵ
			acc.x  = x / total_weight;
			acc.y  = y / total_weight;
			acc.z  = z / total_weight;
			
			acc.count ++;
		}
		//�л�����һ������
		acc_avg_times ++;
		if(acc_avg_times >= ACC_AVG_WINDOWS)
			acc_avg_times = 0;
		
		//gyro 
		gyro.pitch = ((double)mpu.gyro_x - (mpu.gyro_x_offset / (int)(mpu.gyro_range / 250))) 
					/ (65536.0 / 2.0 / (double)mpu.gyro_range);
		gyro.roll  = ((double)mpu.gyro_y - (mpu.gyro_y_offset / (int)(mpu.gyro_range / 250))) 
					/ (65536.0 / 2.0 / (double)mpu.gyro_range);
		gyro.yaw   = ((double)mpu.gyro_z - (mpu.gyro_z_offset / (int)(mpu.gyro_range / 250))) 
					/ (65536.0 / 2.0 / (double)mpu.gyro_range);
		gyro.count ++;
		

		
		//�����ź��� ������̬����
		//֪ͨposition���м���
		position_release_start(); 
		
		//����mpu�¶�
		mpu.temperature = 35.f + ((float)(mpu.temp - 521)) / 340.f;

		//GYRO�Զ��Ż�����
		#define MPU_GYRO_AUTO_RANGE_MIN (0.2 * 32768)
		#define MPU_GYRO_AUTO_RANGE_MAX (0.8 * 32768)
		
		//GYRO���̹����ˣ�����С����
		if((mpu.gyro_x < MPU_GYRO_AUTO_RANGE_MIN && mpu.gyro_x > -MPU_GYRO_AUTO_RANGE_MIN) &&
		   (mpu.gyro_y < MPU_GYRO_AUTO_RANGE_MIN && mpu.gyro_y > -MPU_GYRO_AUTO_RANGE_MIN) &&
		   (mpu.gyro_z < MPU_GYRO_AUTO_RANGE_MIN && mpu.gyro_z > -MPU_GYRO_AUTO_RANGE_MIN))
		{
			switch(mpu.gyro_range)
			{
				case 2000: mpu.gyro_range = 1000; break;
				case 1000: mpu.gyro_range = 500;  break;
				case 500 : mpu.gyro_range = 250;  break;
				default  : mpu.gyro_range = 250;  break;
			}
			//��������
			mpu_change_range(MPU_GYRO, mpu.gyro_range);		
		}
		
		//gyro ����̫С��
		if((mpu.gyro_x > MPU_GYRO_AUTO_RANGE_MAX || mpu.gyro_x < -MPU_GYRO_AUTO_RANGE_MAX) ||
		   (mpu.gyro_y > MPU_GYRO_AUTO_RANGE_MAX || mpu.gyro_y < -MPU_GYRO_AUTO_RANGE_MAX) ||
		   (mpu.gyro_z > MPU_GYRO_AUTO_RANGE_MAX || mpu.gyro_z < -MPU_GYRO_AUTO_RANGE_MAX))
		{
			switch(mpu.gyro_range)
			{
				case 250  : mpu.gyro_range = 500;   break;
				case 500  : mpu.gyro_range = 1000;  break;
				case 1000 : mpu.gyro_range = 2000;  break;
				default   : mpu.gyro_range = 2000;  break;
			}
			//��������
			mpu_change_range(MPU_GYRO, mpu.gyro_range);		
		}
		
		//ACC�Զ��Ż�����
		#define MPU_ACC_AUTO_RANGE_MIN (0.3 * 32768)
		#define MPU_ACC_AUTO_RANGE_MAX (0.6 * 32768)
		//ACC ���̹���
		if((mpu.acc_x < MPU_ACC_AUTO_RANGE_MIN && mpu.acc_x > -MPU_ACC_AUTO_RANGE_MIN) &&
		   (mpu.acc_y < MPU_ACC_AUTO_RANGE_MIN && mpu.acc_y > -MPU_ACC_AUTO_RANGE_MIN) &&
		   (mpu.acc_z < MPU_ACC_AUTO_RANGE_MIN && mpu.acc_z > -MPU_ACC_AUTO_RANGE_MIN))
		{
			switch(mpu.acc_range)
			{
				case 16 : mpu.acc_range = 8;  break;
				case 8  : mpu.acc_range = 4;  break;
				case 4  : mpu.acc_range = 2;  break;
				default : mpu.acc_range = 2;  break;
			}
			//��������
			mpu_change_range(MPU_ACC, mpu.acc_range);		
		}
		
		//ACC ���̹�С
		if((mpu.acc_x > MPU_ACC_AUTO_RANGE_MAX || mpu.acc_x < -MPU_ACC_AUTO_RANGE_MAX) ||
		   (mpu.acc_y > MPU_ACC_AUTO_RANGE_MAX || mpu.acc_y < -MPU_ACC_AUTO_RANGE_MAX) ||
		   (mpu.acc_z > MPU_ACC_AUTO_RANGE_MAX || mpu.acc_z < -MPU_ACC_AUTO_RANGE_MAX))
		{
			switch(mpu.acc_range)
			{
				case 2 : mpu.acc_range  = 4;  break;
				case 4  : mpu.acc_range = 8;  break;
				case 8  : mpu.acc_range = 16; break;
				default : mpu.acc_range = 16; break;
			}
			//��������
			mpu_change_range(MPU_ACC, mpu.acc_range);		
		}

	}
}


//HMC5883L ���̶�Ӧ�ļĴ���ֵ
#define HMC_RANGE_088_REG ((unsigned char)0x00 << 5)
#define HMC_RANGE_13_REG  ((unsigned char)0x01 << 5)
#define HMC_RANGE_19_REG  ((unsigned char)0x02 << 5)
#define HMC_RANGE_25_REG  ((unsigned char)0x03 << 5)
#define HMC_RANGE_40_REG  ((unsigned char)0x04 << 5)
#define HMC_RANGE_47_REG  ((unsigned char)0x05 << 5)
#define HMC_RANGE_56_REG  ((unsigned char)0x06 << 5)
#define HMC_RANGE_81_REG  ((unsigned char)0x07 << 5)

//���̶�Ӧ��LSB
#define HMC_RANGE_088 1370
#define HMC_RANGE_13  1090
#define HMC_RANGE_19  820
#define HMC_RANGE_25  660
#define HMC_RANGE_40  440
#define HMC_RANGE_47  390
#define HMC_RANGE_56  330
#define HMC_RANGE_81  230

//hmc5883l ԭʼ����
struct _hmc{
	short int x;
	short int y;
	short int z;
	
	unsigned int range;	//����
}hmc;


//У׼����
struct 
{
	float xsf;	   //��Բ����
	float ysf;	   //��Բ����
	float zsf;
	int xoff;	   //���
	int yoff;	   //���
	int zoff;
	
	int mea_range; //����
	float mea_lsb; //LSB	

	int xmax;
	int xmin;
	int ymax;
	int ymin;
	int zmax;
	int zmin;

	unsigned int max_value ;

}hmc_offset;

/* ������ʼ�� */
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

//*****************����ʱ�ã��Ժ󽫻�д��flash��**************
	hmc_offset.xmax = 453;
	hmc_offset.xmin = -382;
	hmc_offset.ymax = 295;
	hmc_offset.ymin = -510;
	hmc_offset.zmax = 480;
	hmc_offset.zmin = -314;


	hmc_offset.xsf = 1.0;
	hmc_offset.ysf = (float)(hmc_offset.xmax - hmc_offset.xmin) / (float)(hmc_offset.ymax - hmc_offset.ymin);		
	hmc_offset.zsf = (float)(hmc_offset.xmax - hmc_offset.xmin) / (float)(hmc_offset.zmax - hmc_offset.zmin);		
	hmc_offset.xoff =((hmc_offset.xmax - hmc_offset.xmin) / 2 - hmc_offset.xmax) * hmc_offset.xsf;
	hmc_offset.yoff =((hmc_offset.ymax - hmc_offset.ymin) / 2 - hmc_offset.ymax) * hmc_offset.ysf;	
	hmc_offset.zoff =((hmc_offset.zmax - hmc_offset.zmin) / 2 - hmc_offset.zmax) * hmc_offset.zsf;
//***************************************************************

	return 0;
}

/* ����OFFSET */
unsigned int get_hmc_offset(void)
{
	static unsigned int count = 0 ; 
	static unsigned int count_check = 0;
	static unsigned int max_value_bef;
	unsigned int temp;

	//����Ѿ������˶���
	count ++;

	//ȡ�õ�ǰ�ų�ǿ��
	temp = sqrt(hmc.y * hmc.y + hmc.z * hmc.z + hmc.x * hmc.x);

	//��������������Ҵ��ڵ�ǰ�ų�ǿ��
	if(temp > hmc_offset.max_value &&
	   hmc.x <= 2047 && hmc.x >= -2048 &&
	   hmc.y <= 2047 && hmc.y >= -2048 &&
	   hmc.z <= 2047 && hmc.y >= -2048)
	{
		hmc_offset.max_value = temp;
	}

	//��ȡX�����ֵ����Сֵ
	if(hmc.x > hmc_offset.xmax && hmc.x <= 2047)
	{
		hmc_offset.xmax = hmc.x;	
	}
	else if(hmc.x < hmc_offset.xmin && hmc.x >= -2048)
	{
		hmc_offset.xmin = hmc.x;
	}
	//��ȡY�����ֵ����Сֵ
	if(hmc.y > hmc_offset.ymax && hmc.y <= 2047)
	{
		hmc_offset.ymax = hmc.y;	
	}
	else if(hmc.y < hmc_offset.ymin && hmc.y >= -2048)
	{
		hmc_offset.ymin = hmc.y;
	}

	//��ȡZ�����ֵ����Сֵ
	if(hmc.z > hmc_offset.zmax && hmc.z <= 2047)
	{
		hmc_offset.zmax = hmc.z;	
	}
	else if(hmc.z < hmc_offset.zmin && hmc.z >= -2048)
	{
		hmc_offset.zmin = hmc.z;
	}

	//��������������ͱ�־ȡֵ����������OFFSET
	if(hmc_offset.xmax - hmc_offset.xmin > hmc_offset.max_value * 0.8 &&
	   hmc_offset.ymax - hmc_offset.ymin > hmc_offset.max_value * 0.8 &&
	   hmc_offset.zmax - hmc_offset.zmin > hmc_offset.max_value * 0.8 &&
	   count > 50*5 )		 //����Ҫ����5��
	{
		hmc_offset.xsf = 1.0;
		hmc_offset.ysf = (float)(hmc_offset.xmax - hmc_offset.xmin) / 
						 (float)(hmc_offset.ymax - hmc_offset.ymin);		
		hmc_offset.zsf = (float)(hmc_offset.xmax - hmc_offset.xmin) / 
						 (float)(hmc_offset.zmax - hmc_offset.zmin);		
	   	hmc_offset.xoff =((hmc_offset.xmax - hmc_offset.xmin) / 2 - hmc_offset.xmax) * hmc_offset.xsf;
		hmc_offset.yoff =((hmc_offset.ymax - hmc_offset.ymin) / 2 - hmc_offset.ymax) * hmc_offset.ysf;	
		hmc_offset.zoff =((hmc_offset.zmax - hmc_offset.zmin) / 2 - hmc_offset.zmax) * hmc_offset.zsf;	

		//ȷ��һ������
		if(count_check < 50)
		{
			//�����������̫�󣬾�����У׼
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

//��ʱ����ʼ��
void thread_hmc_timer_init(void)
{
	/* ��ʼ��һ���ź��� */
	rt_sem_init(&sem_hmc,"hmc",0 ,RT_IPC_FLAG_FIFO);
		
	/* ������ʱ��hmc */
	rt_timer_init(&tim_hmc,					/* ���ƿ� */
					"hmc", 					/* ��ʱ������ */
					timer_hmc, 				/* ��ʱʱ�ص��Ĵ�����*/
					RT_NULL,				/* ��ʱ��������ڲ���*/
					20, 					/* ��ʱ���ȣ���OS TickΪ��λ*/
					RT_TIMER_FLAG_PERIODIC);/* �����Զ�ʱ��*/	
	/* ������ʱ��*/
	rt_timer_start(&tim_hmc);

}


/* thread_hmc */
ALIGN(RT_ALIGN_SIZE)
static char thread_hmc_stack[512];
static struct rt_thread thread_hmc_handle;

void entry_thread_hmc(void* parameter)
{	
	unsigned char count;
	
	float xh;
	float yh;
	float s_pitch;
	float s_roll;
	float c_pitch;
	float c_roll;
	float mag_x;
	float mag_y;
	float mag_z;

	//��ʼ����ʱ��������
	thread_hmc_timer_init();
	
	//��ʼ������
	hmc_factor_init();
	
	while(1)
	{	
		//�ȴ���ʱ�������Ĺ̶�ʱ��Ķ�ʱ�ʼ�
		rt_sem_take(&sem_hmc, RT_WAITING_FOREVER);
		
		//take the i2c bus
		i2c1_take(RT_WAITING_FOREVER);
		count = 6;
		I2C_DMA_Read(hmc_raw, HMC5883_RD_ADDR, 0x03, &count);
		i2c1_release();
		
		//��������
		hmc.x = (short)hmc_raw[0]<<8 | (short)hmc_raw[1];
		hmc.z = (short)hmc_raw[2]<<8 | (short)hmc_raw[3];
		hmc.y = (short)hmc_raw[4]<<8 | (short)hmc_raw[5];
				
		//У׼
//		get_hmc_offset();
		
		//�ж��Ƿ�����
		if(hmc.x >= -2048 && hmc.x <= 2047)
		{
			mag.x = ((float)hmc.x * hmc_offset.xsf + hmc_offset.xoff) / hmc_offset.mea_lsb;
		}
		if(hmc.y >= -2048 && hmc.y <= 2047)
		{
			mag.y = ((float)hmc.y * hmc_offset.ysf + hmc_offset.yoff) / hmc_offset.mea_lsb;
		}
		if(hmc.z >= -2048 && hmc.z <= 2047)
		{
			mag.z = ((float)hmc.z * hmc_offset.zsf + hmc_offset.zoff) / hmc_offset.mea_lsb;
		}
				
// 		//��ʱֻ�ü��ٶȼƵĽǶ��������

		s_pitch = sin(-out_angle.pitch / 180.0 * PI);
		s_roll  = sin(-out_angle.roll  / 180.0 * PI);
		c_pitch = cos(-out_angle.pitch / 180.0 * PI);
		c_roll  = cos(-out_angle.roll  / 180.0 * PI);
		
// 		s_pitch = sin(-acc.angle.pitch);
// 		s_roll  = sin(-acc.angle.roll);
// 		c_pitch = cos(-acc.angle.pitch);
// 		c_roll  = cos(-acc.angle.roll);
		
		mag_x = -mag.x;
		mag_y = mag.y;
		mag_z = -mag.z;
		
		xh = mag_x*c_pitch + mag_y*s_roll*s_pitch - mag_z*c_roll*s_pitch;//X�ᷴ�� Z��Ҳ����
		yh = mag_y*c_roll  + mag_z*s_roll;

		mag.course = ((float)atan2(yh,xh)) * 180.0f / (float)PI;
		
	}
}

/* ����AIR PRESS ���߳� */
void thread_air_press_init(void)
{
	//air_press
	rt_thread_init(&thread_air_press_handle,
                   "airpress",
                   entry_thread_air_press,
                   RT_NULL,
                   &thread_air_press_stack[0],
                   sizeof(thread_air_press_stack),7,1);
    rt_thread_startup(&thread_air_press_handle);
}

/* ����mpu���߳� */
void thread_mpu_init(void)
{
	//mpu
	rt_thread_init(&thread_mpu_handle,
                   "mpu",
                   entry_thread_mpu,
                   RT_NULL,
                   &thread_mpu_stack[0],
                   sizeof(thread_mpu_stack),6,1);
    rt_thread_startup(&thread_mpu_handle);
}

/* ����hmc���߳� */
void thread_hmc_init(void)
{
	//mpu
	rt_thread_init(&thread_hmc_handle,
                   "hmc",
                   entry_thread_hmc,
                   RT_NULL,
                   &thread_hmc_stack[0],
                   sizeof(thread_hmc_stack),6,1);
    rt_thread_startup(&thread_hmc_handle);
}

//send data to mini_ahrs
//just for text
void send_raw_data(void)
{
// 	static unsigned char num = 0;
// 	static unsigned int check_sum = 0;
// 	static unsigned char tx_buf[30];
// 	
// 	
// 	
// 	tx_buf[0] = 0xa5;
// 	tx_buf[1] = 0x5a;
// 	tx_buf[2] = 22;
// 	tx_buf[3] = 0xa2;

// 	tx_buf[4] = mpu_raw[8];
// 	tx_buf[5] = mpu_raw[9];
// 	tx_buf[6] = mpu_raw[10];
// 	tx_buf[7] = mpu_raw[11];
// 	tx_buf[8] = mpu_raw[12];
// 	tx_buf[9] = mpu_raw[13];
// 	
// 	tx_buf[10] = mpu_raw[0];
// 	tx_buf[11] = mpu_raw[1];
// 	tx_buf[12] = mpu_raw[2];
// 	tx_buf[13] = mpu_raw[3];
// 	tx_buf[14] = mpu_raw[4];
// 	tx_buf[15] = mpu_raw[5];
// 	
// 	tx_buf[16] = hmc_raw[0];
// 	tx_buf[17] = hmc_raw[1];
// 	tx_buf[18] = hmc_raw[2];
// 	tx_buf[19] = hmc_raw[3];
// 	tx_buf[20] = hmc_raw[4];
// 	tx_buf[21] = hmc_raw[5];
// 	

// 	for(num = 2; num <= 21; num++)
// 	{
// 		check_sum += tx_buf[num];
// 	}
// 	tx_buf[22] = check_sum &0xff;
// 	tx_buf[23] = 0xaa;
// 	
// 	for(num = 0; num <= 23; num++)
// 		USART_SendData(USART2, tx_buf[num]);

}

/* sensors �߳����� */
void thread_sensors(void)
{

	//��������ʼ��
	sensors_init();

	//���������������߳�
//	thread_air_press_init();
//	thread_hmc_init();
//	thread_mpu_init();
	

	//��ѭ��
	while(1)
	{
	//	send_raw_data();
		rt_thread_delay(20);
	}
}
 
