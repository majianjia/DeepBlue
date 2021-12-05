/*
 * File      : thread_position.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-1-22    majianjia   the first version
 */
 
#include "stm32f4xx.h"
#include <rtthread.h>

#include "math.h"

#include "thread_position.h"
#include "time_measure.h"
#include "struct_all.h"

#include "db_can_message.h"

/* 四元数姿态算法 */

//四元数,使用全局变量来保持上一次的四元数值  
double x = 0.0;	
double y = 0.0;
double z = 0.0;
double w = 1.0;	 		  

//核心算法1：欧拉角转四元数	（输入弧度）
struct _quaternion from_euler_angle(struct _euler_angle ea)
{
	static struct _quaternion out;

    double cos_roll  = cos(ea.roll * 0.5);
    double sin_roll  = sin(ea.roll * 0.5);
    double cos_pitch = cos(ea.pitch * 0.5);
    double sin_pitch = sin(ea.pitch * 0.5);
    double cos_yaw   = cos(ea.yaw * 0.5);
    double sin_yaw   = sin(ea.yaw * 0.5);

    out.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    out.x = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    out.y = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    out.z = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;

	return out;
}		 

//核心算法2：四元数转欧拉角	（输出弧度）
double clamp(double a, double min, double max)
{
	if(a > max) return max;
	if(a < min)	return min;
	return a;
}

struct _euler_angle to_euler_angle(void) 
{
    static struct _euler_angle ea;

    ea.roll  = atan2(2.0 * (w * z + x * y) , 1.0 - 2.0  * (z * z + x * x));
    ea.pitch = asin (clamp(2.0  * (w * x - y * z) , -1.0 , 1.0));	  
    ea.yaw   = atan2(2.0 * (w * y + z * x) , 1.0 - 2.0  * (x * x + y * y));

    return ea;
}

//核心算法3，四元数乘法
void normalize(struct _quaternion *c)
{
    double s = sqrt(c->w*c->w + c->x*c->x + c->y*c->y + c->z*c->z);

   	c->w /= s;
    c->x /= s;
    c->y /= s;
    c->z /= s;
}
//四元数乘法
struct _quaternion multiply(struct _quaternion b)
{
    static struct _quaternion c;

    c.w = w*b.w - x*b.x - y*b.y - z*b.z;
    c.x = w*b.x + x*b.w + y*b.z - z*b.y;
    c.y = w*b.y - x*b.z + y*b.w + z*b.x;
    c.z = w*b.z + x*b.y - y*b.x + z*b.w;

    normalize(&c);

    return c;
}

/* 更新四元数 */
void update_quaternion(struct _euler_angle angle_in)
{
	struct _quaternion now;
	
	now = from_euler_angle(angle_in);			//转换欧拉角为四元数
	now = multiply(now);						//四元数乘法

	x = now.x;									//保存本次四元数结果
	y =	now.y;
	z = now.z;
	w = now.w;	
}

/* posture_computer 姿态计算函数 */
//输入 最近的角速度
//输出 当前姿态
struct _euler_angle posture_computer(struct _euler_angle angle_in)
{
	struct _quaternion now;
	
	now = from_euler_angle(angle_in);				//转换欧拉角为四元数
	now = multiply(now);						//四元数乘法

	x = now.x;									//保存本次四元数结果
	y =	now.y;
	z = now.z;
	w = now.w;	

	return to_euler_angle();						//直接用已经保存的四元数来计算输出欧拉角
}



//sem
static struct rt_semaphore pos_sem;
//初始化参数
void position_init(void)
{
	rt_sem_init(&pos_sem,"position",0 ,RT_IPC_FLAG_FIFO);
}

void position_release_start(void)
{
	rt_sem_release(&pos_sem);	
}


//方便调试，先放在外面
double acc_factor = 0.5;			//数据融合的加速度计衰减系数

//position线程主体
void thread_position(void)
{
	//欧拉角
	struct _euler_angle in;
	struct _euler_angle out;
	
	//初始化定时器 and 邮箱
	position_init();
	
	//主循环
	while(1)
	{
		//wait until gyro frame update
		rt_sem_take(&pos_sem, RT_TICK_PER_SECOND);	
start_measure_time();		
		//用加速度计计算倾角
		if(acc.axis.x == 0 && acc.axis.y == 0 &&acc.axis.z == 0) //data wrong
			continue ;
		acc.g = sqrt(acc.axis.y * acc.axis.y + acc.axis.z * acc.axis.z + acc.axis.x * acc.axis.x);
		acc.angle.pitch = asin(acc.axis.x / acc.g);
		if(acc.axis.z >= 0 )
			acc.angle.roll  = asin(acc.axis.y / acc.g);
		else if(acc.axis.z < 0 && acc.axis.y <= 0)
			acc.angle.roll  = -PI - (asin(acc.axis.y / acc.g));
		else if(acc.axis.z < 0 && acc.axis.y > 0)
			acc.angle.roll  = PI - (asin(acc.axis.y / acc.g));
return_time();
		//陀螺仪的值换算成弧度
		in.pitch = gyro.rad_pitch /(double)AHRS_FREQUENCY;
		in.roll  = gyro.rad_roll  /(double)AHRS_FREQUENCY;
		in.yaw   = gyro.rad_yaw   /(double)AHRS_FREQUENCY;
		
		//加表修正值范围
		#define acc_factor_max 1.0f 
		#define acc_factor_min 0.1f

return_time();		
		//加速度对比上一次姿态角度，进行融合的过程
		if(rt_tick_get() >= RT_TICK_PER_SECOND*3) //前几个数据会出错
		{
			//G值偏差较大时，修改acc融合值 +-1.1g 内才能修正
			acc_factor = acc_factor_max - (acc.g -	1.f)*(acc.g - 1.f) * 1000.f;
			//限幅
			if(acc_factor < acc_factor_min)
				acc_factor = acc_factor_min;
		
			//roll 在 -90~+90范围内
			if((double)acc.angle.roll - out.roll <= 1.0*PI && (double)acc.angle.roll - out.roll >= -1.0*PI)
				in.roll += ((double)acc.angle.roll - out.roll) /(double)AHRS_FREQUENCY * acc_factor;
			//roll 在 90 ~ 180范围
			else if((double)acc.angle.roll - out.roll > 1.0*PI)
				in.roll += (((double)acc.angle.roll - out.roll) - 2.0*PI)/(double)AHRS_FREQUENCY * acc_factor;
			//roll 在 -90~-180
			else if((double)acc.angle.roll - out.roll < -1.0*PI)	
				in.roll += (((double)acc.angle.roll - out.roll) + 2.0*PI)/(double)AHRS_FREQUENCY * acc_factor;
			
			//ROLL在正面时 -90~+90°
			if((double)acc.angle.roll <= 0.5*PI && (double)acc.angle.roll >= -0.5*PI)
			{
				//course 在 -180~+180范围内
				if((double)mag.course/180.0*PI - out.yaw <= 1.0 * PI && (double)mag.course/180.0*PI - out.yaw >= -1.0*PI)
					in.yaw += ((double)mag.course/180.0*PI - out.yaw) /(double)AHRS_FREQUENCY * 1.5f;
				//course 在 90 ~ 180范围
				else if((double)mag.course/180.0*PI - out.yaw > 1.0 * PI)
					in.yaw += (((double)mag.course/180.0*PI - out.yaw) - 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;
				//course 在 -90~-180
				else if((double)mag.course/180.0*PI - out.yaw < -1.0 * PI)	
					in.yaw += (((double)mag.course/180.0*PI - out.yaw) + 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;
				
				//pitch 在 -90~+90范围内 板子在正面时
				in.pitch += ((double)acc.angle.pitch - out.pitch) /(double)AHRS_FREQUENCY * acc_factor;
				
			}
			//反面
			else
			{
				//course 在 -180~+180范围内
				if((double)mag.course/180.0*PI - out.yaw <= 1.0 * PI && (double)mag.course/180.0*PI - out.yaw >= -1.0*PI)
					in.yaw -= ((double)mag.course/180.0*PI - out.yaw) /(double)AHRS_FREQUENCY * 1.5f;
				//course 在 90 ~ 180范围
				else if((double)mag.course/180.0*PI - out.yaw > 1.0 * PI)
					in.yaw -= (((double)mag.course/180.0*PI - out.yaw) - 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;
				//course 在 -90~-180
				else if((double)mag.course/180.0*PI - out.yaw < -1.0 * PI)	
					in.yaw -= (((double)mag.course/180.0*PI - out.yaw) + 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;	
				
				//pitch 在反面时
				in.pitch -= ((double)acc.angle.pitch - out.pitch) /(double)AHRS_FREQUENCY * acc_factor;
			}	
		}
		//在前1秒之内
		else
		{
			in.pitch = 0.0;
			in.roll  = 0.0;
			in.yaw   = (double)(mag.course / (double)AHRS_FREQUENCY / 180.0 * PI);	
		}
return_time();	
		//最关键的一句
		out = posture_computer(in);
return_time();
		//rad output
		out_angle.rad_pitch  = out.pitch;
		out_angle.rad_roll   = out.roll;
		out_angle.rad_yaw    = out.yaw;
		//degree output
		out_angle.pitch = out.pitch * 180.0 / PI;
		out_angle.roll  = out.roll  * 180.0 / PI;
		out_angle.yaw   = out.yaw   * 180.0 / PI;
return_time();
		
		{
			int can_message_send(CanTxMsg *TxMessage);
			CanTxMsg TxMessage;
			static int i = 0;
			i++;
			//500HZ send altitude infomation to canbus
			if(i%4 == 0)
			{			
				float pitch = out_angle.pitch;
				float roll = out_angle.roll;
				float yaw = out_angle.yaw;
				float rpitch = out_angle.rad_pitch;
				float rroll = out_angle.rad_roll;
				float ryaw = out_angle.rad_yaw;		
				
				//send new position
				TxMessage.StdId = CAN_TYPE_MASTER | CAN_ALT_ANGLE | CAN_MESSAGE_INDEX0;
				TxMessage.ExtId = 0x00;
				TxMessage.RTR = CAN_RTR_DATA;
				TxMessage.IDE = CAN_ID_STD;
				TxMessage.DLC = 8;
				
				//copy data to message
				rt_memcpy(&(TxMessage.Data[0]), &pitch, sizeof(pitch));
				rt_memcpy(&(TxMessage.Data[4]), &roll, sizeof(roll));

				can_message_send(&TxMessage);
				
				TxMessage.StdId = CAN_TYPE_MASTER | CAN_ALT_ANGLE | CAN_MESSAGE_INDEX1;
				//copy data to message
				rt_memcpy(&(TxMessage.Data[0]), &yaw, sizeof(yaw));
				rt_memcpy(&(TxMessage.Data[4]), &rpitch, sizeof(rpitch));

				can_message_send(&TxMessage);		

				TxMessage.StdId = CAN_TYPE_MASTER | CAN_ALT_ANGLE | CAN_MESSAGE_INDEX2;
				//copy data to message
				rt_memcpy(&(TxMessage.Data[0]), &rroll, sizeof(rroll));
				rt_memcpy(&(TxMessage.Data[4]), &ryaw, sizeof(ryaw));

				can_message_send(&TxMessage);		
			}

		}
return_time();
	}
}


