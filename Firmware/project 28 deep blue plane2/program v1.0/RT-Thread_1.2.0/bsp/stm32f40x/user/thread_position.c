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

/* ��Ԫ����̬�㷨 */

//��Ԫ��,ʹ��ȫ�ֱ�����������һ�ε���Ԫ��ֵ  
double x = 0.0;	
double y = 0.0;
double z = 0.0;
double w = 1.0;	 		  

//�����㷨1��ŷ����ת��Ԫ��	�����뻡�ȣ�
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

//�����㷨2����Ԫ��תŷ����	��������ȣ�
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

//�����㷨3����Ԫ���˷�
void normalize(struct _quaternion *c)
{
    double s = sqrt(c->w*c->w + c->x*c->x + c->y*c->y + c->z*c->z);

   	c->w /= s;
    c->x /= s;
    c->y /= s;
    c->z /= s;
}
//��Ԫ���˷�
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

/* ������Ԫ�� */
void update_quaternion(struct _euler_angle angle_in)
{
	struct _quaternion now;
	
	now = from_euler_angle(angle_in);			//ת��ŷ����Ϊ��Ԫ��
	now = multiply(now);						//��Ԫ���˷�

	x = now.x;									//���汾����Ԫ�����
	y =	now.y;
	z = now.z;
	w = now.w;	
}

/* posture_computer ��̬���㺯�� */
//���� ����Ľ��ٶ�
//��� ��ǰ��̬
struct _euler_angle posture_computer(struct _euler_angle angle_in)
{
	struct _quaternion now;
	
	now = from_euler_angle(angle_in);				//ת��ŷ����Ϊ��Ԫ��
	now = multiply(now);						//��Ԫ���˷�

	x = now.x;									//���汾����Ԫ�����
	y =	now.y;
	z = now.z;
	w = now.w;	

	return to_euler_angle();						//ֱ�����Ѿ��������Ԫ�����������ŷ����
}



//sem
static struct rt_semaphore pos_sem;
//��ʼ������
void position_init(void)
{
	rt_sem_init(&pos_sem,"position",0 ,RT_IPC_FLAG_FIFO);
}

void position_release_start(void)
{
	rt_sem_release(&pos_sem);	
}


//������ԣ��ȷ�������
double acc_factor = 0.5;			//�����ںϵļ��ٶȼ�˥��ϵ��

//position�߳�����
void thread_position(void)
{
	//ŷ����
	struct _euler_angle in;
	struct _euler_angle out;
	
	//��ʼ����ʱ�� and ����
	position_init();
	
	//��ѭ��
	while(1)
	{
		//wait until gyro frame update
		rt_sem_take(&pos_sem, RT_TICK_PER_SECOND);	
start_measure_time();		
		//�ü��ٶȼƼ������
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
		//�����ǵ�ֵ����ɻ���
		in.pitch = gyro.rad_pitch /(double)AHRS_FREQUENCY;
		in.roll  = gyro.rad_roll  /(double)AHRS_FREQUENCY;
		in.yaw   = gyro.rad_yaw   /(double)AHRS_FREQUENCY;
		
		//�ӱ�����ֵ��Χ
		#define acc_factor_max 1.0f 
		#define acc_factor_min 0.1f

return_time();		
		//���ٶȶԱ���һ����̬�Ƕȣ������ںϵĹ���
		if(rt_tick_get() >= RT_TICK_PER_SECOND*3) //ǰ�������ݻ����
		{
			//Gֵƫ��ϴ�ʱ���޸�acc�ں�ֵ +-1.1g �ڲ�������
			acc_factor = acc_factor_max - (acc.g -	1.f)*(acc.g - 1.f) * 1000.f;
			//�޷�
			if(acc_factor < acc_factor_min)
				acc_factor = acc_factor_min;
		
			//roll �� -90~+90��Χ��
			if((double)acc.angle.roll - out.roll <= 1.0*PI && (double)acc.angle.roll - out.roll >= -1.0*PI)
				in.roll += ((double)acc.angle.roll - out.roll) /(double)AHRS_FREQUENCY * acc_factor;
			//roll �� 90 ~ 180��Χ
			else if((double)acc.angle.roll - out.roll > 1.0*PI)
				in.roll += (((double)acc.angle.roll - out.roll) - 2.0*PI)/(double)AHRS_FREQUENCY * acc_factor;
			//roll �� -90~-180
			else if((double)acc.angle.roll - out.roll < -1.0*PI)	
				in.roll += (((double)acc.angle.roll - out.roll) + 2.0*PI)/(double)AHRS_FREQUENCY * acc_factor;
			
			//ROLL������ʱ -90~+90��
			if((double)acc.angle.roll <= 0.5*PI && (double)acc.angle.roll >= -0.5*PI)
			{
				//course �� -180~+180��Χ��
				if((double)mag.course/180.0*PI - out.yaw <= 1.0 * PI && (double)mag.course/180.0*PI - out.yaw >= -1.0*PI)
					in.yaw += ((double)mag.course/180.0*PI - out.yaw) /(double)AHRS_FREQUENCY * 1.5f;
				//course �� 90 ~ 180��Χ
				else if((double)mag.course/180.0*PI - out.yaw > 1.0 * PI)
					in.yaw += (((double)mag.course/180.0*PI - out.yaw) - 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;
				//course �� -90~-180
				else if((double)mag.course/180.0*PI - out.yaw < -1.0 * PI)	
					in.yaw += (((double)mag.course/180.0*PI - out.yaw) + 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;
				
				//pitch �� -90~+90��Χ�� ����������ʱ
				in.pitch += ((double)acc.angle.pitch - out.pitch) /(double)AHRS_FREQUENCY * acc_factor;
				
			}
			//����
			else
			{
				//course �� -180~+180��Χ��
				if((double)mag.course/180.0*PI - out.yaw <= 1.0 * PI && (double)mag.course/180.0*PI - out.yaw >= -1.0*PI)
					in.yaw -= ((double)mag.course/180.0*PI - out.yaw) /(double)AHRS_FREQUENCY * 1.5f;
				//course �� 90 ~ 180��Χ
				else if((double)mag.course/180.0*PI - out.yaw > 1.0 * PI)
					in.yaw -= (((double)mag.course/180.0*PI - out.yaw) - 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;
				//course �� -90~-180
				else if((double)mag.course/180.0*PI - out.yaw < -1.0 * PI)	
					in.yaw -= (((double)mag.course/180.0*PI - out.yaw) + 2.0*PI)/(double)AHRS_FREQUENCY * 1.5f;	
				
				//pitch �ڷ���ʱ
				in.pitch -= ((double)acc.angle.pitch - out.pitch) /(double)AHRS_FREQUENCY * acc_factor;
			}	
		}
		//��ǰ1��֮��
		else
		{
			in.pitch = 0.0;
			in.roll  = 0.0;
			in.yaw   = (double)(mag.course / (double)AHRS_FREQUENCY / 180.0 * PI);	
		}
return_time();	
		//��ؼ���һ��
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


