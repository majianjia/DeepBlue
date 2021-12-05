/*
 * File      : thread_expand.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-04    majianjia   the first version
 */
 
#include "stm32f4xx.h"
#include <rtthread.h>

#include "struct_all.h"
#include "thread_expand.h"
#include "db_can_message.h"

#include "string.h"


static struct rt_timer timer;


//airspeed
void air_speed_decode(CanRxMsg rx)
{
	//is the message an air speed?
	if((rx.StdId & CAN_FILTER) == CAN_AIR_SPEED)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				memcpy(&(air_speed.press_diff), &(rx.Data[0]), sizeof(air_speed.press_diff));	
				memcpy(&(air_speed.speed), &(rx.Data[4]), 4);
			
				air_speed.count ++;
				air_speed.flag = 1;
				break;
			case CAN_MESSAGE_INDEX1:
				memcpy(&(air_speed.rt_speed), &(rx.Data[0]), sizeof(air_speed.rt_speed));	
		
				break;
			default:
				if(air_speed.flag == 1)
					air_speed.flag = 0;
				break;
		}	
	}
	else
	{
		air_speed.flag = 0;
	}
}

void opt_floaw_decode(CanRxMsg rx)
{
	//is the message an ult rang ?
	if((rx.StdId & CAN_FILTER) == CAN_OPT)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				memcpy(&(opt_flow.x), rx.Data, sizeof(opt_flow.x));
				memcpy(&(opt_flow.y), &rx.Data[4], sizeof(opt_flow.y));

				opt_flow.flag = 1;
			
			break;

			
			default :break;			
		}
		opt_flow.count ++;
		
	}
	else
	{
		opt_flow.flag = 0;
	}

}

void ultrasonic_ranging_decode(CanRxMsg rx)
{
	//is the message an ult rang ?
	if((rx.StdId & CAN_FILTER) == CAN_ULT_RANG)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				memcpy(&(ult_data.distance), rx.Data, sizeof(ult_data.distance));
				memcpy(&(ult_data.speed), &rx.Data[4], sizeof(ult_data.speed));

				//暂时不计入三角函数
				ult_data.altitude = ult_data.distance;
				
				ult_data.flag = 1;
			
			break;
			
			case CAN_MESSAGE_INDEX1:
				
			break;
			
			default :break;			
		}
		ult_data.count ++;
		
	}
	else
	{
		ult_data.flag = 0;
	}

}


//battery
void battery_decode(CanRxMsg rx)
{
	//is the message an battery message?
	if((rx.StdId & CAN_FILTER) == CAN_BATTERY)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
			{
				unsigned short temp16;
				memcpy(&(temp16), &(rx.Data[2]), sizeof(temp16));
				system_info.power_battery.v = (float)temp16/100.f;
				
				memcpy(&(temp16), &(rx.Data[4]), sizeof(temp16));
				system_info.power_battery.a = (float)temp16/100.f;
				
				memcpy(&(temp16), &(rx.Data[6]), sizeof(temp16));
				system_info.power_battery.rt = (float)temp16/100.f;
				
				system_info.power_battery.count ++;
				system_info.power_battery.flag = 1;
			
			}
				break;
			case CAN_MESSAGE_INDEX1:
		
				break;
			default:
				if(system_info.power_battery.flag == 1)
					system_info.power_battery.flag = 0;
				break;
		}	
	}
	else
	{
		system_info.power_battery.flag = 0;
	}
}


//ppm
void ppm_decode(CanRxMsg rx)
{
	extern void ppm_can_update(void);
	
	//is the message an air speed?
	if((rx.StdId & CAN_FILTER) == CAN_PPM)
	{
		//find out the rx message index number
		switch(rx.StdId & CAN_INDEX_FILTER)
		{
			case CAN_MESSAGE_INDEX0:
				ppm.can.ch1 = rx.Data[0]<<8 |  rx.Data[1];
				ppm.can.ch2 = rx.Data[2]<<8 |  rx.Data[3];
				ppm.can.ch3 = rx.Data[4]<<8 |  rx.Data[5];
				ppm.can.ch4 = rx.Data[6]<<8 |  rx.Data[7];
			
				ppm.can.count ++;
				ppm.can.flag = 1;
			
				ppm_can_update();
				
				break;
			case CAN_MESSAGE_INDEX1:
				ppm.can.ch5 = rx.Data[0]<<8 |  rx.Data[1];
				ppm.can.ch6 = rx.Data[2]<<8 |  rx.Data[3];
				ppm.can.ch7 = rx.Data[4]<<8 |  rx.Data[5];
				ppm.can.ch8 = rx.Data[6]<<8 |  rx.Data[7];
			
				ppm.can.count ++;
				ppm.can.flag = 1;

				ppm_can_update();//releas samphore for updating ppm input data			
		
				break;
			default:
				if(ppm.can.flag == 1)
					ppm.can.flag = 0;
				break;
		}	
	}
	else
	{
		ppm.flag = 0;
	}
}


//CHECK IF DATA Is good
void timer_expand_timeout(void *p)
{
	static unsigned int ult_count = 0;
	static unsigned int ppm_count = 0;
	
	//no new data update then this is wrong.
	if(ult_data.count == ult_count)
	{
		ult_data.flag = 0;
	}
	ult_count = ult_data.count;
	
	//no new ppm update then this is wrong.
	if(ppm.can.count == ppm_count)
	{
		ppm.can.flag = 0;
	}
	ppm_count = ppm.can.count;


}
	
//
void thread_expand(void)
{
	/* 创建超时定时器 */
	rt_timer_init(&timer,
					"expand", 				/* 定时器名字 */
					timer_expand_timeout, 				/* 超时时回调的处理函数*/
					RT_NULL,				/* 超时函数的入口参数*/
					RT_TICK_PER_SECOND, 					/* 定时长度，以OS Tick为单位*/
					RT_TIMER_FLAG_PERIODIC);/* 周期性定时器*/	
	/* 启动定时器*/
	rt_timer_start(&timer);	
	
	
	
	while(1)
	{
		rt_thread_delay(1);

	
	}


}


