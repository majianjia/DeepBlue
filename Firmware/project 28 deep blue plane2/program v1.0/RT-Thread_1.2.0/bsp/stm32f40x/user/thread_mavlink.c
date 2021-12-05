/*
 * File      : thread_mavlink.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-05-02    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>

#include "struct_all.h"
#include "time_measure.h"

#include <stdint.h>
#pragma anon_unions
#include "mavlink.h"
#include "thread_mavlink.h"

//memory pool
#define MAVLINK_MEMPOOL_BLOCK_SIZE (256)
ALIGN(RT_ALIGN_SIZE)
static unsigned char mempool_tx[(MAVLINK_MEMPOOL_BLOCK_SIZE+4)*2];

static struct rt_mempool tx_mp;
unsigned char *tx_buf = RT_NULL;

static struct rt_timer timer;
static struct rt_semaphore sem;


//mavlink
mavlink_system_t mavlink_system;

// Define the system type, in this case an airplane
uint8_t system_type = MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight


void mavlink_init(void)
{
	mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
	mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
	mavlink_system.type = MAV_TYPE_QUADROTOR;    ///< This system is an quadcopter

}

void timeout(void *p)
{
	//system state
	if(ctrl.throttle.output.value > 200)
	{
		system_mode = MAV_MODE_AUTO_ARMED;
		system_state = MAV_STATE_ACTIVE;
	}
	else
	{
		system_mode = MAV_MODE_MANUAL_ARMED;
		system_state = MAV_STATE_STANDBY;
	}
	
	rt_sem_release(&sem);
}


void thread_mavlink(void *parameters)
{
	mavlink_message_t msg;
	unsigned short len;
	
	rt_device_t dev_uart = RT_NULL;
	
	mavlink_init();
	
	rt_mp_init(&tx_mp, "mavlink", &mempool_tx[0], sizeof(mempool_tx), MAVLINK_MEMPOOL_BLOCK_SIZE);
	rt_sem_init(&sem, "mavlink", 0, RT_IPC_FLAG_FIFO);
	rt_timer_init(&timer, "mavlink", timeout, RT_NULL, 100, RT_TIMER_FLAG_PERIODIC);
	rt_timer_start(&timer);
	
	//find uart device
	while(1)
	{
		dev_uart = rt_device_find("uart2");
		if(dev_uart != RT_NULL) 
		{
			rt_device_init(dev_uart);
			rt_device_open(dev_uart, RT_DEVICE_OFLAG_RDWR);
			break;
		}
			
		rt_thread_delay(20);
	}
	
	//main loop
	while(1)
	{
		//wait till a static period
		rt_sem_take(&sem, RT_WAITING_FOREVER);
		
		//send IMU
		{

			//get buffer
			tx_buf = rt_mp_alloc(&tx_mp, 1000);
			//if get a buffer
			if(tx_buf != RT_NULL)
			{
				mavlink_system.compid = MAV_COMP_ID_IMU;
				
				mavlink_msg_attitude_pack(mavlink_system.sysid,
										mavlink_system.compid, 
										&msg, 
										rt_tick_get(),
										(float)out_angle.rad_roll,
										(float)out_angle.rad_pitch,
										(float)out_angle.rad_yaw,
										(float)gyro.rad_roll,
										(float)gyro.rad_pitch,
										(float)gyro.rad_yaw);

				len = mavlink_msg_to_send_buffer(tx_buf, &msg);
				
				rt_device_write(dev_uart, 0, tx_buf, len);
				rt_mp_free(tx_buf);
			}
			//if get no buffer
			else
			{
			}					
		}
		
		//send GPS
		{
			static unsigned long count= 0;
			
			//2.5HZ update rate
			count++;
			//if(count%4 == 0)
			if(system_info.startup_second < 30) //just for first time
			{	
				//get buffer
				tx_buf = rt_mp_alloc(&tx_mp, 1000);
				//if get a buffer
				if(tx_buf != RT_NULL)
				{
					mavlink_system.compid = MAV_COMP_ID_GPS;
					
					mavlink_msg_gps_raw_int_pack(mavlink_system.sysid,
											mavlink_system.compid, 
											&msg,
											rt_tick_get(),
											0,
											(int)(gps.lat *1e7),
											(int)(gps.lon *1e7),
											(int)(gps.altitude *1000),
											0xffff, //eph unkonw 
											0xffff, //epv unkonw
											(int)(gps.speed*100),
											(int)gps.course,
											gps.stars);		

					len = mavlink_msg_to_send_buffer(tx_buf, &msg);
					
					rt_device_write(dev_uart, 0, tx_buf, len);
					rt_mp_free(tx_buf);
				}
				//if get no buffer
				else
				{
				}
			}			
		}
		
		//2.5HZ update rate  current position
		{
			static unsigned long count= 0;
			count++;
			if(count%4 == 2)
			{	
				//get buffer
				tx_buf = rt_mp_alloc(&tx_mp, 1000);
				//if get a buffer
				if(tx_buf != RT_NULL)
				{
					mavlink_system.compid = MAV_COMP_ID_GPS;
					
					mavlink_msg_global_position_int_pack(mavlink_system.sysid,
											mavlink_system.compid, 
											&msg,
											rt_tick_get(),
											(int)(gps.lat *1e7),
											(int)(gps.lon *1e7),
											(int)(gps.altitude *1000),
											(int)(gps.altitude *1000), //relative_alt
											0, 
											0, 
											0,
											(int)(out_angle.yaw*100));		

					len = mavlink_msg_to_send_buffer(tx_buf, &msg);
					
					rt_device_write(dev_uart, 0, tx_buf, len);
					rt_mp_free(tx_buf);
				}
				//if get no buffer
				else
				{
				}
			}			
		}
		
		//1HZ update rate  battery
		{
			static unsigned long count= 0;
			count++;
			if(count%10 == 5)
			{	
				//get buffer
				tx_buf = rt_mp_alloc(&tx_mp, 1000);
				//if get a buffer
				if(tx_buf != RT_NULL)
				{
					mavlink_system.compid = MAV_COMP_ID_ALL;
					
					mavlink_msg_battery_status_pack(mavlink_system.sysid,
											mavlink_system.compid, 
											&msg,
											0,
											(int)(system_info.power_battery.v *1000),
											-1,
											-1,
											-1, 
											-1, 
											-1, 
											(int)(system_info.power_battery.a *100),
											-1,
											-1,
											-1);		

					len = mavlink_msg_to_send_buffer(tx_buf, &msg);
					
					rt_device_write(dev_uart, 0, tx_buf, len);
					rt_mp_free(tx_buf);
				}
				//if get no buffer
				else
				{
				}
			}			
		}
	
		//send heart beat
		{
			static unsigned long second = 0;		
			
			//1 message per second
			if(second != system_info.startup_second)
			{	
				second = system_info.startup_second; //mark new second

				//get buffer
				tx_buf = rt_mp_alloc(&tx_mp, 1000);
				//if get a buffer
				if(tx_buf != RT_NULL)
				{
					mavlink_system.compid = MAV_COMP_ID_ALL;
					mavlink_msg_heartbeat_pack(mavlink_system.sysid, 
											mavlink_system.compid, 
											&msg, 
											system_type,
											autopilot_type, 
											system_mode, 
											custom_mode, 
											system_state);
			
					len = mavlink_msg_to_send_buffer(tx_buf, &msg);
					
					rt_device_write(dev_uart, 0, tx_buf, len);
					rt_mp_free(tx_buf);
				}
				//if get no buffer
				else
				{
				}
				
			}
		}
	
	
	}
}

