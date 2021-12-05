/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f4xx.h"
#include <board.h>
#include <rtthread.h>

#include "dfs_posix.h"
#include "struct_all.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif

void rt_init_thread_entry(void* parameter)
{
	extern void rt_platform_init(void);
	extern void rt_components_board_init(void);
	extern void rt_components_init(void);
	extern void cpu_usage_init(void);
	//init platform
	rt_platform_init();
	
	rt_components_board_init();
	
	rt_components_init();	

	


	


//GUI
}

//FS
#ifdef RT_USING_DFS
#ifdef RT_USING_DFS_ROMFS
int romfs_init(void)
{
	extern const struct romfs_dirent romfs_root;
	
 	if (dfs_mount(RT_NULL, "/", "rom", 0, &romfs_root) == 0)
 	{
 		//rt_kprintf("ROM File System initialized!\n");
		return 0;
	}
 	else
	{
 		//rt_kprintf("ROM File System initialzation failed!\n");
		return -1;
	}	
}
INIT_ENV_EXPORT(romfs_init);
#endif
#endif



#define LED_RED_ON		GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define LED_RED_OFF		GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define LED_GREEN_ON	GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define LED_GREEN_OFF	GPIO_ResetBits(GPIOB, GPIO_Pin_10)
struct rt_timer timer_red;
struct rt_timer timer_gre;
static int red = 0;
static int red_on, red_off;
static int gre_on, gre_off;
static int green = 0;

void timeout_red(void *p)
{
	if(red)
	{
		LED_RED_OFF;
		red = 0;
		rt_timer_control(&timer_red, RT_TIMER_CTRL_SET_TIME, &red_off);
		
	}
	else
	{
		LED_RED_ON;
		red = 1;
		rt_timer_control(&timer_red, RT_TIMER_CTRL_SET_TIME, &red_on);
	
	}

}
void timeout_gre(void *p)
{
	if(green)
	{
		LED_GREEN_OFF;
		green = 0;
		rt_timer_control(&timer_gre, RT_TIMER_CTRL_SET_TIME, &gre_off);
	}
	else
	{
		LED_GREEN_ON;
		green = 1;
		rt_timer_control(&timer_gre, RT_TIMER_CTRL_SET_TIME, &gre_on);
	}

}

void led_red_set(int a)
{
	if(a)
		LED_RED_ON;
	else
		LED_RED_OFF;
}
void led_green_set(int a)
{
	if(a)
		LED_GREEN_ON;
	else
		LED_GREEN_OFF;
}

void led_red_flash(int time_on, int time_off, unsigned int flag)
{
	static unsigned int status = 0;
	red_on = time_on;
	red_off = time_off;
	
	if(flag == ENABLE)
	{
		if(status == 0)
		{
			rt_timer_control(&timer_red, RT_TIMER_CTRL_SET_TIME, &time_on);
			rt_timer_start(&timer_red);
			status = 1;
		}
	}
	else
	{
		status = 0;
		rt_timer_stop(&timer_red);
		led_red_set(0);
	}
		
}

void led_green_flash(int time_on, int time_off,  unsigned int flag)
{
	static unsigned int status = 0;
	gre_on = time_on;
	gre_off = time_off;
	
	if(flag == ENABLE)
	{		
		if(status == 0)
		{
			rt_timer_control(&timer_gre, RT_TIMER_CTRL_SET_TIME, &time_on);
			rt_timer_start(&timer_gre);
			status = 1;
		}

	}
	else
	{
		status = 0;
		rt_timer_stop(&timer_gre);
		led_green_set(0);
	}
		
}







ALIGN(RT_ALIGN_SIZE)
static char thread_led1_stack[1024];
struct rt_thread thread_led1;
static void rt_thread_entry_led1(void* parameter)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_11);

	rt_timer_init(&timer_red, "red", timeout_red, RT_NULL, 100, RT_TIMER_FLAG_PERIODIC);
	rt_timer_init(&timer_gre, "green", timeout_gre, RT_NULL, 100, RT_TIMER_FLAG_PERIODIC);

    while (1)
    {
        rt_thread_delay(RT_TICK_PER_SECOND/10);
		
		if(gps.flag)
		{
			led_red_set(1);
		}
		else
		{
			led_red_flash(RT_TICK_PER_SECOND/2,RT_TICK_PER_SECOND/2, ENABLE);
		}
		
    }
}

ALIGN(RT_ALIGN_SIZE)
static char thread_second_stack[256];
struct rt_thread thread_second;
static void rt_thread_entry_second(void* parameter)
{
    while (1)
    {
		system_info.startup_second++;
		
        rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

/* thread_gps */
ALIGN(RT_ALIGN_SIZE)
static char thread_gps_stack[1024];
static struct rt_thread thread_gps_handle;
void rt_entry_thread_gps(void* parameter)
{
	extern void thread_gps(void *parameter);
	thread_gps(parameter);
}

/* thread_sd_hotswap */
ALIGN(RT_ALIGN_SIZE)
static char thread_sd_stack[1024];
static struct rt_thread thread_sd_handle;
extern void thread_sd(void *parameter);
void rt_entry_thread_sd(void* parameter)
{
	void rt_thread_sd_hotswap(void* parameter);
	rt_thread_sd_hotswap(parameter);
}


/* thread_gyro */
ALIGN(RT_ALIGN_SIZE)
static char thread_gyro_stack[1024];
static struct rt_thread thread_gyro_handle;
void rt_entry_thread_gyro(void* parameter)
{
	extern void thread_gyro(void *parameter);
	thread_gyro(parameter);
}

/* thread_acc */
ALIGN(RT_ALIGN_SIZE)
static char thread_acc_stack[1024];
static struct rt_thread thread_acc_handle;
void rt_entry_thread_acc(void* parameter)
{
	extern void thread_acc(void *parameter);
	thread_acc(parameter);
}

/* thread_COMPASS */
ALIGN(RT_ALIGN_SIZE)
static char thread_compass_stack[1024];
static struct rt_thread thread_compass_handle;
void rt_entry_thread_compass(void* parameter)
{
	extern void thread_compass(void *parameter);
	thread_compass(parameter);
}

/* thread_airpress */
ALIGN(RT_ALIGN_SIZE)
static char thread_airpress_stack[1024];
static struct rt_thread thread_airpress_handle;
void rt_entry_thread_airpress(void* parameter)
{
	extern void thread_airpress(void *parameter);
	thread_airpress(parameter);
}

/* thread_heartbeat */
ALIGN(RT_ALIGN_SIZE)
static char thread_heartbeat_stack[1024];
static struct rt_thread thread_heartbeat_handle;
void rt_entry_thread_heartbeat(void* parameter)
{
	extern void thread_heartbeat(void *parameter);
	thread_heartbeat(parameter);
}


/* thread_calendar */
ALIGN(RT_ALIGN_SIZE)
static char thread_calendar_stack[1024];
static struct rt_thread thread_calendar_handle;
void rt_entry_thread_calendar(void* parameter)
{
	extern void thread_calendar(void *parameter);
	thread_calendar(parameter);
}
/* thread_log */
ALIGN(RT_ALIGN_SIZE)
static char thread_log_stack[1024];
static struct rt_thread thread_log_handle;
void rt_entry_thread_log(void* parameter)
{
	extern void thread_log(void *parameter);
	thread_log(parameter);
}

/* thread_position */
ALIGN(RT_ALIGN_SIZE)
static char thread_position_stack[2048];
static struct rt_thread thread_position_handle;
void rt_entry_thread_position(void* parameter)
{
	extern void thread_position(void *parameter);
	thread_position(parameter);
}


/* thread_controller */
ALIGN(RT_ALIGN_SIZE)
static char thread_controller_stack[2048];
static struct rt_thread thread_controller_handle;
void rt_entry_thread_controller(void* parameter)
{
	extern void thread_controller(void *parameter);
	thread_controller(parameter);
}

/* thread_can*/
ALIGN(RT_ALIGN_SIZE)
static char thread_canbus_stack[1024];
static struct rt_thread thread_canbus_handle;
void rt_entry_thread_canbus(void* parameter)
{
	extern void thread_canbus(void *parameter);
	thread_canbus(parameter);
}

/* thread_ppm*/
ALIGN(RT_ALIGN_SIZE)
static char thread_ppm_stack[1024];
static struct rt_thread thread_ppm_handle;
void rt_entry_thread_ppm(void* parameter)
{
	extern void thread_ppm(void *parameter);
	thread_ppm(parameter);
}

/* thread_navigator*/
ALIGN(RT_ALIGN_SIZE)
static char thread_navigator_stack[1024];
static struct rt_thread thread_navigator_handle;
void rt_entry_thread_navigator(void* parameter)
{
	extern void thread_navigator(void *parameter);
	thread_navigator(parameter);
}


/* thread_expand*/
ALIGN(RT_ALIGN_SIZE)
static char thread_expand_stack[1024];
static struct rt_thread thread_expand_handle;
void rt_entry_thread_expand(void* parameter)
{
	extern void thread_expand(void *parameter);
	thread_expand(parameter);
}

/* thread_mavlink*/
ALIGN(RT_ALIGN_SIZE)
static char thread_mavlink_stack[1024];
static struct rt_thread thread_mavlink_handle;
void rt_entry_thread_mavlink(void* parameter)
{
	extern void thread_mavlink(void *parameter);
	thread_mavlink(parameter);
}


int rt_application_init()
{
    rt_thread_t init_thread;

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 2, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 20, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    //------- init led1 thread
    rt_thread_init(&thread_led1,
                   "led1",
                   rt_thread_entry_led1,
                   RT_NULL,
                   &thread_led1_stack[0],
                   sizeof(thread_led1_stack),20,1);
    rt_thread_startup(&thread_led1);
	
	//------- init second thread
    rt_thread_init(&thread_second,
                   "second",
                   rt_thread_entry_second,
                   RT_NULL,
                   &thread_second_stack[0],
                   sizeof(thread_second_stack),3,5);
    rt_thread_startup(&thread_second);
	
    //------- GPS
    rt_thread_init(&thread_gps_handle,
                   "gps",
                   rt_entry_thread_gps,
                   RT_NULL,
                   &thread_gps_stack[0],
                   sizeof(thread_gps_stack),10,5);
    rt_thread_startup(&thread_gps_handle);

	//------- sd
    rt_thread_init(&thread_sd_handle,
                   "sd",
                   rt_entry_thread_sd,
                   RT_NULL,
                   &thread_sd_stack[0],
                   sizeof(thread_sd_stack),20,1);
    rt_thread_startup(&thread_sd_handle);
	
	//------- gyro
    rt_thread_init(&thread_gyro_handle,
                   "gyro",
                   rt_entry_thread_gyro,
                   RT_NULL,
                   &thread_gyro_stack[0],
                   sizeof(thread_gyro_stack),4,1);
    rt_thread_startup(&thread_gyro_handle);
	
	//------- acc
    rt_thread_init(&thread_acc_handle,
                   "acc",
                   rt_entry_thread_acc,
                   RT_NULL,
                   &thread_acc_stack[0],
                   sizeof(thread_acc_stack),5,1);
    rt_thread_startup(&thread_acc_handle);
	
	//------- compass
    rt_thread_init(&thread_compass_handle,
                   "compass",
                   rt_entry_thread_compass,
                   RT_NULL,
                   &thread_compass_stack[0],
                   sizeof(thread_compass_stack),5,1);
    rt_thread_startup(&thread_compass_handle);


	//------- airpress
    rt_thread_init(&thread_airpress_handle,
                   "airpress",
                   rt_entry_thread_airpress,
                   RT_NULL,
                   &thread_airpress_stack[0],
                   sizeof(thread_airpress_stack),5,1);
    rt_thread_startup(&thread_airpress_handle);
	
	
	//------- heartbeat
    rt_thread_init(&thread_heartbeat_handle,
                   "hbeat",
                   rt_entry_thread_heartbeat,
                   RT_NULL,
                   &thread_heartbeat_stack[0],
                   sizeof(thread_heartbeat_stack),8,1);
    rt_thread_startup(&thread_heartbeat_handle);
	
	//------- position
    rt_thread_init(&thread_position_handle,
                   "position",
                   rt_entry_thread_position,
                   RT_NULL,
                   &thread_position_stack[0],
                   sizeof(thread_position_stack),6,1);
    rt_thread_startup(&thread_position_handle);

	//------- calendar
    rt_thread_init(&thread_calendar_handle,
                   "calendar",
                   rt_entry_thread_calendar,
                   RT_NULL,
                   &thread_calendar_stack[0],
                   sizeof(thread_calendar_stack),8,1);
    rt_thread_startup(&thread_calendar_handle);
	
	
	//------- log
    rt_thread_init(&thread_log_handle,
                   "log",
                   rt_entry_thread_log,
                   RT_NULL,
                   &thread_log_stack[0],
                   sizeof(thread_log_stack),22,1);
    rt_thread_startup(&thread_log_handle);
	
	//------- controller
    rt_thread_init(&thread_controller_handle,
                   "ctrl",
                   rt_entry_thread_controller,
                   RT_NULL,
                   &thread_controller_stack[0],
                   sizeof(thread_controller_stack),7,1);
    rt_thread_startup(&thread_controller_handle);
	
	//------- canbus
    rt_thread_init(&thread_canbus_handle,
                   "canbus",
                   rt_entry_thread_canbus,
                   RT_NULL,
                   &thread_canbus_stack[0],
                   sizeof(thread_canbus_stack),11,1);
    rt_thread_startup(&thread_canbus_handle);
	
	//------- ppm
    rt_thread_init(&thread_ppm_handle,
                   "ppm",
                   rt_entry_thread_ppm,
                   RT_NULL,
                   &thread_ppm_stack[0],
                   sizeof(thread_ppm_stack),12,1);
    rt_thread_startup(&thread_ppm_handle);
	
	//------- navigator
    rt_thread_init(&thread_navigator_handle,
                   "navigator",
                   rt_entry_thread_navigator,
                   RT_NULL,
                   &thread_navigator_stack[0],
                   sizeof(thread_navigator_stack),13,1);
    rt_thread_startup(&thread_navigator_handle);

	//------- expand
    rt_thread_init(&thread_expand_handle,
                   "expand",
                   rt_entry_thread_expand,
                   RT_NULL,
                   &thread_expand_stack[0],
                   sizeof(thread_expand_stack),15,1);
    rt_thread_startup(&thread_expand_handle);
	
	//------- mavlink
    rt_thread_init(&thread_mavlink_handle,
                   "mavlink",
                   rt_entry_thread_mavlink,
                   RT_NULL,
                   &thread_mavlink_stack[0],
                   sizeof(thread_mavlink_stack),12,1);
    rt_thread_startup(&thread_mavlink_handle);


    return 0;
}

/*@}*/
