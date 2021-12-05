
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

#include <board.h>
#include <rtthread.h>

#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

#include "thread_main.h"

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t main_stack[ 512 ];
static struct rt_thread main_thread;
static void main_thread_entry(void* parameter)
{
	thread_main();
}

// void rt_init_thread_entry(void* parameter)
// {
// 	/* re-init device driver */
// 	rt_device_init_all();
// }

int rt_application_init()
{
//	rt_thread_t init_thread;
	extern void cpu_usage_init();
	
	//test the cpu usage
	cpu_usage_init();

    /* init main thread */
	rt_thread_init(&main_thread,
		"main",
		main_thread_entry, 
		RT_NULL,
		(rt_uint8_t*)&main_stack[0], 
		sizeof(main_stack), 10, 5);
    rt_thread_startup(&main_thread);
	



// #if (RT_THREAD_PRIORITY_MAX == 32)
// 	init_thread = rt_thread_create("init",
// 								rt_init_thread_entry, RT_NULL,
// 								2048, 8, 20);
// #else
// 	init_thread = rt_thread_create("init",
// 								rt_init_thread_entry, RT_NULL,
// 								2048, 80, 20);
// #endif
// 	rt_thread_startup(init_thread);

	return 0;
}

/*@}*/
