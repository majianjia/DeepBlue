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
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif


void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

#ifdef  RT_USING_FINSH
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif  /* RT_USING_FINSH */

}



/* thread_main */
ALIGN(RT_ALIGN_SIZE)
static char thread_main_stack[1024];
static struct rt_thread thread_main_handle;
void rt_entry_thread_main(void* parameter)
{
	extern int thread_main(void);
	thread_main();
}
//called in thread_can
void esc32_start_up(void)
{
	rt_thread_startup(&thread_main_handle);
}

/* thread_can */
ALIGN(RT_ALIGN_SIZE)
static char thread_can_stack[1024];
static struct rt_thread thread_can_handle;
void rt_entry_thread_can(void* parameter)
{
	extern int thread_can(void);
	thread_can();
}


int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;


#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);
	
	//------- can
    rt_thread_init(&thread_can_handle,
                   "can",
                   rt_entry_thread_can,
                   RT_NULL,
                   &thread_can_stack[0],
                   sizeof(thread_can_stack),3,1);
    rt_thread_startup(&thread_can_handle);
	
	//------- main
    rt_thread_init(&thread_main_handle,
                   "main",
                   rt_entry_thread_main,
                   RT_NULL,
                   &thread_main_stack[0],
                   sizeof(thread_main_stack),2,1);
   // rt_thread_startup(&thread_main_handle); //not start up until can bus have finished the initialization
	


    return 0;
}

/*@}*/
