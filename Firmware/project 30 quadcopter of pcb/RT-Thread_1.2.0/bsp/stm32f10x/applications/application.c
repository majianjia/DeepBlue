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

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t adc_stack[ 1024 ];
static struct rt_thread adc_thread;
static void adc_thread_entry(void* parameter)
{
	extern void thread_adc(void);
	thread_adc();
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t main_stack[ 1024 ];
static struct rt_thread main_thread;
static void main_thread_entry(void* parameter)
{
	extern void thread_main(void);
	thread_main();
}


//ALIGN(RT_ALIGN_SIZE)
//static rt_uint8_t led_stack[ 512 ];
//static struct rt_thread led_thread;
//rt_uint8_t cpu_major, cpu_minor;
//static void led_thread_entry(void* parameter)
//{
//	extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
//    GPIO_InitTypeDef GPIO_InitStructure;

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
//	  
//	/* Disable the Serial Wire Jtag Debug Port SWJ-DP */
//    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable ,ENABLE);

//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	GPIO_ResetBits(GPIOB, GPIO_Pin_4);


//	{
////		int count = 50;
////		while (count--)
////		{
////			rt_thread_delay(1);
////			GPIO_SetBits(GPIOB, GPIO_Pin_4);
////			rt_thread_delay(1);
////			GPIO_ResetBits(GPIOB, GPIO_Pin_4);

////		}
//		
//		while(1)
//		{
//			cpu_usage_get(&cpu_major, &cpu_minor);
//			rt_thread_delay(100);
//		}
//	}
//}

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

#ifdef  RT_USING_FINSH
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif  /* RT_USING_FINSH */

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    /* mount sd card fat partition 1 as root directory */
    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("File System initialized!\n");
    }
    else
        rt_kprintf("File System initialzation failed!\n");
#endif  /* RT_USING_DFS */

#ifdef RT_USING_RTGUI
    {
        extern void rt_hw_lcd_init();
        extern void rtgui_touch_hw_init(void);

        rt_device_t lcd;

        /* init lcd */
        rt_hw_lcd_init();

        /* init touch panel */
        rtgui_touch_hw_init();

        /* re-init device driver */
        rt_device_init_all();

        /* find lcd device */
        lcd = rt_device_find("lcd");

        /* set lcd device as rtgui graphic driver */
        rtgui_graphic_set_device(lcd);

#ifndef RT_USING_COMPONENTS_INIT
        /* init rtgui system server */
        rtgui_system_server_init();
#endif

        calibration_set_restore(cali_setup);
        calibration_set_after(cali_store);
        calibration_init();
    }
#endif /* #ifdef RT_USING_RTGUI */
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


    /* init led thread */
    result = rt_thread_init(&adc_thread,
                            "adc",
                            adc_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&adc_stack[0],
                            sizeof(adc_stack),
                            10,5);
     rt_thread_startup(&adc_thread);			

    /* init main thread */
    result = rt_thread_init(&main_thread,
                            "main",
                            main_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&main_stack[0],
                            sizeof(main_stack),
                            2,1);
     rt_thread_startup(&main_thread);								


    return 0;
}

/*@}*/
