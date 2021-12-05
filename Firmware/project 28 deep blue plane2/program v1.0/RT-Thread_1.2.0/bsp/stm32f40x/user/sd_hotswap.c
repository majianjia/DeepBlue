/*
 * File      : sd_hotswap.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.11.6    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include "msd.h"
#include "dfs_posix.h"


#define SDIN_GPIO_PIN 		GPIO_Pin_5
#define SDIN_GPIO 			GPIOC
#define SDIN_GPIO_RCC   	RCC_AHB1Periph_GPIOC
#define SDIN				GPIO_ReadInputDataBit(SDIN_GPIO, SDIN_GPIO_PIN)

#define SDIN_INSET			RESET

#define SDPWR_GPIO_PIN 		GPIO_Pin_4
#define SDPWR_GPIO 			GPIOC
#define SDPWR_GPIO_RCC   	RCC_AHB1Periph_GPIOC
#define SDPWR_ON			GPIO_ResetBits(SDPWR_GPIO, SDPWR_GPIO_PIN)
#define SDPWR_OFF			GPIO_SetBits(SDPWR_GPIO, SDPWR_GPIO_PIN)


void rt_thread_sd_hotswap(void* parameter)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO Periph clock enable */
    RCC_AHB1PeriphClockCmd(SDIN_GPIO_RCC, ENABLE);

    /* Configure  */
    GPIO_InitStructure.GPIO_Pin = SDIN_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SDIN_GPIO, &GPIO_InitStructure);
	
	/* GPIO Periph clock enable */
    RCC_AHB1PeriphClockCmd(SDIN_GPIO_RCC, ENABLE);
	 /* Configure  */
    GPIO_InitStructure.GPIO_Pin = SDPWR_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SDPWR_GPIO, &GPIO_InitStructure);
	
	//turn off the power of sdcard
	//SDPWR_ON;
	
    while (1)
    {
		rt_thread_delay(RT_TICK_PER_SECOND/10);

#ifdef RT_USING_DFS_ELMFAT
		//no inset
		if(SDIN != SDIN_INSET)
		{
			//wait until the sdcard power is steady
			rt_thread_delay(RT_TICK_PER_SECOND/2);
			if(SDIN == SDIN_INSET)
				continue;
			
			//find sd0 to judge if sdcard have been init
			if(rt_device_find("sd0") != RT_NULL)
			{
				if(dfs_unmount("/sd/"))
				{
					rt_kprintf("unmount \"/sd\" failed!\n");
				}
				
				if(rt_device_unregister(rt_device_find("sd0")) == RT_EOK)
					rt_kprintf("sdcard device sd0 was unmounted. \n");
			}

		}
		
		//inserted
		else
		{
			//wait until the sdcard power is steady
			rt_thread_delay(RT_TICK_PER_SECOND/2);
			if(SDIN != SDIN_INSET)
				continue;			
			//find sd0 to judge if sdcard have been init
			if(rt_device_find("sd0") == RT_NULL)
			{
				//
				rt_kprintf("New sdcard is detected. \n");

				/* mount FAT file system on SD card */
				msd_init("sd0", "spi10");

				//init the sdcard device
				rt_device_init(rt_device_find("sd0"));

				if (dfs_mount("sd0", "/sd", "elm", 0, 0) == 0) 
					rt_kprintf("SDCard File System initialized!\n");
				else
					rt_kprintf("SDCard File System initialzation failed!\n");
			}

		}
#endif		

    }
}
