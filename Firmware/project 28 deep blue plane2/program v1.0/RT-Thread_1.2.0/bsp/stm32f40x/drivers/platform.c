/*
 * File      : platform.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.3.16    majianjia   the first version
 */

#include "stm32f4xx.h"
#include "stm32f20x_40x_spi.h"
#include "platform.h"


#define USING_SPI1

/*** SPI1 BUS and device
SPI1_MOSI: PA7
SPI1_MISO: PA6
SPI1_SCK : PA5
SPI1_NSS : PA4
*/
void rt_hw_spi1_init(void)
{
	/* register SPI bus */
	static struct stm32_spi_bus stm32_spi;
	
	/* SPI1 configure */
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Enable SPI1 Periph clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //for emi
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; //NO PULL, pull up will let sdcard unsteady

		/* Configure SPI1 pins */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	} /* SPI1 configuration */

	/* register SPI1 */
	stm32_spi_register(SPI1, &stm32_spi, "spi1");

	/* attach spi10 spi11 */
	{
		static struct rt_spi_device rt_spi_device_10;
		static struct stm32_spi_cs  stm32_spi_cs_10;
		
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		stm32_spi_cs_10.GPIOx = GPIOA;
		stm32_spi_cs_10.GPIO_Pin = GPIO_Pin_4;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_SetBits(GPIOA, GPIO_Pin_1);

		rt_spi_bus_attach_device(&rt_spi_device_10, "spi10", "spi1", (void*)&stm32_spi_cs_10);
	} /* attach spi10 */
	
}


/*** SPI3 BUS and device
SPI1_MOSI: PC10
SPI1_MISO: PC11
SPI1_SCK : PC12
SPI1_NSS : PE3,4,5,6
*/
void rt_hw_spi3_init(void)
{
	/* register SPI bus */
	static struct stm32_spi_bus stm32_spi;
	
	/* SPI1 configure */
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		/* Enable SPI Periph clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //for emi
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; //NO PULL, pull up will let sdcard unsteady

		/* Configure SPI1 pins */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	} /* SPI3 configuration */

	/* register SPI3 */
	stm32_spi_register(SPI3, &stm32_spi, "spi3");
	
	/* attach spi30 spi31 spi32 spi31*/
	{
		static struct rt_spi_device rt_spi_device_30;
		static struct stm32_spi_cs  stm32_spi_cs_30;
		static struct rt_spi_device rt_spi_device_31;
		static struct stm32_spi_cs  stm32_spi_cs_31;
		static struct rt_spi_device rt_spi_device_32;
		static struct stm32_spi_cs  stm32_spi_cs_32;
		static struct rt_spi_device rt_spi_device_33;
		static struct stm32_spi_cs  stm32_spi_cs_33;
		
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

		stm32_spi_cs_30.GPIOx = GPIOE;
		stm32_spi_cs_30.GPIO_Pin = GPIO_Pin_3;
		stm32_spi_cs_31.GPIOx = GPIOE;
		stm32_spi_cs_31.GPIO_Pin = GPIO_Pin_4;
		stm32_spi_cs_32.GPIOx = GPIOE;
		stm32_spi_cs_32.GPIO_Pin = GPIO_Pin_5;
		stm32_spi_cs_33.GPIOx = GPIOE;
		stm32_spi_cs_33.GPIO_Pin = GPIO_Pin_6;

		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
		GPIO_Init(GPIOE, &GPIO_InitStructure);

		GPIO_SetBits(GPIOE, GPIO_Pin_3);
		GPIO_SetBits(GPIOE, GPIO_Pin_4);
		GPIO_SetBits(GPIOE, GPIO_Pin_5);
		GPIO_SetBits(GPIOE, GPIO_Pin_6);

		rt_spi_bus_attach_device(&rt_spi_device_30, "spi30", "spi3", (void*)&stm32_spi_cs_30);
		rt_spi_bus_attach_device(&rt_spi_device_31, "spi31", "spi3", (void*)&stm32_spi_cs_31);
		rt_spi_bus_attach_device(&rt_spi_device_32, "spi32", "spi3", (void*)&stm32_spi_cs_32);
		rt_spi_bus_attach_device(&rt_spi_device_33, "spi33", "spi3", (void*)&stm32_spi_cs_33);
		
		
		/* config spi */
		{
			struct rt_spi_configuration cfg;
			cfg.data_width = 8;
			cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
			
			
			//SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M, ...
			cfg.max_hz = 1000*11000; /* 11000kbit/s */ 
			rt_spi_configure(&rt_spi_device_30, &cfg);
			
			cfg.max_hz = 1000*6000; /* 5000kbit/s */
			rt_spi_configure(&rt_spi_device_31, &cfg);
			
			cfg.max_hz = 1000*6000; /* 8000kbit/s */
			rt_spi_configure(&rt_spi_device_32, &cfg);
			
			cfg.max_hz = 1000*11000; /* 10000kbit/s */
			rt_spi_configure(&rt_spi_device_33, &cfg);
		} /* config spi */
		
		
	} /* attach spi10 */
	
}




void rt_platform_init(void)
{
    rt_hw_spi1_init();
	rt_hw_spi3_init();

}
