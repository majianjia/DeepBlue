/*
 * File      : thread_airpress.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-02    majianjia   the first version
 */


#ifndef __THREAD_AIRPRESS_H__
#define __THREAD_AIRPRESS_H__

#define MS5611_ADDR_7BIT		0x76 	//CSB SET HIGH	
#define MS5611_RD_ADDR			((MS5611_ADDR_7BIT << 1) | 0x01)
#define MS5611_WR_ADDR			((MS5611_ADDR_7BIT << 1) & 0xFE)

#define MS_COMMAND_RST			0x1E
#define MS_COMMAND_D1_256		0x40
#define MS_COMMAND_D1_512		0x42
#define MS_COMMAND_D1_1024 		0x44
#define MS_COMMAND_D1_2048		0x46
#define MS_COMMAND_D1_4096		0x48
#define MS_COMMAND_D2_256		0x50
#define MS_COMMAND_D2_512		0x52
#define MS_COMMAND_D2_1024		0x54
#define MS_COMMAND_D2_2048		0x56
#define MS_COMMAND_D2_4096		0x58

#define MS_COMMAND_ADC_READ		0x00
#define MS_COMMAND_PROM_READ	0xA0


#endif
