/*
 * File      : thread_expand.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-04    majianjia   the first version
 */
 
 
#ifndef __THREAD_EXPAND_H__
#define __THREAD_EXPAND_H__

#include "stm32f4xx.h"

extern void thread_expand(void);

extern void air_speed_decode(CanRxMsg rx);
extern void ultrasonic_ranging_decode(CanRxMsg rx);
//battery
extern void battery_decode(CanRxMsg rx);

//ppm
extern void ppm_decode(CanRxMsg rx);
extern void opt_floaw_decode(CanRxMsg rx);

#endif

