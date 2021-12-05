/*
 * File      : cpuusage.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-1-2     majianjia    the first version
 */

#ifndef __CPUUSAGE_H__
#define __CPUUSAGE_H__

#include <rtthread.h>
#include <rthw.h>

extern void cpu_usage_init(void);
extern void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);

extern rt_uint8_t cpu_usage_major;
extern rt_uint8_t cpu_usage_minor;

#endif

