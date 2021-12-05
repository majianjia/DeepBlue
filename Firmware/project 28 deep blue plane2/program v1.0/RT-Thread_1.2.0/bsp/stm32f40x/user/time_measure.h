/*---------------------------------------------------------------*
*time_measure.h for AFC V0.2
*注意：
*	   此.h文件仅用于测量系统计算用时。
*占用硬件：timer5		
*开始测量：void start_measure_time(void)
*停止测量: unsigned int stop_measure_time(void)	   
*测量时间：0~65535us
*2011.10.16
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/
#ifndef __TIME_MEASURE_H__
#define __TIME_MEASURE_H__

//#include"stm32f10x.h"


extern struct _time_meas time_meas;

extern void start_measure_time(void);	   //开始一个新的测量，并更新测量值
extern unsigned int return_time(void);	   //记录一个时间点
extern void stop_measure_time(void);	   //完成此次测量，并更新测量值
	 

#endif 


