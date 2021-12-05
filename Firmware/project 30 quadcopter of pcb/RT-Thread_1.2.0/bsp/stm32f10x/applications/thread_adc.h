/*
 * File      : thread_airpress.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-02    majianjia   the first version
 */


#ifndef __THREAD_ADC_H__
#define __THREAD_ADC_H__

struct _adc
{
	float bat_volt;
	float bat_curr;
	float vcc5_curr;
	float bat_1s;
	float bat_2s;
	float bat_3s;
	float bat_4s;
	float bat_5s;
	
	float temp;
	float refint;

};
extern struct _adc adc;




#endif
