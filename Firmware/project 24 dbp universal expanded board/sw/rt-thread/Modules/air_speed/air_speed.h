/*
 * File      :air_speed.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.21   majianjia   the first version
 */


#ifndef __AIR_SPEED_H__
#define __AIR_SPEED_H__

extern void thread_as(void);

extern float mpxv_volt_raw;	//��������ѹ
extern float mpxv_out_raw ;		//�����ѹ
extern float mpxv_out ;		//�˲��������ѹ
extern float mpxv_volt;
extern float air_speed ;
extern float rt_air_speed;
extern float mpxv_p;
extern float p_diff;


#endif
