/*
 * File      : thread_position.h
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-1-22    majianjia   the first version
 */
 
 #ifndef __THREAD_POSITION_H__
 #define __THREAD_POSITION_H__
 
 //��Ԫ�������õ��Ľṹ��   ��Ԫ��
struct _quaternion
{
	float w;
	float x;
	float y;
	float z;
};

 extern void position_release_start(void);
 extern void thread_position(void);
 
 
 #endif

