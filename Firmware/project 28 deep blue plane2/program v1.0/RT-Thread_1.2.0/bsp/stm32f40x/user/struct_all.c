/*---------------------------------------------------------------*
*struct_all.c for AFC V0.2
*
*用于全局变量的保存，所有的关键的全局变量将在这里定义
*通常他们只有一个线程写入，一个或多个线程任意读取
*2011.10.13
*2013.1.23   change for STM32F4
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/

#include "struct_all.h"
#include "rtthread.h"


/* 系统相关信息 */
struct _system_info system_info;

/* 陀螺仪数据输出结构体 单位 度/s2 */
struct _gyro gyro;

/* 加速度计数据输出结构体 单位 mg */
struct _acc acc;

/* 磁阻传感器输出结构体 ? */
struct _mag mag;

/* 接收机PPM信号 */
struct _ppm ppm;

/* gps信息 */
struct _gps gps;

/* 角度最终计算值 */
struct _out_angle out_angle;

/* 磁阻传感器给陀螺仪的修正值 */
struct _gyro_offset gyro_offset;

/* 加速度计修正值 */
struct _acc_offset acc_offset;

/* 超声波测距输出数据 */
struct _ult_data ult_data;

/* 空气信息 */				 
struct _air_info air_info;

/* 空速 */
struct _air_speed air_speed;

// /* 控制算法 的PID参数*/
struct _pid pid;
//控制算法
struct _ctrl ctrl;

/* 飞行器 期望状态 包括 angle position */
struct _expectation expectation;

/* 飞机相关 */
struct _plane plane;

/* 控制器信息 状态 */
struct _control control;

/* navigation infomations */
struct _navigation navigation;

/* optical flow */
struct _opt_flow opt_flow;
