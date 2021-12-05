/*---------------------------------------------------------------*
*struct_all.c for AFC V0.2
*
*����ȫ�ֱ����ı��棬���еĹؼ���ȫ�ֱ����������ﶨ��
*ͨ������ֻ��һ���߳�д�룬һ�������߳������ȡ
*2011.10.13
*2013.1.23   change for STM32F4
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/

#include "struct_all.h"
#include "rtthread.h"


/* ϵͳ�����Ϣ */
struct _system_info system_info;

/* ��������������ṹ�� ��λ ��/s2 */
struct _gyro gyro;

/* ���ٶȼ���������ṹ�� ��λ mg */
struct _acc acc;

/* ���贫��������ṹ�� � */
struct _mag mag;

/* ���ջ�PPM�ź� */
struct _ppm ppm;

/* gps��Ϣ */
struct _gps gps;

/* �Ƕ����ռ���ֵ */
struct _out_angle out_angle;

/* ���贫�����������ǵ�����ֵ */
struct _gyro_offset gyro_offset;

/* ���ٶȼ�����ֵ */
struct _acc_offset acc_offset;

/* ���������������� */
struct _ult_data ult_data;

/* ������Ϣ */				 
struct _air_info air_info;

/* ���� */
struct _air_speed air_speed;

// /* �����㷨 ��PID����*/
struct _pid pid;
//�����㷨
struct _ctrl ctrl;

/* ������ ����״̬ ���� angle position */
struct _expectation expectation;

/* �ɻ���� */
struct _plane plane;

/* ��������Ϣ ״̬ */
struct _control control;

/* navigation infomations */
struct _navigation navigation;

/* optical flow */
struct _opt_flow opt_flow;
