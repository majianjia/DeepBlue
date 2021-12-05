/*---------------------------------------------------------------*
*struct_all.c for AFC V0.2
*
*ÓÃÓÚÈ«¾Ö±äÁ¿µÄ±£´æ£¬ËùÓĞµÄ¹Ø¼üµÄÈ«¾Ö±äÁ¿½«ÔÚÕâÀï¶¨Òå
*Í¨³£ËûÃÇÖ»ÓĞÒ»¸öÏß³ÌĞ´Èë£¬Ò»¸ö»ò¶à¸öÏß³ÌÈÎÒâ¶ÁÈ¡
*2011.10.13
*2013.1.23   change for STM32F4
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/

#include "struct_all.h"
#include "rtthread.h"


/* ÏµÍ³Ïà¹ØĞÅÏ¢ */
struct _system_info system_info;

/* ÍÓÂİÒÇÊı¾İÊä³ö½á¹¹Ìå µ¥Î» ¶È/s2 */
struct _gyro gyro;

/* ¼ÓËÙ¶È¼ÆÊı¾İÊä³ö½á¹¹Ìå µ¥Î» mg */
struct _acc acc;

/* ´Å×è´«¸ĞÆ÷Êä³ö½á¹¹Ìå µ */
struct _mag mag;

/* ½ÓÊÕ»úPPMĞÅºÅ */
struct _ppm ppm;

/* gpsĞÅÏ¢ */
struct _gps gps;

/* ½Ç¶È×îÖÕ¼ÆËãÖµ */
struct _out_angle out_angle;

/* ´Å×è´«¸ĞÆ÷¸øÍÓÂİÒÇµÄĞŞÕıÖµ */
struct _gyro_offset gyro_offset;

/* ¼ÓËÙ¶È¼ÆĞŞÕıÖµ */
struct _acc_offset acc_offset;

/* ³¬Éù²¨²â¾àÊä³öÊı¾İ */
struct _ult_data ult_data;

/* ¿ÕÆøĞÅÏ¢ */				 
struct _air_info air_info;

/* ¿ÕËÙ */
struct _air_speed air_speed;

// /* ¿ØÖÆËã·¨ µÄPID²ÎÊı*/
struct _pid pid;
//¿ØÖÆËã·¨
struct _ctrl ctrl;

/* ·ÉĞĞÆ÷ ÆÚÍû×´Ì¬ °üÀ¨ angle position */
struct _expectation expectation;

/* ·É»úÏà¹Ø */
struct _plane plane;

/* ¿ØÖÆÆ÷ĞÅÏ¢ ×´Ì¬ */
struct _control control;

/* navigation infomations */
struct _navigation navigation;

/* optical flow */
struct _opt_flow opt_flow;
