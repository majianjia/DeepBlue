/*
    This file is part of AutoQuad ESC32.

    AutoQuad ESC32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad ESC32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad ESC32.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _CONFIG_H
#define _CONFIG_H

#define DEFAULT_CONFIG_VERSION		1.503f
#define DEFAULT_STARTUP_MODE		0.0f
#define DEFAULT_BAUD_RATE		230400
//#define DEFAULT_BAUD_RATE		115200

#define DEFAULT_PTERM			0.25f
#define DEFAULT_PNFAC			10.0f
#define DEFAULT_ITERM			0.0006f
#define DEFAULT_INFAC			0.15f

#define DEFAULT_FF1TERM			0.0f
#define DEFAULT_FF2TERM			0.0f

#define DEFAULT_CL1TERM			0.0f
#define DEFAULT_CL2TERM			0.0f
#define DEFAULT_CL3TERM			0.0f
#define DEFAULT_CL4TERM			0.0f
#define DEFAULT_CL5TERM			0.0f

#define DEFAULT_THR1TERM		0.0f
#define DEFAULT_THR2TERM		1.0f

#define DEFAULT_SHUNT_RESISTANCE	0.5f	    // milli Ohms
#define DEFAULT_MIN_PERIOD		50.0f	    // us
#define DEFAULT_MAX_PERIOD		12000.0f    // us
#define DEFAULT_BLANKING_MICROS		30.0f	    // us
#define DEFAULT_ADVANCE			10.0f	    // electrical degrees
#define DEFAULT_START_VOLTAGE		1.1f	    // voltage used to start motor
#define DEFAULT_START_ALIGN_TIME	600	    // ms to align rotor in known position
#define DEFAULT_START_ALIGN_VOLTAGE	0.9f	    // max voltage during align (around 0.8 * START_VOLTAGE)
#define DEFAULT_START_STEPS_NUM		0.0f	    // steps without commutation
#define DEFAULT_START_STEPS_PERIOD	16000	    // us betweet steps
#define DEFAULT_START_STEPS_ACCEL	0.0f	    // us each following step will be shorter (acceleration)
#define DEFAULT_GOOD_DETECTS_START	75.0f	    // after which will go into RUNNING mode
#define DEFAULT_BAD_DETECTS_DISARM	48.0f	    // after which will go into DISARMED mode
#define DEFAULT_MAX_CURRENT		20.0f	    // amps
#define DEFAULT_SWITCH_FREQ		20.0f	    // output PWM frequency in KHz
#define DEFAULT_MOTOR_POLES		14.0f

#define DEFAULT_PWM_MIN_PERIOD		2200	    // minimum valid period
#define DEFAULT_PWM_MAX_PERIOD		25000	    // maximum valid period

#define DEFAULT_PWM_MIN_VALUE		750	    // minimum to consider pulse a valid signal
#define DEFAULT_PWM_LO_VALUE		1000	    // lowest running value
#define DEFAULT_PWM_HI_VALUE		1950	    // highest running value
#define DEFAULT_PWM_MAX_VALUE		2250	    // maximum to consider pulse a valid signal
#define DEFAULT_PWM_MIN_START		1100	    // minimum value required to start

#define DEFAULT_PWM_LOWPASS		0.0f	    // lowpass on PWM input values (0 = none, 10 = heavy, no upper limit)
#define DEFAULT_RPM_MEAS_LP		0.5f	    // lowpass measured RPM values for closed loop control (0 = none, 0.99 = max, >=1 not allowed)

#define DEFAULT_PWM_RPM_SCALE		6500	    // RPM equivalent of maximum PWM IN in CLOSED_LOOP mode

#define DEFAULT_FET_BRAKING		0

#define DEFAULT_SERVO_DUTY		16.0f	    // %
#define DEFAULT_SERVO_P			0.05f
#define DEFAULT_SERVO_D			0.0f
#define DEFAULT_SERVO_MAX_RATE		1000.0f	    // deg/s
#define DEFAULT_SERVO_SCALE		360.0f	    // deg

#define FLASH_PAGE_SIZE			((uint16_t)0x400)
#define FLASH_WRITE_ADDR		(0x08000000 + (uint32_t)FLASH_PAGE_SIZE * 63)    // use the last KB for storage 配置参数保存地址

enum configParameters {
    CONFIG_VERSION = 0,
    STARTUP_MODE,/* 0==正常的开环模式 1==闭环RPM模式 2==推力闭环模式 */
    BAUD_RATE,   /* 默认的波特率 */

	PTERM,       /* 转速PI控制,P项 */
    ITERM,       /* 转速PI控制,I项 */

	FF1TERM,     /*  */
    FF2TERM,     /**/

	CL1TERM,     /**/
    CL2TERM,     /**/
    CL3TERM,     /**/
    CL4TERM,     /**/
    CL5TERM,     /**/
    SHUNT_RESISTANCE, /* 蛇形分流电阻值 */
    MIN_PERIOD,    /* 最小的换向周期 */
    MAX_PERIOD,    /* 最大的换向周期 */
    BLANKING_MICROS,/* 反电动势 */
    ADVANCE,       /* 提前多少角度测量换向 */
    START_VOLTAGE, /* 启动电压值,启动过程中.根据此值计算出启动的PWM周期 */
    GOOD_DETECTS_START, /* 电机在启动状态时,要有多少个好的检测,才认为电机是正常的 */
    BAD_DETECTS_DISARM, /* 运行过程中.检测到错误的状态后.会停止电机运行 */
    MAX_CURRENT,   /* 最大的电流 */
    SWITCH_FREQ,   /* PWM周期 单位是KHz 默认20KHz */
    MOTOR_POLES,   /* 电机使用多少对磁 */

	PWM_MIN_PERIOD,/* PWM输入最小的周期 在最小周期到最大周期内输入的频率是有效的 */
    PWM_MAX_PERIOD,/* PWM输入最大的周期 */
    PWM_MIN_VALUE, /* PWM输入最小的高电平时间(脉宽长度) */
    PWM_LO_VALUE,
    PWM_HI_VALUE,
    PWM_MAX_VALUE, /* PWM输如最大的高电平时间(脉宽长度) */
    PWM_MIN_START,
    PWM_RPM_SCALE,

	FET_BRAKING,   /* 制动模式 0=关闭 1=开启 */
    PNFAC,
    INFAC,
    THR1TERM,
    THR2TERM,
    START_ALIGN_TIME,
    START_ALIGN_VOLTAGE,
    START_STEPS_NUM,
    START_STEPS_PERIOD,
    START_STEPS_ACCEL,
    PWM_LOWPASS,
    RPM_MEAS_LP,
    SERVO_DUTY,
    SERVO_P,
    SERVO_D,
    SERVO_MAX_RATE,
    SERVO_SCALE,
    CONFIG_NUM_PARAMS
};

extern float p[CONFIG_NUM_PARAMS];
extern const char *configParameterStrings[];
extern const char *configFormatStrings[];

extern void configInit(void);
extern int configSetParam(char *param, float value);
extern int configSetParamByID(int i, float value);
extern int configGetId(char *param);
extern float configGetParam(char *param);
extern void configLoadDefault(void);
extern void configReadFlash(void);
extern int configWriteFlash(void);

#endif
