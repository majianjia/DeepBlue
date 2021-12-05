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

#define DEFAULT_CONFIG_VERSION		2.01f
#define DEFAULT_STARTUP_MODE		0.0f
#define DEFAULT_BAUD_RATE		230400
#define DEFAULT_ESC_ID			0

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
#define DEFAULT_DIRECTION		1.0f	    // 1 == forward, -1 == reverse

#define FLASH_PAGE_SIZE			((uint16_t)0x400)
#define FLASH_WRITE_ADDR		(0x08000000 + (uint32_t)FLASH_PAGE_SIZE * 63)    // use the last KB for storage

enum configParameters {
    CONFIG_VERSION = 0,
    STARTUP_MODE,
    BAUD_RATE,
    PTERM,
    ITERM,
    FF1TERM,
    FF2TERM,
    CL1TERM,
    CL2TERM,
    CL3TERM,
    CL4TERM,
    CL5TERM,
    SHUNT_RESISTANCE,
    MIN_PERIOD,
    MAX_PERIOD,
    BLANKING_MICROS,
    ADVANCE,
    START_VOLTAGE,
    GOOD_DETECTS_START,
    BAD_DETECTS_DISARM,
    MAX_CURRENT,
    SWITCH_FREQ,
    MOTOR_POLES,
    PWM_MIN_PERIOD,
    PWM_MAX_PERIOD,
    PWM_MIN_VALUE,
    PWM_LO_VALUE,
    PWM_HI_VALUE,
    PWM_MAX_VALUE,
    PWM_MIN_START,
    PWM_RPM_SCALE,
    FET_BRAKING,
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
    ESC_ID,
    DIRECTION,
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
