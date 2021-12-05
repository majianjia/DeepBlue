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

#include "config.h"
#include "fet.h"
#include "pwm.h"
#include "adc.h"
#include "run.h"
#include "serial.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_rcc.h"
#include <string.h>
#include <math.h>

float p[CONFIG_NUM_PARAMS];

const char *configParameterStrings[] = {
    "CONFIG_VERSION",
    "STARTUP_MODE",         //����ģʽ (�ŷ�ģʽ .....)
    "BAUD_RATE",            //���ڲ�����
    "PTERM",
    "ITERM",
    "FF1TERM",
    "FF2TERM",
    "CL1TERM",
    "CL2TERM",
    "CL3TERM",
    "CL4TERM",
    "CL5TERM",
    "SHUNT_RESISTANCE",
    "MIN_PERIOD",
    "MAX_PERIOD",
    "BLANKING_MICROS",
    "ADVANCE",
    "START_VOLTAGE",
    "GOOD_DETECTS_START",
    "BAD_DETECTS_DISARM",
    "MAX_CURRENT",
    "SWITCH_FREQ",
    "MOTOR_POLES",
    "PWM_MIN_PERIOD",
    "PWM_MAX_PERIOD",
    "PWM_MIN_VALUE",
    "PWM_LO_VALUE",
    "PWM_HI_VALUE",
    "PWM_MAX_VALUE",
    "PWM_MIN_START",
    "PWM_RPM_SCALE",
    "FET_BRAKING",          //�����ƶ�ģʽ
    "PNFAC",
    "INFAC",
    "THR1TERM",
    "THR2TERM",
    "START_ALIGN_TIME",
    "START_ALIGN_VOLTAGE",
    "START_STEPS_NUM",
    "START_STEPS_PERIOD",
    "START_STEPS_ACCEL",
    "PWM_LOWPASS",
    "RPM_MEAS_LP",
    "SERVO_DUTY",
    "SERVO_P",             //�ŷ�ģʽ�� PID���� P����
    "SERVO_D",             //�ŷ�ģʽ�� PID���� D����
    "SERVO_MAX_RATE",
    "SERVO_SCALE"
};

const char *configFormatStrings[] = {
    "%f",	    // CONFIG_VERSION
    "%.0f",	    // STARTUP_MODE
    "%.0f baud",    // BAUD_RATE
    "%.3f",	    // PTERM
    "%.5f",	    // ITERM
    "%+e",	    // FF1TERM
    "%+e",	    // FF2TERM
    "%+e",	    // CL1TERM
    "%+e",	    // CL2TERM
    "%+e",	    // CL3TERM
    "%+e",	    // CL4TERM
    "%+e",	    // CL5TERM
    "%.3f mohms",   // SHUNT_RESISTANCE
    "%.0f us",	    // MIN_PERIOD
    "%.0f us",	    // MAX_PERIOD
    "%.0f us",	    // BLANKING_MICROS
    "%.2f Degs",    // ADVANCE
    "%.2f Volts",   // START_VOLTAGE
    "%.0f",	    // GOOD_DETECTS_START
    "%.0f",	    // BAD_DETECTS_DISARM
    "%.1f Amps",    // MAX_CURRENT
    "%.1f KHz",	    // SWITCH_FREQ
    "%.0f",	    // MOTOR_POLES
    "%.0f us",	    // PWM_MIN_PERIOD
    "%.0f us",	    // PWM_MAX_PERIOD
    "%.0f us",	    // PWM_MIN_VALUE
    "%.0f us",	    // PWM_LO_VALUE
    "%.0f us",	    // PWM_HI_VALUE
    "%.0f us",	    // PWM_MAX_VALUE
    "%.0f us",	    // PWM_MIN_START
    "%.0f RPM",	    // PWM_RPM_SCALE
    "%.0f",	    // FET_BRAKING
    "%.2f",	    // PNFAC
    "%.2f",	    // INFAC
    "%+e",	    // THR1TERM
    "%+e",	    // THR2TERM
    "%.0f ms",	    // START_ALIGN_TIME
    "%.2f Volts",   // START_ALIGN_VOLTAGE
    "%.0f",	    // START_STEPS_NUM
    "%.0f us",	    // START_STEPS_PERIOD
    "%.0f us",	    // START_STEPS_ACCEL
    "%2.2f",	    // PWM_LOWPASS
    "%.3f",	    // RPM_MEAS_LP
    "%.1f %%",	    // SERVO_DUTY
    "%.3f",	    // SERVO_P
    "%.3f",	    // SERVO_D
    "%.1f deg/s",   // SERVO_MAX_RATE
    "%.1f deg"	    // SERVO_SCALE
};

void configInit(void) {
    float ver;

    configLoadDefault();

	//�������ַ��ȡ���汾��
    ver = *(float *)FLASH_WRITE_ADDR;

    if (isnan(ver))//�ж�ver  ���verΪ�Ǹ���(NAN��ֵΪ0xffffffff)������1�����򷵻�0 
		configWriteFlash();//����д��flash
    else if (ver >= p[CONFIG_VERSION])
		configReadFlash();
    else if (p[CONFIG_VERSION] > ver)
		configWriteFlash();
}

// recalculate constants with bounds checking
//�������߽�ļ��
static void configRecalcConst(void) {
    adcSetConstants();
    fetSetConstants();
    runSetConstants();
    pwmSetConstants();
    serialSetConstants();
}

//���ݴ�������index,���ò���ֵ
int configSetParamByID(int i, float value) {
    int ret = 0;

    if (i >= 0 && i < CONFIG_NUM_PARAMS) {
		p[i] = value;
		configRecalcConst();

		ret = 1;
    }

    return ret;
}

//�����ַ���parm,���ò���value
int configSetParam(char *param, float value) {
    int ret = 0;
    int i;

    for (i = 0; i < CONFIG_NUM_PARAMS; i++) {
		if (!strncasecmp(configParameterStrings[i], param, strlen(configParameterStrings[i]))) {
			configSetParamByID(i, value);
			ret = 1;
			break;
		}
    }

    return ret;
}

//���ݲ���param(�ַ���).��ȡ����������index
int configGetId(char *param) {
	int i;

	for (i = 0; i < CONFIG_NUM_PARAMS; i++)
		if (!strncasecmp(configParameterStrings[i], param, strlen(configParameterStrings[i])))//�ַ����Ƚ�,�Ƚϳɹ���������index
			return i;

	return -1;
}

#if 0//�������û�е���
float configGetParam(char *param) {
    int i;

    i = configGetId(param);

    if (i >= 0)
	return p[i];
    else
	//return __float32_nan;
	return NAN;//AXian �Ҳ���__float32_nan�������.����ô������
}
#endif

void configLoadDefault(void) {
    p[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;          //�����ñ�İ汾
    p[STARTUP_MODE] = DEFAULT_STARTUP_MODE;              //����ģʽ(�ŷ�ģʽ ....)
    p[BAUD_RATE] = DEFAULT_BAUD_RATE;                    //����Ĭ�ϲ�����

	p[PTERM] = DEFAULT_PTERM;
    p[ITERM] = DEFAULT_ITERM;
    p[FF1TERM] = DEFAULT_FF1TERM;
    p[FF2TERM] = DEFAULT_FF2TERM;
    p[CL1TERM] = DEFAULT_CL1TERM;
    p[CL2TERM] = DEFAULT_CL2TERM;
    p[CL3TERM] = DEFAULT_CL3TERM;
    p[CL4TERM] = DEFAULT_CL4TERM;
    p[CL5TERM] = DEFAULT_CL5TERM;

	p[SHUNT_RESISTANCE] = DEFAULT_SHUNT_RESISTANCE;
    p[MIN_PERIOD] = DEFAULT_MIN_PERIOD;
    p[MAX_PERIOD] = DEFAULT_MAX_PERIOD;
    p[BLANKING_MICROS] = DEFAULT_BLANKING_MICROS;
    p[ADVANCE] = DEFAULT_ADVANCE;
    p[START_VOLTAGE] = DEFAULT_START_VOLTAGE;
    p[GOOD_DETECTS_START] = DEFAULT_GOOD_DETECTS_START;
    p[BAD_DETECTS_DISARM] = DEFAULT_BAD_DETECTS_DISARM;
    p[MAX_CURRENT] = DEFAULT_MAX_CURRENT;
    p[SWITCH_FREQ] = DEFAULT_SWITCH_FREQ;
    p[MOTOR_POLES] = DEFAULT_MOTOR_POLES;

	p[PWM_MIN_PERIOD] = DEFAULT_PWM_MIN_PERIOD;           //PWM timer1 ch1 ����ģʽ,��С������
    p[PWM_MAX_PERIOD] = DEFAULT_PWM_MAX_PERIOD;           //PWM timer1 ch1 ����ģʽ,��������
    p[PWM_MIN_VALUE] = DEFAULT_PWM_MIN_VALUE;             //PWM timer1 ch2 ����ģʽ,��С������
    p[PWM_LO_VALUE] = DEFAULT_PWM_LO_VALUE;
    p[PWM_HI_VALUE] = DEFAULT_PWM_HI_VALUE;
    p[PWM_MAX_VALUE] = DEFAULT_PWM_MAX_VALUE;             //PWM timer1 ch2 ����ģʽ,��������\

    p[PWM_MIN_START] = DEFAULT_PWM_MIN_START;
    p[PWM_RPM_SCALE] = DEFAULT_PWM_RPM_SCALE;
    p[FET_BRAKING] = DEFAULT_FET_BRAKING;                 //=1 ���������ƶ�ģʽ, =0 �������ƶ�
    p[PNFAC] = DEFAULT_PNFAC;
    p[INFAC] = DEFAULT_INFAC;
    p[THR1TERM] = DEFAULT_THR1TERM;
    p[THR2TERM] = DEFAULT_THR2TERM;

	p[START_ALIGN_TIME] = DEFAULT_START_ALIGN_TIME;
    p[START_ALIGN_VOLTAGE] = DEFAULT_START_ALIGN_VOLTAGE;
    p[START_STEPS_NUM] = DEFAULT_START_STEPS_NUM;
    p[START_STEPS_PERIOD] = DEFAULT_START_STEPS_PERIOD;
    p[START_STEPS_ACCEL] = DEFAULT_START_STEPS_ACCEL;

	p[PWM_LOWPASS] = DEFAULT_PWM_LOWPASS;
    p[RPM_MEAS_LP] = DEFAULT_RPM_MEAS_LP;

	p[SERVO_DUTY] = DEFAULT_SERVO_DUTY;
    p[SERVO_P] = DEFAULT_SERVO_P;                         //�ŷ�ģʽ�� PID���� P����
    p[SERVO_D] = DEFAULT_SERVO_D;                         //�ŷ�ģʽ�� PID���� D����
    p[SERVO_MAX_RATE] = DEFAULT_SERVO_MAX_RATE;
    p[SERVO_SCALE] = DEFAULT_SERVO_SCALE;

    configRecalcConst();
}

//�������ļ�д��flash
int configWriteFlash(void) 
{
    uint16_t prevReloadVal;
    FLASH_Status FLASHStatus;
    uint32_t address;
    int ret = 0;

    prevReloadVal = runIWDGInit(999);

    // Startup HSI clock
    RCC_HSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != SET)
		runFeedIWDG();

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if ((FLASHStatus = FLASH_ErasePage(FLASH_WRITE_ADDR)) == FLASH_COMPLETE) 
	{
		address = 0;
		while (FLASHStatus == FLASH_COMPLETE && address < sizeof(p)) 
		{
			if ((FLASHStatus = FLASH_ProgramWord(FLASH_WRITE_ADDR + address, *(uint32_t *)((char *)p + address))) != FLASH_COMPLETE)
				break;

			address += 4;
		}

		ret = 1;
    }

    FLASH_Lock();

    runFeedIWDG();

    // Shutdown HSI clock
    RCC_HSICmd(DISABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == SET)
		;

    runIWDGInit(prevReloadVal);

    return ret;
}

//��ȡflash�ϵĲ������ڴ���
void configReadFlash(void) {
    memcpy(p, (char *)FLASH_WRITE_ADDR, sizeof(p));

    configRecalcConst();
}
