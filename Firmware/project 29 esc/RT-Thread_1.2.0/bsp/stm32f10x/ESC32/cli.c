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

#include "cli.h"
#include "getbuildnum.h"
#include "main.h"
#include "serial.h"
#include "run.h"
#include "fet.h"
#include "adc.h"
#include "pwm.h"
#include "config.h"
#include "rcc.h"
#include "timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

char version[16];       //当前软件版本
static char cliBuf[32]; //串口接收到的数据保存在此数组里面,根据cliBufIndex来索引
static int cliBufIndex; //当前串口缓存的数组index
static char tempBuf[64];//临时寄存器
static int cliTelemetry;//自动打印数据模式,在systick中断中会调用cliCheck函数.然后可以自动打印当前的信息(串口方式)

// this table must be sorted by command name
//这个数组必须按照从小到大排列.因为后面用二分法来搜索
static const cliCommand_t cliCommandTable[] = {
    {"arm", "", cliFuncArm},                              //切换为手动运行
    {"beep", "<frequency> <duration>", cliFuncBeep},      //让电机beep
    {"binary", "", cliFuncBinary},                        //串口切换为 binary控制模式.
    {"bootloader", "", cliFuncBoot},                      //重启esc32

	{"config", "[READ | WRITE | DEFAULT]", cliFuncConfig},//参数表的 读取 写入 恢复模式操作

	{"disarm", "", cliFuncDisarm},                        //停止手动运行模式
    {"duty", "<percent>", cliFuncDuty},                   //设置占空比(0~100%,值越高转速越快)

	{"help", "", cliFuncHelp},                            //显示支持的功能命令的帮助信息
    {"input", "[PWM | UART | I2C | CAN]", cliFuncInput},  //设置输入控制模式
    {"mode", "[OPEN_LOOP | RPM | THRUST | SERVO]", cliFuncMode}, //设置运行模式
    
	{"pos", "<degrees>", cliFuncPos},                   //设置电机要转到什么角度(只在伺服控制模式下使用),传递进来的参数是电机的目标角度
    {"pwm", "<microseconds>", cliFuncPwm},              //设置电机的PWM
	{"rpm", "<target>", cliFuncRpm},                    //设置目标速度

	{"set", "LIST | [<PARAMETER> <value>]", cliFuncSet},//设置参数表
    {"start", "", cliFuncStart},                        //电机开始运行
    {"status", "", cliFuncStatus},                      //显示状态
    {"stop", "", cliFuncStop},                          //电机停止运行

	{"telemetry", "<Hz>", cliFuncTelemetry},            //自动显示电调状态(和控制无关)
    {"version", "", cliFuncVer}                         //显示版本
};

#define CLI_N_CMDS (sizeof cliCommandTable / sizeof cliCommandTable[0])

static const char *cliInputModes[] = {
    "PWM",
    "UART",
    "I2C",
    "CAN",
    "OW"
};

static const char *cliStates[] = {
    "DISARMED",
    "STOPPED",
    "PRE-START",
    "STARTING",
    "RUNNING"
};

static const char *cliRunModes[] = {
    "OPEN_LOOP",
    "RPM",
    "THRUST",
    "SERVO"
};

static const char cliHome[] = {0x1b, 0x5b, 0x48, 0x00};
static const char cliClear[] = {0x1b, 0x5b, 0x32, 0x4a, 0x00};
static const char cliClearEOL[] = {0x1b, 0x5b, 0x4b, 0x00};
static const char cliClearEOS[] = {0x1b, 0x5b, 0x4a, 0x00};
static const char *stopError = "ESC must be stopped first\r\n";
static const char *runError = "ESC not running\r\n";

//命令提示.
void cliUsage(cliCommand_t *cmd) {
    serialPrint("usage: ");
    serialPrint(cmd->name);
    serialWrite(' ');
    serialPrint(cmd->params);
    serialPrint("\r\n");
}
//设置控制模式(串口 can iic pwm ow)
static void cliFuncChangeInput(uint8_t input) {
	if (inputMode != input) {
		inputMode = input;
		sprintf(tempBuf, "Input mode set to %s\r\n", cliInputModes[input]);
		serialPrint(tempBuf);
	}
}
//电机手动运行
static void cliFuncArm(void *cmd, char *cmdLine) {
	if (state > ESC_STATE_DISARMED) {
		serialPrint("ESC already armed\r\n");
	}
	else {
		if (runMode != SERVO_MODE)
			cliFuncChangeInput(ESC_INPUT_UART);
		runArm();
		serialPrint("ESC armed\r\n");
	}
}
//让电机beep
static void cliFuncBeep(void *cmd, char *cmdLine) {
	uint16_t freq, dur;

	if (state > ESC_STATE_STOPPED) {
		serialPrint(stopError);
	}
	else {
		if (sscanf(cmdLine, "%hu %hu", &freq, &dur) != 2) {
			cliUsage((cliCommand_t *)cmd);
		}
		else if (freq < 10 || freq > 5000) {
			serialPrint("frequency out of range: 10 => 5000\r\n");
		}
		else if (dur < 1 || dur > 1000) {
			serialPrint("duration out of range: 1 => 1000\r\n");
		}
		else {
			fetBeep(freq, dur);
		}
	}
}

static void cliFuncBinary(void *cmd, char *cmdLine) {
    if (state > ESC_STATE_STOPPED) {
		serialPrint(stopError);
    }
    else {
		serialPrint("Entering binary command mode...\r\n");
		cliTelemetry = 0;
		commandMode = BINARY_MODE;
    }
}

static void cliFuncBoot(void *cmd, char *cmdLine) {
    if (state != ESC_STATE_DISARMED) {
		serialPrint("ESC armed, disarm first\r\n");
    }
    else {
		serialPrint("Rebooting in boot loader mode...\r\n");
		timerDelay(0xffff);

		rccReset();
    }
}

static void cliFuncConfig(void *cmd, char *cmdLine) {
	char param[8];

	if (state > ESC_STATE_STOPPED) {
		serialPrint(stopError);
	}
	else if (sscanf(cmdLine, "%8s", param) != 1) {
		cliUsage((cliCommand_t *)cmd);
	}
	else if (!strcasecmp(param, "default")) {
		configLoadDefault();
		serialPrint("CONFIG: defaults loaded\r\n");
	}
	else if (!strcasecmp(param, "read")) {
		configReadFlash();
		serialPrint("CONFIG: read flash\r\n");
	}
	else if (!strcasecmp(param, "write")) 
	{
		if (configWriteFlash()) {
			serialPrint("CONFIG: wrote flash\r\n");
		}
		else {
			serialPrint("CONFIG: write flash failed!\r\n");
		}
	}
	else {
		cliUsage((cliCommand_t *)cmd);
	}
}

static void cliFuncDisarm(void *cmd, char *cmdLine) {
    runDisarm(REASON_CLI);
    cliFuncChangeInput(ESC_INPUT_UART);
    serialPrint("ESC disarmed\r\n");
}

static void cliFuncDuty(void *cmd, char *cmdLine) {
	float duty;

	if (state < ESC_STATE_RUNNING) {
		serialPrint(runError);
	}
	else 
	{
		if (sscanf(cmdLine, "%f", &duty) != 1) {
			cliUsage((cliCommand_t *)cmd);
		}
		else if (!runDuty(duty)) 
		{
			//duty 不在0~100范围内
			serialPrint("duty out of range: 0 => 100\r\n");
		}
		else {
			sprintf(tempBuf, "Fet duty set to %.2f%%\r\n", (float)fetDutyCycle/fetPeriod*100.0f);//占空比(0~100) / 最大周期 * 100
			serialPrint(tempBuf);
		}
	}
}

static void cliFuncHelp(void *cmd, char *cmdLine) {
    int i;

    serialPrint("Available commands:\r\n\n");

    for (i = 0; i < CLI_N_CMDS; i++) {
		serialPrint(cliCommandTable[i].name);
		serialWrite(' ');
		serialPrint(cliCommandTable[i].params);
		serialPrint("\r\n");
    }
}
//更改输入模式(pwm iic can uart)
static void cliFuncInput(void *cmd, char *cmdLine) {
	char mode[sizeof cliInputModes[0]];
	int i;

	if (sscanf(cmdLine, "%7s", mode) != 1) {
		cliUsage((cliCommand_t *)cmd);
	}
	else 
	{
		for (i = 0; i < (sizeof cliInputModes / sizeof cliInputModes[0]); i++) {
			if (!strncasecmp(cliInputModes[i], mode, 3))
				break;
		}

		if (i < (sizeof cliInputModes / sizeof cliInputModes[0])) {
			cliFuncDisarm(cmd, cmdLine);
			cliFuncChangeInput(i);//更改输入模式
		}
		else
			cliUsage((cliCommand_t *)cmd);//没有找到支持的模式
	}
}

static void cliFuncMode(void *cmd, char *cmdLine) {
	char mode[sizeof cliRunModes[0]];
	int i;

	if (sscanf(cmdLine, "%10s", mode) != 1) {
		cliUsage((cliCommand_t *)cmd);
	}
	else {
		for (i = 0; i < (sizeof cliRunModes / sizeof cliRunModes[0]); i++) {
			if (!strncasecmp(cliRunModes[i], mode, strlen(cliRunModes[i])))
				break;
		}

		if (i < (sizeof cliRunModes / sizeof cliRunModes[0])) {
			cliFuncDisarm(cmd, cmdLine);
			runMode = i;
			sprintf(tempBuf, "Run mode set to %s\r\n", cliRunModes[i]);
			serialPrint(tempBuf);
		}
		else
			cliUsage((cliCommand_t *)cmd);
	}
}

static void cliFuncPos(void *cmd, char *cmdLine) {
	float angle;

	if (state < ESC_STATE_RUNNING) {
		serialPrint(runError);
	}
	else if (runMode != SERVO_MODE) {
		serialPrint("Command only valid in servo mode\r\n");
	}
	else {
		if (sscanf(cmdLine, "%f", &angle) != 1) {
			cliUsage((cliCommand_t *)cmd);
		}
		else {
			fetSetAngle(angle);
			sprintf(tempBuf, "Position set to %.1f\r\n", angle);
			serialPrint(tempBuf);
		}
	}
}

static void cliFuncPwm(void *cmd, char *cmdLine) {
	uint16_t pwm;

	if (state < ESC_STATE_RUNNING) {
		serialPrint(runError);
	}
	else {
		if (sscanf(cmdLine, "%hu", &pwm) != 1) {
			cliUsage((cliCommand_t *)cmd);
		}
		else if (pwm < pwmLoValue || pwm > pwmHiValue) {
			sprintf(tempBuf, "PWM out of range: %d => %d\r\n", pwmLoValue, pwmHiValue);
			serialPrint(tempBuf);
		}
		else {
			if (runMode != SERVO_MODE)
				runMode = OPEN_LOOP;
			runNewInput(pwm);
			sprintf(tempBuf, "PWM set to %d\r\n", pwm);
			serialPrint(tempBuf);
		}
	}
}
//设置目标转速
static void cliFuncRpm(void *cmd, char *cmdLine) {
	float target;

	if (state < ESC_STATE_RUNNING) {
		serialPrint(runError);
	}
	else 
	{
		if (sscanf(cmdLine, "%f", &target) != 1) {
			cliUsage((cliCommand_t *)cmd);
		}
		else if (p[FF1TERM] == 0.0f) {
			serialPrint("Calibration parameters required\r\n");
		}
		else if (target < 100.0f || target > 10000.0f) {
			serialPrint("RPM out of range: 100 => 10000\r\n");
		}
		else 
		{
			if (runMode != CLOSED_LOOP_RPM) {
				runRpmPIDReset();
				runMode = CLOSED_LOOP_RPM;
			}
			targetRpm = target;//设置目标转速
			sprintf(tempBuf, "RPM set to %6.0f\r\n", target);
			serialPrint(tempBuf);
		}
	}
}

void cliPrintParam(int i) {
    const char *format = "%-20s = ";

    sprintf(tempBuf, format, configParameterStrings[i]);
    serialPrint(tempBuf);
    sprintf(tempBuf, configFormatStrings[i], p[i]);
    serialPrint(tempBuf);
    serialPrint("\r\n");
}

static void cliFuncSet(void *cmd, char *cmdLine) {
	char param[32];
	float value;
	int i;

	if (sscanf(cmdLine, "%32s", param) != 1) {
		cliUsage((cliCommand_t *)cmd);
	}
	else {
		if (!strcasecmp(param, "list")) {
			for (i = 1; i < CONFIG_NUM_PARAMS; i++)
				cliPrintParam(i);
		}
		else {
			i = configGetId(param);

			if (i < 0) {
				sprintf(tempBuf, "SET: no such parameter '%s'\r\n", param);
				serialPrint(tempBuf);
			}
			else {
				if (sscanf(cmdLine + strlen(param)+1, "%f", &value) == 1) {
					if (state > ESC_STATE_STOPPED) {
						sprintf(tempBuf, stopError);
						serialPrint(tempBuf);
					}
					else {
						configSetParamByID(i, value);
						cliPrintParam(i);
					}
				}
				else {
					cliPrintParam(i);
				}
			}
		}
	}
}
//电机启动
static void cliFuncStart(void *cmd, char *cmdLine) {
	if (state == ESC_STATE_DISARMED) {
		serialPrint("ESC disarmed, arm first\r\n");
	}
	else if (state > ESC_STATE_STOPPED) {
		serialPrint("ESC already running\r\n");
	}
	else {
		runStart();
		serialPrint("ESC started\r\n");
	}
}

static void cliFuncStatus(void *cmd, char *cmdLine) {
    const char *formatFloat = "%-12s%10.2f\r\n";
    const char *formatInt = "%-12s%10d\r\n";
    const char *formatString = "%-12s%10s\r\n";
    float duty;

    duty = (float)fetActualDutyCycle/fetPeriod;//  实际占空比 / 周期

    sprintf(tempBuf, formatString, "INPUT MODE", cliInputModes[inputMode]);//控制输入模式
    serialPrint(tempBuf);

    sprintf(tempBuf, formatString, "RUN MODE", cliRunModes[runMode]);      //运行模式
    serialPrint(tempBuf);

    sprintf(tempBuf, formatString, "ESC STATE", cliStates[state]);         //esc32当前状态
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "PERCENT IDLE", idlePercent);            //空闲时间百分比
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "COMM PERIOD", (float)(crossingPeriod/TIMER_MULT));
    serialPrint(tempBuf);

    sprintf(tempBuf, formatInt, "BAD DETECTS", fetTotalBadDetects);        //fet检测到错误的次数
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "FET DUTY", duty*100.0f);                //fet实际占空比
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "RPM", rpm);                             //电机运行时候的转速
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "AMPS AVG", avgAmps);                    //平均电流(单位安培)
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "AMPS MAX", maxAmps);                    //最大电流(单位安培)
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "BAT VOLTS", avgVolts);                  //电池电压
    serialPrint(tempBuf);

    sprintf(tempBuf, formatFloat, "MOTOR VOLTS", avgVolts*duty);           //电机电压
    serialPrint(tempBuf);

#ifdef ESC_DEBUG
    sprintf(tempBuf, formatInt, "DISARM CODE", disarmReason);
    serialPrint(tempBuf);
#endif
}

static void cliFuncStop(void *cmd, char *cmdLine) {
	if (state < ESC_STATE_NOCOMM) {
		serialPrint(runError);
	}
	else {
		runStop();
		cliFuncChangeInput(ESC_INPUT_UART);
		serialPrint("ESC stopping\r\n");
	}
}

static void cliFuncTelemetry(void *cmd, char *cmdLine) {
	uint16_t freq;

	if (sscanf(cmdLine, "%hu", &freq) != 1) {
		cliUsage((cliCommand_t *)cmd);
	}
	else if (freq > 100) {
		serialPrint("Frequency out of range: 0 => 100\r\n");
	}
	else {
		if (freq > 0) {
			cliTelemetry = 1000/freq;
			serialPrint(cliHome);
			serialPrint(cliClear);
			serialWrite('\n');
		}
		else
			cliTelemetry = 0;
	}
}
//显示版本
static void cliFuncVer(void *cmd, char *cmdLine) {
    sprintf(tempBuf, "ESC32 ver %s\r\n", version);
    serialPrint(tempBuf);
}

//bsearch函数所使用的比较实现函数
static int cliCommandComp(const void *c1, const void *c2) {
    const cliCommand_t *cmd1 = c1, *cmd2 = c2;

    return strncasecmp(cmd1->name, cmd2->name, strlen(cmd2->name));
}
//根据name参数.从cliCommandTable表中找到对应的数组信息
//如果没有找到.那么bsearch会返回空
static cliCommand_t *cliCommandGet(char *name) {
     cliCommand_t target = {NULL, NULL, NULL};
	 target.name = name;

    return bsearch(&target, cliCommandTable, CLI_N_CMDS, sizeof cliCommandTable[0], cliCommandComp);
}
//清空缓冲区
static void cliPrompt(void) {
    serialPrint("\r\n> ");
    memset(cliBuf, 0, sizeof(cliBuf));
    cliBufIndex = 0;
}

void cliCheck(void) {
	cliCommand_t *cmd;

	if (cliTelemetry && !(runMilis % cliTelemetry)) 
	{
		//自动输出电调状态
		serialPrint(cliHome);
		sprintf(tempBuf, "Telemetry @ %d Hz\r\n\n", 1000/cliTelemetry);
		serialPrint(tempBuf);
		cliFuncStatus(cmd, "");
		serialPrint("\n> ");
		serialPrint(cliBuf);
		serialPrint(cliClearEOL);
	}

	while (serialAvailable()) //如果串口收到数据.那么进入循环
	{
		char c = serialRead();

		cliBuf[cliBufIndex++] = c;
		if (cliBufIndex == sizeof(cliBuf)) {
			cliBufIndex--;
			c = '\n';
		}

		// EOL
		if (cliBufIndex && (c == '\n' || c == '\r')) 
		{
			if (cliBufIndex > 1) 
			{
				//收到一个有效的命令.开始比较命令正确性.并相应的执行
				serialPrint("\r\n");
				serialPrint(cliClearEOS);
				cliBuf[cliBufIndex] = 0;

				cmd = cliCommandGet(cliBuf);//cliBuf来找到对应的数组

				if (cmd)
					cmd->cmdFunc(cmd, cliBuf + strlen(cmd->name));
				else
					serialPrint("Command not found");

				if (commandMode != CLI_MODE) {
					cliBufIndex = 0;
					return;
				}
			}

			cliPrompt();
		}
		// interrupt
		else if (c == CLI_INTR) {
			cliPrompt();
		}
		// backspace
		else if (c == CLI_BS) {
			if (cliBufIndex > 1) {
				cliBuf[cliBufIndex-2] = 0;
				serialPrint("\r> ");
				serialPrint(cliBuf);
				serialWrite(' ');
				serialPrint("\r> ");
				serialPrint(cliBuf);
				cliBufIndex -= 2;
			}
			else {
				cliBufIndex--;
			}
		}
		// out of range character
		else if (c < 32 || c > 126) {
			serialWrite(CLI_BELL);
			cliBufIndex--;
		}
		else {
			serialWrite(c);
		}
	}
}
//向串口打印一些信息
void cliInit(void) {
    serialPrint(cliHome);
    serialPrint(cliClear);
    sprintf(version, "%s.%d", VERSION, getBuildNumber());

    cliFuncVer(0, 0);
    serialPrint("\r\nCLI ready.\r\n");

    cliPrompt();
}
