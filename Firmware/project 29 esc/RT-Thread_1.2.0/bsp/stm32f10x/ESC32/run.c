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

#include "run.h"
#include "main.h"
#include "timer.h"
#include "adc.h"
#include "fet.h"
#include "pwm.h"
#include "cli.h"
#include "binary.h"
#include "config.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_dbgmcu.h"
#include <math.h>

uint32_t runMilis;   //systick中断中自加.没有什么控制用途
static uint32_t oldIdleCounter;  //上次main函数中,死循环次数.
float idlePercent;   //空闲时间百分比(在main循环里,什么事情也不做.main死循环运行的时间)
float avgAmps, maxAmps; //平均电流, 最大电流
float avgVolts;      //当前ADC采集转换后的电池电压(也就是12v)

float rpm;           //当前转速(1分钟多少转) 测量值 在runRpm函数中计算出来.在runThrotLim中还要继续使用.
float targetRpm;     //目标转速 设定值(只在闭环 或 闭环推力模式下使用此变量)

static float rpmI;
static float runRPMFactor;
static float maxCurrentSQRT;  //最大电流 平方根 后
uint8_t disarmReason;//此变量没啥作用.只用于给上位机显示当前的 调试代码(或者说停止电机的原因)
uint8_t commandMode; //串口通讯的模式, cli是ascii模式, binary是二进制通讯模式
static uint8_t runArmCount;
volatile uint8_t runMode;//运行模式 (开环模式, RPM模式, 推力模式, 伺服模式)
static float maxThrust;

//执行看门狗喂狗
void runFeedIWDG(void) {
#ifdef RUN_ENABLE_IWDG
    IWDG_ReloadCounter();
#endif
}

// setup the hardware independent watchdog
// 初始化并开启独立看门狗
uint16_t runIWDGInit(int ms) 
{
#ifndef RUN_ENABLE_IWDG
    return 0;
#else
	uint16_t prevReloadVal;
	int reloadVal;

	IWDG_ReloadCounter();//喂狗

	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);//当在jtag调试的时候.停止看门狗

	// IWDG timeout equal to 10 ms (the timeout may varies due to LSI frequency dispersion)
	// Enable write access to IWDG_PR and IWDG_RLR registers
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//允许访问IWDG_PR和IWDG_RLR寄存器

	// IWDG counter clock: LSI/4
	IWDG_SetPrescaler(IWDG_Prescaler_4);

	// Set counter reload value to obtain 10ms IWDG TimeOut.
	//  Counter Reload Value	= 10ms/IWDG counter clock period
	//				= 10ms / (RUN_LSI_FREQ/4)
	//				= 0.01s / (RUN_LSI_FREQ/4)
	//				= RUN_LSI_FREQ/(4 * 100)
	//				= RUN_LSI_FREQ/400
	reloadVal = RUN_LSI_FREQ*ms/4000;

	if (reloadVal < 1)
		reloadVal = 1;
	else if (reloadVal > 0xfff)
		reloadVal = 0xfff;

	prevReloadVal = IWDG->RLR;

	IWDG_SetReload(reloadVal);

	// Reload IWDG counter
	IWDG_ReloadCounter();

	// Enable IWDG (the LSI oscillator will be enabled by hardware)
	IWDG_Enable();

	return (prevReloadVal*4000/RUN_LSI_FREQ);
#endif
}

//esc32 非正常停止运行 进入初始化
void runDisarm(int reason) {
	fetSetDutyCycle(0);  //fet占空比设置为0

	timerCancelAlarm2();
	state = ESC_STATE_DISARMED;
	pwmIsrAllOn();

	digitalHi(statusLed);   // turn off
	digitalLo(errorLed);    // turn on
	disarmReason = reason;  // 设置停机原因.给上位机查看状态使用
}

//手动运行
void runArm(void) {
	int i;

	fetSetDutyCycle(0);
	timerCancelAlarm2();
	digitalHi(errorLed);
	digitalLo(statusLed);   // turn on

	if (runMode == SERVO_MODE) {
		state = ESC_STATE_RUNNING;
	}
	else {
		state = ESC_STATE_STOPPED;
		if (inputMode == ESC_INPUT_UART)
			runMode = OPEN_LOOP;
		fetSetBraking(0);
	}

	// extra beeps signifying run mode
	for (i = 0; i < runMode + 1; i++) {
		fetBeep(250, 600);
		timerDelay(10000);
	}

//	fetBeep(150, 800);
}

//电机开始运行
void runStart(void) {
	// reset integral bevore new motor startup
	runRpmPIDReset();//先复位I值

	if ((p[START_ALIGN_TIME] == 0) && (p[START_STEPS_NUM] == 0)) {
		state = ESC_STATE_STARTING;  //设置为准备启动状态
		fetStartCommutation(0);//换向启动
	}
	else {
		motorStartSeqInit();//普通启动
	}
}

//电机停止运行
void runStop(void) {
    runMode = OPEN_LOOP;
    fetSetDutyCycle(0);
}

//设置运行的占空比 duty = 0~100
uint8_t runDuty(float duty) {
    uint8_t ret = 0;

    if (duty >= 0.0f || duty <= 100.0f) {
		runMode = OPEN_LOOP;
		fetSetBraking(0);
		fetSetDutyCycle((uint16_t)(fetPeriod*duty*0.01f));//最大周期 * 占空比(0~100) / 100
		ret = 1;
    }

    return ret;
}

//pwm.c中断中调用  或  串口命令输入调用
void runNewInput(uint16_t setpoint) {
	static uint16_t lastPwm;
	static float filteredSetpoint = 0;

	// Lowpass Input if configured
	// TODO: Make lowpass independent from pwm update rate
	if (p[PWM_LOWPASS]) {
		filteredSetpoint = (p[PWM_LOWPASS] * filteredSetpoint + (float)setpoint) / (1.0f + p[PWM_LOWPASS]);
		setpoint = filteredSetpoint;
	}

	if (state == ESC_STATE_RUNNING && setpoint != lastPwm) 
	{
		if (runMode == OPEN_LOOP) 
		{
			//开环模式
			fetSetDutyCycle(fetPeriod * (int32_t)(setpoint-pwmLoValue) / (int32_t)(pwmHiValue - pwmLoValue));
		}
		else if (runMode == CLOSED_LOOP_RPM) 
		{
			//闭环转速模式
			float target = p[PWM_RPM_SCALE] * (setpoint-pwmLoValue) / (pwmHiValue - pwmLoValue);

			// limit to configured maximum
			targetRpm = (target > p[PWM_RPM_SCALE]) ? p[PWM_RPM_SCALE] : target;
		}
		// THRUST Mode
		else if (runMode == CLOSED_LOOP_THRUST) 
		{
			//闭环推力模式
			float targetThrust;  // desired trust
			float target;        // target(rpm)

			// Calculate targetThrust based on input and MAX_THRUST
			targetThrust = maxThrust * (setpoint-pwmLoValue) / (pwmHiValue - pwmLoValue);

			// Workaraound: Negative targetThrust will screw up sqrtf() and create MAX_RPM on throttle min. Dangerous!
			if (targetThrust > 0.0f) {
				// Calculate target(rpm) based on targetThrust
				target = ((sqrtf(p[THR1TERM] * p[THR1TERM] + 4.0f * p[THR2TERM] * targetThrust) - p[THR1TERM] ) / ( 2.0f * p[THR2TERM] ));
			}
			// targetThrust is negative (pwm_in < pwmLoValue)
			else {
				target = 0.0f;
			}

			// upper limit for targetRpm is configured maximum PWM_RPM_SCALE (which is MAX_RPM)
			targetRpm = (target > p[PWM_RPM_SCALE]) ? p[PWM_RPM_SCALE] : target;
		}
		else if (runMode == SERVO_MODE) 
		{
			//伺服模式下
			fetSetAngleFromPwm(setpoint);
		}

		lastPwm = setpoint;
	}
	else if ((state == ESC_STATE_NOCOMM || state == ESC_STATE_STARTING) && setpoint <= pwmLoValue) 
	{
		fetSetDutyCycle(0);
		state = ESC_STATE_RUNNING;
	}
	else if (state == ESC_STATE_DISARMED && setpoint > pwmMinValue && setpoint <= pwmLoValue) 
	{
		runArmCount++;
		if (runArmCount > RUN_ARM_COUNT)
			runArm();
	}
	else {
		runArmCount = 0;
	}

	if (state == ESC_STATE_STOPPED && setpoint >= pwmMinStart) {
		//电机开始运行
		runStart();
	}
}

//电调运行看门狗. 主要是判断电调的当前一些状态.做出停机等处理
static void runWatchDog(void) 
{
	register uint32_t t, d, p;

	//__asm volatile ("cpsid i");
	//CPSID_I();
	__disable_irq();
	t = timerMicros;      //当前的系统tick时间
	d = detectedCrossing;
	p = pwmValidMicros;   //在PWM输入模式下.把timerMicros的时间赋值给此变量
	//__asm volatile ("cpsie i");
	//CPSIE_I();
	__enable_irq();

	if (state == ESC_STATE_STARTING && fetGoodDetects > fetStartDetects) //这里要检测到fetStartDetects好的检测,才允许切换电机状态
	{
		//是启动状态.切换到 运行状态
		state = ESC_STATE_RUNNING;
		digitalHi(statusLed);   // turn off
	}
	else if (state >= ESC_STATE_STOPPED) 
	{
		//运行模式状态下.会一直在这里检测状态.如果状态不对出错.会调用runDisarm函数停止

		// running or starting
		d = (t >= d) ? (t - d) : (TIMER_MASK - d + t);

		// timeout if PWM signal disappears
		if (inputMode == ESC_INPUT_PWM) 
		{
			//PWM模式 判断PWM输入是否超时
			p = (t >= p) ? (t - p) : (TIMER_MASK - p + t);

			if (p > PWM_TIMEOUT)
				runDisarm(REASON_PWM_TIMEOUT);//pwm输入超时
		}

		if (state >= ESC_STATE_STARTING && d > ADC_CROSSING_TIMEOUT) 
		{
			if (fetDutyCycle > 0) {
				runDisarm(REASON_CROSSING_TIMEOUT);//错误停止
			}
			else 
			{
				runArm();//手动运行起来
				pwmIsrRunOn();//PWM开启输入比较
			}
		}
		else if (state >= ESC_STATE_STARTING && fetBadDetects > fetDisarmDetects)  //运行状态中  检测到错误的个数后.进入这个判断
		{
			//在运行过程中,出现错误.停止运行
			if (fetDutyCycle > 0)
				runDisarm(REASON_BAD_DETECTS);//错误停止
		}
		else if (state == ESC_STATE_STOPPED) 
		{
			//停止模式
			adcAmpsOffset = adcAvgAmps;	// record current amperage offset
		}
	}
	else if (state == ESC_STATE_DISARMED && !(runMilis % 100)) 
	{
		//停止模式下
		adcAmpsOffset = adcAvgAmps;	// record current amperage offset
		digitalTogg(errorLed);
	}
}

void runRpmPIDReset(void) {
    rpmI = 0.0f;
}

//这个应该是计算PID
//rpm:测量的转速值
//target:目标的转速值
static int32_t runRpmPID(float rpm, float target) {
	float error;
	float ff, rpmP;
	float iTerm = rpmI;
	float output;

	// feed forward
	ff = ((target*target* p[FF1TERM] + target*p[FF2TERM]) / avgVolts) * fetPeriod;

	error = (target - rpm);//计算出偏差

	if (error > 1000.0f)
		error = 1000.0f;

	if (error > 0.0f) {
		rpmP = error * p[PTERM];  //P
		rpmI += error * p[ITERM]; //I
	}
	else {
		rpmP =  error * p[PTERM] * p[PNFAC];
		rpmI += error * p[ITERM] * p[INFAC];
	}

	if (fetBrakingEnabled) 
	{
		//开启了制动模式
		if (rpm < 300.0f) {
			fetSetBraking(0);
		}
		else if (error <= -100.0f) {
			fetSetBraking(1);
		}
		else if (fetBraking && error > -25.0f){
			fetSetBraking(0);
		}
	}

	output = ff + (rpmP + rpmI) * (1.0f / 1500.0f) * fetPeriod;

	// don't allow integral to continue to rise if at max output
	if (output >= fetPeriod)
		rpmI = iTerm;

	return output;
}

//计算出电机转速,根据当前转速计算出PID输出值,设置占空比
static uint8_t runRpm(void) 
{
    if (state > ESC_STATE_STARTING) 
	{
		//电机处于运行状态 计算出当前转速rpm
		//	rpm = rpm * 0.90f + (runRPMFactor / (float)crossingPeriod) * 0.10f;
		//	rpm -= (rpm - (runRPMFactor / (float)crossingPeriod)) * 0.25f;
		//	rpm = (rpm + (runRPMFactor / (float)crossingPeriod)) * 0.5f;
		//	rpm = (rpm + ((32768.0f * runRPMFactor) / (float)adcCrossingPeriod)) * 0.5f; // increased resolution, fixed filter here
		rpm = p[RPM_MEAS_LP] * rpm + ((32768.0f * runRPMFactor) / (float)adcCrossingPeriod) * (1.0f - p[RPM_MEAS_LP]); // increased resolution, variable filter here

		// run closed loop control
		if (runMode == CLOSED_LOOP_RPM) 
		{
			//运行在闭环模式下
			fetSetDutyCycle(runRpmPID(rpm, targetRpm));
			return 1;
		}
		// run closed loop control also for THRUST mode
		else if (runMode == CLOSED_LOOP_THRUST) 
		{
			//运行在闭环推力模式
			fetSetDutyCycle(runRpmPID(rpm, targetRpm));
			return 1;
		}
		else 
		{
			return 0;
		}
	}
	else 
	{
		//电机在停止状态下
		rpm = 0.0f;
		return 0;
    }
}

static void runSetupPVD(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Configure EXTI Line16(PVD Output) to generate an interrupt on rising and falling edges
    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable the PVD Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure the PVD Level to 2.2V
    PWR_PVDLevelConfig(PWR_PVDLevel_2V2);//配置pvd电压等级.当电压小于2.2V的时候产生中断

    // Enable the PVD Output
    PWR_PVDCmd(ENABLE);
}

void runInit(void) {
    runSetupPVD();
    runSetConstants();
    runMode = p[STARTUP_MODE];//启动 运行模式

	//系统tickcount时钟
	//SysTick is used for rtt and the systick_interrupt is replace by a thread, which is located in main.c  
   // SysTick_Config(SystemCoreClock / 1000); // 1ms
   // NVIC_SetPriority(SysTick_IRQn, 2);	    // lower priority

    // setup hardware watchdog
    runIWDGInit(20);
}

#define RUN_CURRENT_ITERM	1.0f
#define RUN_CURRENT_PTERM	10.0f
#define RUN_MAX_DUTY_INCREASE	1.0f

float currentIState;

//根据PID计算出PWM占空比的值
static int32_t runCurrentPID(int32_t duty) {
    float error;
    float pTerm, iTerm;

    error = avgAmps - p[MAX_CURRENT];

    currentIState += error;
    if (currentIState < 0.0f)
		currentIState = 0.0f;
    iTerm = currentIState * RUN_CURRENT_ITERM;

    pTerm = error * RUN_CURRENT_PTERM;
    if (pTerm < 0.0f)
		pTerm = 0.0f;

    duty = duty - iTerm - pTerm;

    if (duty < 0)
		duty = 0;

    return duty;
}

//计算得到实际的占空比fetActualDutyCycle
//参数duty:实际上就是fetDutyCycle传递进来的.想要运行的周期
static void runThrotLim(int32_t duty) 
{
	float maxVolts; //最大的电压
	int32_t maxDuty;//最大的周期

	// only if a limit is set
	if (p[MAX_CURRENT] > 0.0f) 
	{
		//如果实际的占空比和设置的占空比不一样.那么会实时改变CPU的PWM寄存器.

		// if current limiter is calibrated - best performance   使用电流限制器校准.性能最好
		if (p[CL1TERM] != 0.0f) 
		{
			maxVolts = p[CL1TERM] + p[CL2TERM]*rpm + p[CL3TERM]*p[MAX_CURRENT] + p[CL4TERM]*rpm*maxCurrentSQRT + p[CL5TERM]*maxCurrentSQRT;
			maxDuty = maxVolts * (fetPeriod / avgVolts);

			if (duty > maxDuty)
				fetActualDutyCycle = maxDuty;
			else
				fetActualDutyCycle = duty;
		}
		// otherwise, use PID - less accurate, lower performance  使用PID来计算.不大准确.性能低
		else 
		{
			fetActualDutyCycle += fetPeriod * (RUN_MAX_DUTY_INCREASE * 0.01f);
			if (fetActualDutyCycle > duty)
				fetActualDutyCycle = duty;
			fetActualDutyCycle = runCurrentPID(fetActualDutyCycle);//用PID来计算出当前要运行的占空比
		}
	}
	else {
		fetActualDutyCycle = duty;
	}

	//设置到CPU寄存器里.算出来的实际PWM占空比
	_fetSetDutyCycle(fetActualDutyCycle);
}

#if 1

//系统tickcount中断
//void SysTick_Handler(void) {
void period_1ms(void)
{
    // reload the hardware watchdog
    runFeedIWDG();


    avgVolts = adcAvgVolts * ADC_TO_VOLTS;                     //转换后的电池电压(一般是12V) = ADC采集电压原始值 * 电压算法
    avgAmps = (adcAvgAmps - adcAmpsOffset) * adcToAmps;        //平均电流 = (当前电流 - 停止时候的电流) * 转换公式
    maxAmps = (adcMaxAmps - adcAmpsOffset) * adcToAmps;        //最大电流 = (最大电流 - 停止时候的电流) * 转换公式


    if (runMode == SERVO_MODE) 
	{
		//伺服模式
		fetUpdateServo();
    }
    else 
	{
		runWatchDog();//检测电调的状态.做出相应的停机处理
		runRpm();     //计算RPM,计算PID,设置运行PWM占空比
		runThrotLim(fetDutyCycle);//计算得到实际PWM占空比.如果有偏差.那么在这里会实时改变PWM的占空比值
    }


//not used anymore
	//计算空闲时间百分比 通过串口发送给上位机  没什么用途
//    idlePercent = 100.0f * (idleCounter-oldIdleCounter) * minCycles / totalCycles;
//  //空闲时间百分比 = 100 * (本次循环次数 - 上次循环次数) * 最小周期 / 总共周期
//    oldIdleCounter = idleCounter;
//    totalCycles = 0;


	//处理串口数据 和串口交互使用的
    if (commandMode == CLI_MODE)
		cliCheck();    //ascii模式
    else
		binaryCheck(); //二进制模式

    runMilis++;
}

#endif

//低电压中断
void PVD_IRQHandler(void) {
    // voltage dropping too low
    if (EXTI_GetITStatus(EXTI_Line16) != RESET) {
		// shut everything down
		runDisarm(REASON_LOW_VOLTAGE);

		// turn on both LEDs
		digitalLo(statusLed);
		digitalLo(errorLed);

		EXTI_ClearITPendingBit(EXTI_Line16);
    }
}

void runSetConstants(void) {
    int32_t startupMode = (int)p[STARTUP_MODE];
    float maxCurrent = p[MAX_CURRENT];

	//运行模式
    if (startupMode < 0 || startupMode >= NUM_RUN_MODES)
		startupMode = 0;

    if (maxCurrent > RUN_MAX_MAX_CURRENT)
		maxCurrent = RUN_MAX_MAX_CURRENT;
    else if (maxCurrent < RUN_MIN_MAX_CURRENT)
		maxCurrent = RUN_MIN_MAX_CURRENT;

    runRPMFactor = (1e6f * (float)TIMER_MULT * 120.0f) / (p[MOTOR_POLES] * 6.0f);
    maxCurrentSQRT = sqrtf(maxCurrent);

    p[MOTOR_POLES] = (int)p[MOTOR_POLES];
    p[STARTUP_MODE] = startupMode;
    p[MAX_CURRENT] = maxCurrent;

    // Calculate MAX_THRUST from PWM_RPM_SCALE (which is MAX_RPM) and THRxTERMs
    // Based on "thrust = rpm * a1 + rpm^2 * a2"
    maxThrust = p[PWM_RPM_SCALE] * p[THR1TERM] + p[PWM_RPM_SCALE] * p[PWM_RPM_SCALE] * p[THR2TERM];
}
