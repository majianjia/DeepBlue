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

#include "main.h"
#include "fet.h"
#include "timer.h"
#include "adc.h"
#include "run.h"
#include "config.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_iwdg.h"
#include "misc.h"
#include <math.h>

/*
               -----------------------------------------------
              | Step1 | Step2 | Step3 | Step4 | Step5 | Step6 |
    ----------------------------------------------------------
   | A_H      |   1   |   0   |   0   |   0   |   0   |   1   |
    ----------------------------------------------------------
   | A_L      |   0   |   0   |   1   |   1   |   0   |   0   |
    ----------------------------------------------------------
   | B_H      |   0   |   0   |   0   |   1   |   1   |   0   |
    ----------------------------------------------------------
   | B_L      |   1   |   1   |   0   |   0   |   0   |   0   |
    ----------------------------------------------------------
   | C_H      |   0   |   1   |   1   |   0   |   0   |   0   |
    ----------------------------------------------------------
   | C_L      |   0   |   0   |   0   |   0   |   1   |   1   |
    ----------------------------------------------------------
*
* Step1 AH -> BL
* Step2 CH -> BL
* Step3 CH -> AL
* Step4 BH -> AL
* Step5 BH -> CL
* Step6 AH -> CL
*/

//以下这几个数组只会在fetSetStep函数中用到
//AH BH CH都是切换gpio的模式,设置为PWM或io模式
//AL BL CL都是设置gpio的输出电平
//                       STEP0   STEP1   STEP2   STEP3   STEP4   STEP5   STEP6      对照上面的表格(STEP0不要参与对照)
static uint32_t AH[7] = {  0,      1,      0,      0,      0,      0,      1};
static uint32_t AL[7] = {AL_OFF, AL_OFF, AL_OFF, AL_ON,  AL_ON,  AL_OFF, AL_OFF};
static uint32_t BH[7] = {  0,      0,      0,      0,      1,      1,      0};
static uint32_t BL[7] = {BL_ON,  BL_ON,  BL_ON,  BL_OFF, BL_OFF, BL_OFF, BL_OFF};	// note: step 0 has B_LO energized
static uint32_t CH[7] = {  0,      0,      1,      1,      0,      0,      0};
static uint32_t CL[7] = {CL_OFF, CL_OFF, CL_OFF, CL_OFF, CL_OFF, CL_ON,  CL_ON};


static int32_t fetSwitchFreq;
static int32_t fetStartDuty;    //在启动过程中.计算出启动那瞬间的PWM占空比

int16_t fetStartDetects;        //启动时.要检测到大于此变量的值.电机才算没问题.然后切换到运行状态
int16_t fetDisarmDetects;

int32_t fetPeriod;              //fet最大的周期
int32_t fetActualDutyCycle;     //fet实际的占空周期
volatile int32_t fetDutyCycle;  //fet设置的占空比 当此变量大于0时.代表电机运转了 等于0代表停止运行

volatile uint8_t fetStep;       //在adc中断函数中,会计算出下一个要运行的电机Step
static volatile uint8_t fetNextStep;//电机运行的下一个Step, 函数fetSetStep中,会自己指定到下一个Step

volatile uint32_t fetGoodDetects;    //fet 正确的检测次数
volatile uint32_t fetBadDetects;     //fet 总共错误的检测次数
volatile uint32_t fetTotalBadDetects;//fet 总共错误的检测次数

volatile uint32_t fetCommutationMicros;//电机换向的时间(在换向的时候获取到)
int8_t fetBrakingEnabled;//=1 开启制动模式,允许制动,在参数表中设置
int8_t fetBraking;       //=1 制动模式 
static int16_t startSeqCnt;
static int8_t  startSeqStp; //电机刚启动的时候用到的变量 电机换向变量
static float fetServoAngle; //电机要运行的目标角度(仅在伺服模式下有效) PID中的设定值
static float fetServoMaxRate;

static int16_t fetSine[FET_SERVO_RESOLUTION];


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 下面5个函数都在伺服模式下使用的
static void fetCreateSine(void) {
	float a;
	int i;

	for (i = 0; i < FET_SERVO_RESOLUTION; i++) {
		a = M_PI * 2.0f * i / FET_SERVO_RESOLUTION; //乘以2 扩大2倍的方式来计算???

		// third order harmonic injection
		fetSine[i] = (sinf(a) + sinf(a*3.0f)/6.0f) * (2.0f/sqrtf(3.0f)) * (float)fetPeriod / 2.0f;
	}
}

//设置伺服模式下 计算出来的占空比
static void _fetSetServoDuty(uint16_t duty[3]) {
    FET_H_TIMER->FET_A_H_CHANNEL = duty[0];
    FET_H_TIMER->FET_B_H_CHANNEL = duty[1];
    FET_H_TIMER->FET_C_H_CHANNEL = duty[2];

    FET_MASTER_TIMER->FET_A_L_CHANNEL = duty[0] + FET_DEADTIME;
    FET_MASTER_TIMER->FET_B_L_CHANNEL = duty[1] + FET_DEADTIME;
    FET_MASTER_TIMER->FET_C_L_CHANNEL = duty[2] + FET_DEADTIME;
}

//fet伺服模式下,pid计算
void fetUpdateServo(void) {
	static float myAngle = 0.0f;       //上次的采样值
	static float servoDState = 0.0f;   //微分计算
	float a, e;

	uint16_t pwm[3];
	int index;

	if (state == ESC_STATE_RUNNING) 
	{
		//全部设置为PWM模式
		*AL_BITBAND = 1;
		*BL_BITBAND = 1;
		*CL_BITBAND = 1;

		*AH_BITBAND = 1;
		*BH_BITBAND = 1;
		*CH_BITBAND = 1;


		//伺服模式下 PID计算 P计算
		e = (fetServoAngle - myAngle);
		a = e * p[SERVO_P];               //比例计算
		if (a > fetServoMaxRate)
			a = fetServoMaxRate;
		else if (a < -fetServoMaxRate)
			a = -fetServoMaxRate;

		myAngle += a;

		myAngle += (a - servoDState) * p[SERVO_D];   //微分计算
		servoDState = a;


		//求出fetSine数组的index号
		index = ((float)FET_SERVO_RESOLUTION * myAngle / 360.0f);
		while (index < 0)
			index += FET_SERVO_RESOLUTION;
		index = index % FET_SERVO_RESOLUTION;

		//计算出3个输出占空比
		pwm[0] = fetPeriod/2 + fetSine[index] * p[SERVO_DUTY] / 100;

		index = ((index + FET_SERVO_RESOLUTION / 3) % FET_SERVO_RESOLUTION);
		pwm[1] = fetPeriod/2 + fetSine[index] * p[SERVO_DUTY] / 100;

		index = ((index + FET_SERVO_RESOLUTION / 3) % FET_SERVO_RESOLUTION);
		pwm[2] = fetPeriod/2 + fetSine[index] * p[SERVO_DUTY] / 100;

		//设置到CPU寄存器里
		_fetSetServoDuty(pwm);
	}
	else 
	{
		//停止运行状态下 全部设置为GPIO模式
		*AL_BITBAND = 0;
		*BL_BITBAND = 0;
		*CL_BITBAND = 0;

		*AH_BITBAND = 0;
		*BH_BITBAND = 0;
		*CH_BITBAND = 0;
	}
}

//电机要运行的角度,从获取到的PWM值设置(仅在伺服模式下使用)
void fetSetAngleFromPwm(int32_t pwm) {
    fetServoAngle = pwm * p[SERVO_SCALE] * p[MOTOR_POLES] * 0.5f / fetPeriod;
}

//在cli.c文件中,函数cliFuncPos调用,(只在伺服模式下使用),设置电机要运行到什么角度
//angle：电机要运行到目标的角度
void fetSetAngle(float angle) {
    fetServoAngle = angle * p[MOTOR_POLES] * 0.5f;
//	fetServoAngle = 角度 * p / 2
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if 0
#define FET_TEST_DELAY	1000
uint8_t fetSelfTest(void) {
	int32_t baseCurrent;
	int32_t cl1, cl2, cl3;
	int32_t ch1, ch2, ch3;

	// must be disarmed to run self test
	if (state != ESC_STATE_DISARMED)
		return FET_TEST_NOT_RUN;

	fetSetStep(0);

	// shut everything off 所有的GPIO输出低电平
	FET_A_L_OFF;
	FET_B_L_OFF;
	FET_C_L_OFF;

	FET_A_H_OFF;
	FET_B_H_OFF;
	FET_C_H_OFF;

	// record base current
	timerDelay(FET_TEST_DELAY);
	baseCurrent = adcAvgAmps;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 测试 ：AH BH CH  PWM输出，AL BL CL 全部都输出高电平
	// manually set HI output duty cycle (1/16th power)
	FET_H_TIMER->FET_A_H_CHANNEL = fetPeriod - (fetPeriod>>4);
	FET_H_TIMER->FET_B_H_CHANNEL = fetPeriod - (fetPeriod>>4);
	FET_H_TIMER->FET_C_H_CHANNEL = fetPeriod - (fetPeriod>>4);

	// all lows on
	FET_A_L_ON;
	FET_B_L_ON;
	FET_C_L_ON;

	timerDelay(FET_TEST_DELAY*10);

	// Phase A hi FET
	*AH_BITBAND = 1;
	timerDelay(FET_TEST_DELAY);
	ch1 = (adcAvgAmps - baseCurrent)>>ADC_AMPS_PRECISION;
	*AH_BITBAND = 0;
	timerDelay(FET_TEST_DELAY*10);

	// Phase B hi FET
	*BH_BITBAND = 1;
	timerDelay(FET_TEST_DELAY);
	ch2 = (adcAvgAmps - baseCurrent)>>ADC_AMPS_PRECISION;
	*BH_BITBAND = 0;
	timerDelay(FET_TEST_DELAY*10);

	// Phase C hi FET
	*CH_BITBAND = 1;
	timerDelay(FET_TEST_DELAY);
	ch3 = (adcAvgAmps - baseCurrent)>>ADC_AMPS_PRECISION;
	*CH_BITBAND = 0;
	timerDelay(FET_TEST_DELAY*10);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 测试 ：AH BH CH  全部都输出高电平，AL BL CL PWM输出
	// all lows off
	FET_A_L_OFF;
	FET_B_L_OFF;
	FET_C_L_OFF;

	// manually set LO output duty cycle (1/16th power)
	FET_MASTER_TIMER->FET_A_L_CHANNEL = (fetPeriod>>4);
	FET_MASTER_TIMER->FET_B_L_CHANNEL = (fetPeriod>>4);
	FET_MASTER_TIMER->FET_C_L_CHANNEL = (fetPeriod>>4);

	*AL_BITBAND = 0;
	*BL_BITBAND = 0;
	*CL_BITBAND = 0;

	timerDelay(FET_TEST_DELAY*10);

	// all highs on
	FET_A_H_ON;
	FET_B_H_ON;
	FET_C_H_ON;

	timerDelay(FET_TEST_DELAY*10);

	// Phase A lo FET
	*AL_BITBAND = 1;
	timerDelay(FET_TEST_DELAY);
	cl1 = (adcAvgAmps - baseCurrent)>>ADC_AMPS_PRECISION;
	*AL_BITBAND = 0;
	timerDelay(FET_TEST_DELAY*10);

	// Phase B lo FET
	*BL_BITBAND = 1;
	timerDelay(FET_TEST_DELAY);
	cl2 = (adcAvgAmps - baseCurrent)>>ADC_AMPS_PRECISION;
	*BL_BITBAND = 0;
	timerDelay(FET_TEST_DELAY*10);

	// Phase C lo FET
	*CL_BITBAND = 1;
	timerDelay(FET_TEST_DELAY);
	cl3 = (adcAvgAmps - baseCurrent)>>ADC_AMPS_PRECISION;
	*CL_BITBAND = 0;
	timerDelay(FET_TEST_DELAY*10);


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// shut everything off
	FET_A_L_OFF;
	FET_B_L_OFF;
	FET_C_L_OFF;

	FET_A_H_OFF;
	FET_B_H_OFF;
	FET_C_H_OFF;

	*AL_BITBAND = 0;
	*BL_BITBAND = 0;
	*CL_BITBAND = 0;

	_fetSetDutyCycle(0);
	fetSetStep(0);

	if (cl1 < 50)
		return FET_TEST_A_LO_FAIL;
	if (cl2 < 50)
		return FET_TEST_B_LO_FAIL;
	if (cl3 < 50)
		return FET_TEST_C_LO_FAIL;
	if (ch1 < 50)
		return FET_TEST_A_HI_FAIL;
	if (ch2 < 50)
		return FET_TEST_B_HI_FAIL;
	if (ch3 < 50)
		return FET_TEST_C_HI_FAIL;

	return FET_TEST_PASSED;
}
#endif


// this assume that one low FET is conducting (s/b B)
void fetBeep(uint16_t freq, uint16_t duration) {
	uint16_t prevReloadVal;
	int i;

	fetSetStep(0);

	prevReloadVal = runIWDGInit(999);

	//__asm volatile ("cpsid i");
	//CPSID_I();
	__disable_irq();

	for (i = 0; i < duration; i++) {
		// reload the hardware watchdog
		runFeedIWDG();

		FET_A_H_ON;
		timerDelay(8);
		FET_A_H_OFF;

		timerDelay(freq);//us级别延时

		// reload the hardware watchdog
		runFeedIWDG();

		FET_C_L_ON;
		timerDelay(8);
		FET_C_H_OFF;

		timerDelay(freq);
	}

	//__asm volatile ("cpsie i");
	//CPSIE_I();
	__enable_irq();

	runIWDGInit(prevReloadVal);
}

//value:=1 设置制动模式
void fetSetBraking(int8_t value) {
	if (value) {
		// set low side for inverted PWM
		//	*AL_BITBAND = AH[fetStep];
		//	*BL_BITBAND = BH[fetStep];
		//	*CL_BITBAND = CH[fetStep];
		//设置AL BL CL 为pwm输出模式
		*AL_BITBAND = 1;
		*BL_BITBAND = 1;
		*CL_BITBAND = 1;
		fetBraking = 1;
	}
	else {
		// switch off PWM
		//设置AL BL CL 为gpio输出模式
		*AL_BITBAND = 0;
		*BL_BITBAND = 0;
		*CL_BITBAND = 0;
		fetBraking = 0;
	}
}

//设置占空比到CPU寄存器
void _fetSetDutyCycle(int32_t dutyCycle) {
	register int32_t tmp;

	tmp = dutyCycle;

	if (state == ESC_STATE_DISARMED)
		tmp = 0;

	//改变TIMER4的占空比 当改变duty后.要设置CPU的寄存器.
	FET_H_TIMER->FET_A_H_CHANNEL = tmp;
	FET_H_TIMER->FET_B_H_CHANNEL = tmp;
	FET_H_TIMER->FET_C_H_CHANNEL = tmp;

	if (fetBrakingEnabled) 
	{
		//制动模式开启
		tmp = dutyCycle + fetPeriod / 8;

		if (tmp < 0)
			tmp = 0;
		else if (tmp > fetPeriod)
			tmp = fetPeriod;

		//改变TIMER3的占空比
		FET_MASTER_TIMER->FET_A_L_CHANNEL = tmp;
		FET_MASTER_TIMER->FET_B_L_CHANNEL = tmp;
		FET_MASTER_TIMER->FET_C_L_CHANNEL = tmp;
	}
}

//设置fet占空比到变量(无论使用哪个模式,最后都要通过此函数来设置PWM占空比)
void fetSetDutyCycle(int32_t requestedDutyCycle) {
	if (requestedDutyCycle > fetPeriod)
		requestedDutyCycle = fetPeriod;
	else if (requestedDutyCycle < 0)
		requestedDutyCycle = 0;

	fetDutyCycle = requestedDutyCycle;
}

//
// Low side FET switching is done via direct GPIO manipulation
// High side FET switching is accomplished by enabling or disabling
// control of the output pin by the PWM timer.  When disabled, the
// GPIO output state is imposed on the pin (FET off.)
// 设置FET下一步 正常运行过程中,通过调用这个函数.来不断切换STEP.改变无刷电机运行状态
// 参数n：代表目前要输出的step
static void fetSetStep(int n) {
    //__asm volatile ("cpsid i");
	//CPSID_I();
	__disable_irq();
    fetCommutationMicros = timerGetMicros();
    //__asm volatile ("cpsie i");
	//CPSIE_I();
	__enable_irq();

	// 下一步
    fetNextStep = n + 1;
    if (fetNextStep > 6)
		fetNextStep = 1;

    // set high side
	// 通过切换H侧,GPIO和PWM模式,来改变是哪个STEP
    *AH_BITBAND = AH[n];//GPIOB_6
    *BH_BITBAND = BH[n];//GPIOB_7
    *CH_BITBAND = CH[n];//GPIOB_8

    if (fetBrakingEnabled)
		fetSetBraking(fetBraking);

    // set low side 
    // L侧 GPIO切换高低电平
    FET_A_L_PORT->BSRR = AL[n];//GPIOA_7
    FET_B_L_PORT->BSRR = BL[n];//GPIOB_0
    FET_C_L_PORT->BSRR = CL[n];//GPIOB_1
}

static void fetSetBaseTime(int32_t period) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 1;                      // 36Mhz
    TIM_TimeBaseStructure.TIM_Period = period-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(FET_MASTER_TIMER, &TIM_TimeBaseStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 1;                      // 36Mhz
    TIM_TimeBaseStructure.TIM_Period = period-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(FET_H_TIMER, &TIM_TimeBaseStructure);

    // sync clocks
    FET_H_TIMER->CNT = 0;
    FET_MASTER_TIMER->CNT = 0;
}

void fetInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    fetSetConstants();

    fetDutyCycle = fetPeriod;
    fetStep = 0;

	//////////////////////////////////////////////////////////////////////////
    // setup low side gates
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // A GPIOA_7
    GPIO_InitStructure.GPIO_Pin = FET_A_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_A_L_PORT, &GPIO_InitStructure);

    // B GPIOB_0
    GPIO_InitStructure.GPIO_Pin = FET_B_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_B_L_PORT, &GPIO_InitStructure);

    // C GPIOB_1
    GPIO_InitStructure.GPIO_Pin = FET_C_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_C_L_PORT, &GPIO_InitStructure);

	//////////////////////////////////////////////////////////////////////////
    // setup GPIO default output states for high side
    // A GPIOB_6
    GPIO_InitStructure.GPIO_Pin = FET_A_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_A_H_PORT, &GPIO_InitStructure);

    // B GPIOB_7
    GPIO_InitStructure.GPIO_Pin = FET_B_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_B_H_PORT, &GPIO_InitStructure);

    // C GPIOB_8
    GPIO_InitStructure.GPIO_Pin = FET_C_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_C_H_PORT, &GPIO_InitStructure);

    // high side N-FET is inactive low 将对应的IO设置为低电平
    FET_A_H_PORT->BSRR = AH_OFF;//GPIOB_6
    FET_B_H_PORT->BSRR = BH_OFF;//GPIOB_7
    FET_C_H_PORT->BSRR = CH_OFF;//GPIOB_8

	//////////////////////////////////////////////////////////////////////////
	//在jtag调试时候.停止定时器
    // allow FET PWM (slave) to run during core halt
    DBGMCU_Config(FET_DBGMCU_STOP, ENABLE);//timer4
    // stop MASTER during core halt
    DBGMCU_Config(FET_MASTER_DBGMCU_STOP, ENABLE);//timer3


	//////////////////////////////////////////////////////////////////////////
	//配置timer3
	//中央对齐模式，TIMx_ARR寄存器被装入缓冲器，
	//CC2、CC3、CC4为输出模式，输出比较预装载使能，PWM模式1
    // setup LO side inverted PWM for FET braking (if enabled)
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = fetDutyCycle;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

    // Phase A
    TIM_OC2Init(FET_MASTER_TIMER, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(FET_MASTER_TIMER, TIM_OCPreload_Enable);

    // Phase B
    TIM_OC3Init(FET_MASTER_TIMER, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(FET_MASTER_TIMER, TIM_OCPreload_Enable);

    // Phase C
    TIM_OC4Init(FET_MASTER_TIMER, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(FET_MASTER_TIMER, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(FET_MASTER_TIMER, ENABLE);

    // now setup the FET driver PWM timer (slave)
    FET_H_TIMER_REMAP;


	//////////////////////////////////////////////////////////////////////////
	//配置timer4
	//中央对齐模式，TIMx_ARR寄存器被装入缓冲器，
	//CC1、CC2、CC3为输出模式，输出比较预装载使能，PWM模式1
	// setup HI side
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = fetDutyCycle;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // Phase A
    TIM_OC1Init(FET_H_TIMER, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(FET_H_TIMER, TIM_OCPreload_Enable);

    // Phase B
    TIM_OC2Init(FET_H_TIMER, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(FET_H_TIMER, TIM_OCPreload_Enable);

    // Phase C
    TIM_OC3Init(FET_H_TIMER, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(FET_H_TIMER, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(FET_H_TIMER, ENABLE);

	//开启时钟
    TIM_Cmd(FET_H_TIMER, ENABLE);     //开启TIMER4模块
    TIM_Cmd(FET_MASTER_TIMER, ENABLE);//开启TIMER3模块


    FET_H_TIMER->CNT = 0;
    FET_MASTER_TIMER->CNT = 0;


	//////////////////////////////////////////////////////////////////////////
	//设置timer4和timer3的GPIO
    // now set AF mode for the high side gates
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // A GPIOB_6
    GPIO_InitStructure.GPIO_Pin = FET_A_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FET_A_H_PORT, &GPIO_InitStructure);

    // B GPIOB_7
    GPIO_InitStructure.GPIO_Pin = FET_B_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FET_B_H_PORT, &GPIO_InitStructure);

    // C GPIOB_8
    GPIO_InitStructure.GPIO_Pin = FET_C_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FET_C_H_PORT, &GPIO_InitStructure);

    // shut 'em down!
    fetSetStep(0);

	//fetSelfTest();
	//fetTest();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//fet 未换向
static void fetMissedCommutate(int period) {
    int32_t newPeriod;

    // commutate
    fetSetStep(fetNextStep);

    newPeriod = period + period/4;
    if (newPeriod > 0xffff/TIMER_MULT)
		newPeriod = 0xffff/TIMER_MULT;
    timerSetAlarm2(newPeriod, fetMissedCommutate, period);
}

//fet 换向 adc中断函数中调用
void fetCommutate(int period) 
{
    // keep count of in order ZC detections
    if (fetStep == fetNextStep) 
	{
		timerCancelAlarm2();

		// commutate
		fetSetStep(fetStep);

		fetGoodDetects++;
		if (fetGoodDetects >= 6)
			fetBadDetects = 0;

		// in case of missed zc
		if (state == ESC_STATE_RUNNING)
			timerSetAlarm2(period + period/2, fetMissedCommutate, period);
		else if (state == ESC_STATE_STARTING)
	//	    timerSetAlarm2(period*2, fetMissedCommutate, period*2);
			timerSetAlarm2(period + period/2, fetMissedCommutate, period);
    }
    else {
		//检测到错误
		fetBadDetects++;
		fetTotalBadDetects++;
		fetGoodDetects = 0;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// initiates motor start sequence   初始化电机启动序列
void motorStartSeqInit(void) {
    // set globals to start position
    startSeqCnt = 0;
    startSeqStp = fetNextStep;

    // set first step
    fetSetBraking(0);
    fetSetStep(startSeqStp);

    // Start sequence will run Without commutation.
    state = ESC_STATE_NOCOMM;

    // start "motorStartSeq"
    timerSetAlarm2(0, motorStartSeq, 0);
}

//电机启动
void fetStartCommutation(uint8_t startStep) 
{
    fetSetBraking(0);
//  启动周期     = 启动的电压量     / 电池电压(12V) * PWM脉冲周期
	fetStartDuty = p[START_VOLTAGE] / avgVolts * fetPeriod;


	adcSetCrossingPeriod(adcMaxPeriod/2);

	detectedCrossing = timerMicros;

	fetDutyCycle = fetStartDuty;
    _fetSetDutyCycle(fetDutyCycle);

	adcMaxAmps = 0;
    fetGoodDetects = 0;
    fetBadDetects = 0;
    fetTotalBadDetects = 0;
    fetNextStep = startStep;

    // start
    timerSetAlarm2(0, fetMissedCommutate, crossingPeriod);
}

// 电机在启动的时候执行
// generates motor start sequence
static void motorStartSeq(int period) {
	int nextPeriod;

	// Static field to align rotor. Without commutation.
	if (startSeqCnt < p[START_ALIGN_TIME]) 
	{
		// 这里启动的时候.没有做电机换向
		// PWM ramp up
		// 启动周期  =                          (这里是个斜坡) / 电池电压(12V) * 周期
		// 启动的时候占空比慢慢增加 ... 做一个斜坡
		fetStartDuty = p[START_ALIGN_VOLTAGE] * ((float)startSeqCnt / p[START_ALIGN_TIME]) / avgVolts * fetPeriod;
		fetDutyCycle = fetStartDuty;
		_fetSetDutyCycle(fetDutyCycle);

		// Prepare next function call
		period     = 1000; // 1 ms
		nextPeriod = 1000;
		timerSetAlarm2(period, motorStartSeq, nextPeriod);
	}
	// Rotating field with optional acceleration but without commutation.
	else if (startSeqCnt < (p[START_ALIGN_TIME] + p[START_STEPS_NUM])) 
	{
		// One time if entering "Rotating field"
		if (startSeqCnt == p[START_ALIGN_TIME])
			period = p[START_STEPS_PERIOD];

		// 电击换相
		// Set next step
		startSeqStp++;
		if (startSeqStp > 6) 
			startSeqStp = 1;
		fetSetStep(startSeqStp);

		// Set PWM
		// 运行频率  =                  / 电池电压(12V) * 周期
		fetStartDuty = p[START_VOLTAGE] / avgVolts * fetPeriod;
		fetDutyCycle = fetStartDuty;
		_fetSetDutyCycle(fetDutyCycle);

		// Prepare next function call
		nextPeriod = period - p[START_STEPS_ACCEL];
		if (nextPeriod < p[MIN_PERIOD])
			nextPeriod = p[MIN_PERIOD]; // avoid negative period
		timerSetAlarm2(period, motorStartSeq, nextPeriod);
	}
	// Continue normal startup with commutation
	else {
		//电机运行了
		// allow commutation
		state = ESC_STATE_STARTING;

		// Count next step (for commutation)
		startSeqStp++;
		if (startSeqStp > 6)
			startSeqStp = 1;

		// start
		fetStartCommutation(startSeqStp);
	}

	// count up step of startup sequence
	startSeqCnt++;
}

#if 0
void fetTest(void) {
	fetSetStep(1);

    //__asm volatile ("cpsid f");
	//CPSID_F();
	__disable_fault_irq();
/*
    // pulse phase A HI FET
    FET_A_H_PORT->BSRR = AH_ON;
    timerDelay(5);
    FET_A_H_PORT->BSRR = AH_OFF;
    timerDelay(20);
    FET_A_H_PORT->BSRR = AH_ON;
    timerDelay(5);
    FET_A_H_PORT->BSRR = AH_OFF;
    timerDelay(5);
    FET_A_H_PORT->BSRR = AH_ON;
    timerDelay(10);
    // leave in inactive state
    FET_A_H_PORT->BSRR = AH_OFF;
*/
//    *AH_BITBAND = 0;

    while (1) {
		FET_A_L_PORT->BSRR = AL_ON;
		FET_B_L_PORT->BSRR = BL_ON;
		FET_C_L_PORT->BSRR = CL_ON;

		timerDelay(10);

		FET_A_L_PORT->BSRR = AL_OFF;
		FET_B_L_PORT->BSRR = BL_OFF;
		FET_C_L_PORT->BSRR = CL_OFF;

		timerDelay(10);
    }

    FET_H_TIMER->FET_A_H_CHANNEL = 200;

    while(1)
		;

    //__asm volatile ("cpsie i");
	//CPSIE_I();
	__enable_irq();
}
#endif

void fetSetConstants(void) {
    float switchFreq = p[SWITCH_FREQ];             //PWM周期 默认20Khz
    float startVoltage = p[START_VOLTAGE];
    float startDetects = p[GOOD_DETECTS_START];
    float disarmDetects = p[BAD_DETECTS_DISARM];
    float fetBraking = p[FET_BRAKING];             //是否开启制动模式
    float servoMaxRate = p[SERVO_MAX_RATE];

    // bounds checking
    if (switchFreq > FET_MAX_SWITCH_FREQ)
		switchFreq = FET_MAX_SWITCH_FREQ;
    else if (switchFreq < FET_MIN_SWITCH_FREQ)
		switchFreq = FET_MIN_SWITCH_FREQ;

    if (startVoltage > FET_MAX_START_VOLTAGE)
		startVoltage = FET_MAX_START_VOLTAGE;
    else if (startVoltage < FET_MIN_START_VOLTAGE)
		startVoltage = FET_MIN_START_VOLTAGE;

    if (startDetects > FET_MAX_START_DETECTS)
		startDetects = FET_MAX_START_DETECTS;
    else if (startDetects < FET_MIN_START_DETECTS)
		startDetects = FET_MIN_START_DETECTS;

    if (disarmDetects > FET_MAX_DISARM_DETECTS)
		disarmDetects = FET_MAX_DISARM_DETECTS;
    else if (disarmDetects < FET_MIN_DISARM_DETECTS)
		disarmDetects = FET_MIN_DISARM_DETECTS;

    if (fetBraking > 0.0f)
		fetBraking = 1.0f;
    else
		fetBraking = 0.0f;

    if (servoMaxRate <= 0.0f)
		servoMaxRate = 360.0f;

	//计算CPU定时器寄存器分频.
    fetSwitchFreq = switchFreq * 1000 * 2;      // 设定的频率*1000 转换为KHz 
    fetPeriod = FET_AHB_FREQ/fetSwitchFreq;     // bus speed / switching frequency - depends on fetSwitchFreq
    fetSetBaseTime(fetPeriod);

    fetStartDetects = startDetects;
    fetDisarmDetects = disarmDetects;
    fetBrakingEnabled = (int8_t)fetBraking;
    fetServoMaxRate = servoMaxRate / 1000.0f * p[MOTOR_POLES] * 0.5f;

    p[SWITCH_FREQ] = switchFreq;
    p[START_VOLTAGE] = startVoltage;
    p[GOOD_DETECTS_START] = startDetects;
    p[BAD_DETECTS_DISARM] = disarmDetects;
    p[FET_BRAKING] = fetBraking;
    p[SERVO_MAX_RATE] = servoMaxRate;

    fetCreateSine();
}
