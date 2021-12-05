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
*/

uint32_t AH[7] = {  0,      1,      0,      0,      0,      0,      1};
uint32_t AL[7] = {AL_OFF, AL_OFF, AL_OFF, AL_ON,  AL_ON,  AL_OFF, AL_OFF};
uint32_t BH[7] = {  0,      0,      0,      0,      1,      1,      0};
uint32_t BL[7] = {BL_ON,  BL_ON,  BL_ON,  BL_OFF, BL_OFF, BL_OFF, BL_OFF};	// note: step 0 has B_LO energized
uint32_t CH[7] = {  0,      0,      1,      1,      0,      0,      0};
uint32_t CL[7] = {CL_OFF, CL_OFF, CL_OFF, CL_OFF, CL_OFF, CL_ON,  CL_ON};

int32_t fetSwitchFreq;
int32_t fetStartDuty;
int16_t fetStartDetects;
int16_t fetDisarmDetects;
int32_t fetPeriod;
int32_t fetActualDutyCycle;
volatile int32_t fetDutyCycle;
volatile uint8_t fetStep;
volatile int8_t fetNextStep;
volatile uint32_t fetGoodDetects;
volatile uint32_t fetBadDetects;
volatile uint32_t fetTotalBadDetects;
volatile uint32_t fetCommutationMicros;
int8_t fetBrakingEnabled;
int8_t fetBraking;
int16_t startSeqCnt;
int8_t fetStepDir;
float fetServoAngle;
float fetServoMaxRate;

int16_t fetSine[FET_SERVO_RESOLUTION];

void fetCreateSine(void) {
    float a;
    int i;

    for (i = 0; i < FET_SERVO_RESOLUTION; i++) {
	a = M_PI * 2.0f * i / FET_SERVO_RESOLUTION;

	// third order harmonic injection
	fetSine[i] = (sinf(a) + sinf(a*3.0f)/6.0f) * (2.0f/sqrtf(3.0f)) * (float)fetPeriod / 2.0f;
    }
}

void _fetSetServoDuty(uint16_t duty[3]) {
    FET_H_TIMER->FET_A_H_CHANNEL = duty[0];
    FET_H_TIMER->FET_B_H_CHANNEL = duty[1];
    FET_H_TIMER->FET_C_H_CHANNEL = duty[2];

    FET_MASTER_TIMER->FET_A_L_CHANNEL = duty[0] + FET_DEADTIME;
    FET_MASTER_TIMER->FET_B_L_CHANNEL = duty[1] + FET_DEADTIME;
    FET_MASTER_TIMER->FET_C_L_CHANNEL = duty[2] + FET_DEADTIME;
}

void fetUpdateServo(void) {
    static float myAngle = 0.0f;
    static float servoDState = 0.0f;
    float a, e;

    uint16_t pwm[3];
    int index;

    if (state == ESC_STATE_RUNNING) {
	*AL_BITBAND = 1;
	*BL_BITBAND = 1;
	*CL_BITBAND = 1;

	*AH_BITBAND = 1;
	*BH_BITBAND = 1;
	*CH_BITBAND = 1;

	e = (fetServoAngle - myAngle);
	a = e * p[SERVO_P];
	if (a > fetServoMaxRate)
	    a = fetServoMaxRate;
	else if (a < -fetServoMaxRate)
	    a = -fetServoMaxRate;

	myAngle += a;

	myAngle += (a - servoDState) * p[SERVO_D];
	servoDState = a;

	index = ((float)FET_SERVO_RESOLUTION * myAngle / 360.0f);
	while (index < 0)
	    index += FET_SERVO_RESOLUTION;
	index = index % FET_SERVO_RESOLUTION;

	pwm[0] = fetPeriod/2 + fetSine[index] * p[SERVO_DUTY] / 100;

	index = ((index + FET_SERVO_RESOLUTION / 3) % FET_SERVO_RESOLUTION);
	pwm[1] = fetPeriod/2 + fetSine[index] * p[SERVO_DUTY] / 100;

	index = ((index + FET_SERVO_RESOLUTION / 3) % FET_SERVO_RESOLUTION);
	pwm[2] = fetPeriod/2 + fetSine[index] * p[SERVO_DUTY] / 100;

	_fetSetServoDuty(pwm);
    }
    else {
	*AL_BITBAND = 0;
	*BL_BITBAND = 0;
	*CL_BITBAND = 0;

	*AH_BITBAND = 0;
	*BH_BITBAND = 0;
	*CH_BITBAND = 0;
    }
}

void fetSetAngleFromPwm(int32_t pwm) {
    fetServoAngle = pwm * p[SERVO_SCALE] * p[MOTOR_POLES] * 0.5f / fetPeriod;
}

void fetSetAngle(float angle) {
    fetServoAngle = angle * p[MOTOR_POLES] * 0.5f;
}

#define FET_TEST_DELAY	1000

uint8_t fetSelfTest(void) {
    int32_t baseCurrent;
    int32_t cl1, cl2, cl3;
    int32_t ch1, ch2, ch3;

    // must be disarmed to run self test
    if (state != ESC_STATE_DISARMED)
	return FET_TEST_NOT_RUN;

    fetSetStep(0);

    // shut everything off
    FET_A_L_OFF;
    FET_B_L_OFF;
    FET_C_L_OFF;

    FET_A_H_OFF;
    FET_B_H_OFF;
    FET_C_H_OFF;

    // record base current
    timerDelay(FET_TEST_DELAY);
    baseCurrent = adcAvgAmps;

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

// this assume that one low FET is conducting (s/b B)
void fetBeep(uint16_t freq, uint16_t duration) {
    uint16_t prevReloadVal;
    int i;

    fetSetStep(0);

    prevReloadVal = runIWDGInit(999);

    __asm volatile ("cpsid i");

    for (i = 0; i < duration; i++) {
	// reload the hardware watchdog
	runFeedIWDG();

	FET_A_H_ON;
	timerDelay(8);
	FET_A_H_OFF;

	timerDelay(freq);

	// reload the hardware watchdog
	runFeedIWDG();

	FET_C_L_ON;
	timerDelay(8);
	FET_C_H_OFF;

	timerDelay(freq);
    }

    __asm volatile ("cpsie i");

    runIWDGInit(prevReloadVal);
}

void fetSetBraking(int8_t value) {
    if (value) {
	// set low side for inverted PWM
//	*AL_BITBAND = AH[fetStep];
//	*BL_BITBAND = BH[fetStep];
//	*CL_BITBAND = CH[fetStep];
	*AL_BITBAND = 1;
	*BL_BITBAND = 1;
	*CL_BITBAND = 1;
	fetBraking = 1;
    }
    else {
	// switch off PWM
	*AL_BITBAND = 0;
	*BL_BITBAND = 0;
	*CL_BITBAND = 0;
	fetBraking = 0;
    }
}

void _fetSetDutyCycle(int32_t dutyCycle) {
    register int32_t tmp;

    tmp = dutyCycle;

    if (state == ESC_STATE_DISARMED)
	tmp = 0;

    FET_H_TIMER->FET_A_H_CHANNEL = tmp;
    FET_H_TIMER->FET_B_H_CHANNEL = tmp;
    FET_H_TIMER->FET_C_H_CHANNEL = tmp;

    if (fetBrakingEnabled) {
	tmp = dutyCycle + fetPeriod / 8;

	if (tmp < 0)
	    tmp = 0;
	else if (tmp > fetPeriod)
	    tmp = fetPeriod;

	FET_MASTER_TIMER->FET_A_L_CHANNEL = tmp;
	FET_MASTER_TIMER->FET_B_L_CHANNEL = tmp;
	FET_MASTER_TIMER->FET_C_L_CHANNEL = tmp;
    }
}

void fetSetDutyCycle(int32_t requestedDutyCycle) {
    if (requestedDutyCycle > fetPeriod)
	requestedDutyCycle = fetPeriod;
    else if (requestedDutyCycle < 0)
	requestedDutyCycle = 0;

    fetDutyCycle = requestedDutyCycle;
}

static inline int8_t fetGetNextStep(int8_t step) {
    int8_t nextStep;

    nextStep = step + fetStepDir;
    if (nextStep > 6)
	nextStep = 1;
    else if (nextStep < 1)
	nextStep = 6;

    return nextStep;
}

//
// Low side FET switching is done via direct GPIO manipulation
// High side FET switching is accomplished by enabling or disabling
// control of the output pin by the PWM timer.  When disabled, the
// GPIO output state is imposed on the pin (FET off.)
//
void fetSetStep(int n) {
    __asm volatile ("cpsid i");
    fetCommutationMicros = timerGetMicros();
    __asm volatile ("cpsie i");

    fetNextStep = fetGetNextStep(n);

    // set high side
    *AH_BITBAND = AH[n];
    *BH_BITBAND = BH[n];
    *CH_BITBAND = CH[n];

    if (fetBrakingEnabled)
	fetSetBraking(fetBraking);

    // set low side
    FET_A_L_PORT->BSRR = AL[n];
    FET_B_L_PORT->BSRR = BL[n];
    FET_C_L_PORT->BSRR = CL[n];
}

void fetSetBaseTime(int32_t period) {
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
    NVIC_InitTypeDef NVIC_InitStructure;

    fetSetConstants();

    fetDutyCycle = fetPeriod;
    fetStep = 0;

    // setup low side gates
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // A
    GPIO_InitStructure.GPIO_Pin = FET_A_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_A_L_PORT, &GPIO_InitStructure);

    // B
    GPIO_InitStructure.GPIO_Pin = FET_B_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_B_L_PORT, &GPIO_InitStructure);

    // C
    GPIO_InitStructure.GPIO_Pin = FET_C_L_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_C_L_PORT, &GPIO_InitStructure);

    // setup GPIO default output states for high side

    // A
    GPIO_InitStructure.GPIO_Pin = FET_A_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_A_H_PORT, &GPIO_InitStructure);

    // B
    GPIO_InitStructure.GPIO_Pin = FET_B_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_B_H_PORT, &GPIO_InitStructure);

    // C
    GPIO_InitStructure.GPIO_Pin = FET_C_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(FET_C_H_PORT, &GPIO_InitStructure);

    // high side N-FET is inactive low
    FET_A_H_PORT->BSRR = AH_OFF;
    FET_B_H_PORT->BSRR = BH_OFF;
    FET_C_H_PORT->BSRR = CH_OFF;

    // allow FET PWM (slave) to run during core halt
    DBGMCU_Config(FET_DBGMCU_STOP, ENABLE);

    // stop MASTER during core halt
    DBGMCU_Config(FET_MASTER_DBGMCU_STOP, ENABLE);

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

    TIM_Cmd(FET_H_TIMER, ENABLE);
    TIM_Cmd(FET_MASTER_TIMER, ENABLE);

    FET_H_TIMER->CNT = 0;
    FET_MASTER_TIMER->CNT = 0;

    // now set AF mode for the high side gates
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // A
    GPIO_InitStructure.GPIO_Pin = FET_A_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FET_A_H_PORT, &GPIO_InitStructure);

    // B
    GPIO_InitStructure.GPIO_Pin = FET_B_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FET_B_H_PORT, &GPIO_InitStructure);

    // C
    GPIO_InitStructure.GPIO_Pin = FET_C_H_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FET_C_H_PORT, &GPIO_InitStructure);

    // shut 'em down!
    fetSetStep(0);

//    fetSelfTest();
//    fetTest();
}

void fetMissedCommutate(int period) {
    int32_t newPeriod;

    // commutate
    fetSetStep(fetNextStep);

    newPeriod = period + period/4;
    if (newPeriod > 0xffff/TIMER_MULT)
	newPeriod = 0xffff/TIMER_MULT;
    timerSetAlarm2(newPeriod, fetMissedCommutate, period);
}

void fetCommutate(int period) {
    if (state != ESC_STATE_NOCOMM) {
	// keep count of in order ZC detections
	if (fetStep == fetNextStep) {
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
//		timerSetAlarm2(period*2, fetMissedCommutate, period*2);
		timerSetAlarm2(period + period/2, fetMissedCommutate, period);
	}
	else {
	    fetBadDetects++;
	    fetTotalBadDetects++;
	    fetGoodDetects = 0;
	}
    }
}

// initiates motor start sequence
void motorStartSeqInit(void) {
    // set globals to start position
    startSeqCnt = 0;

    // set first step
    fetSetBraking(0);
    fetSetStep(fetNextStep);

    // Start sequence will run Without commutation.
    state = ESC_STATE_NOCOMM;

    // start "motorStartSeq"
    timerSetAlarm2(0, motorStartSeq, 0);
}

void fetStartCommutation(uint8_t startStep) {
    fetSetBraking(0);
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

// generates motor start sequence
void motorStartSeq(int period) {
    int nextPeriod;

    // Static field to align rotor. Without commutation.
    if (startSeqCnt < p[START_ALIGN_TIME]) {
	// PWM ramp up
	fetStartDuty = p[START_ALIGN_VOLTAGE] * ((float)startSeqCnt / p[START_ALIGN_TIME]) / avgVolts * fetPeriod;
	fetDutyCycle = fetStartDuty;
	_fetSetDutyCycle(fetDutyCycle);

	// Prepare next function call
	period     = 1000 * TIMER_MULT; // 1 ms
	nextPeriod = 1000 * TIMER_MULT;
	timerSetAlarm2(period, motorStartSeq, nextPeriod);
    }
    // Rotating field with optional acceleration but without commutation.
    else if (startSeqCnt < (p[START_ALIGN_TIME] + p[START_STEPS_NUM])) {
	// One time if entering "Rotating field"
	if (startSeqCnt == p[START_ALIGN_TIME]) {
	    period = p[MAX_PERIOD] * TIMER_MULT;
	    detectedCrossing = timerMicros;
	}

	fetSetStep(fetNextStep);

	// Set PWM
	fetStartDuty = p[START_VOLTAGE] / avgVolts * fetPeriod;
	fetDutyCycle = fetStartDuty;
	_fetSetDutyCycle(fetDutyCycle);

	// Prepare next function call
	nextPeriod = period - (p[START_STEPS_ACCEL] * TIMER_MULT);
	if (nextPeriod < p[START_STEPS_PERIOD] * TIMER_MULT)
	    nextPeriod = crossingPeriod;

	timerSetAlarm2(period, motorStartSeq, nextPeriod);
    }
    else {
	// let motor run
	if (p[START_STEPS_NUM]) {
	    adcMaxAmps = 0;
	    fetGoodDetects = 0;
	    fetBadDetects = 0;
	    fetTotalBadDetects = 0;

	    // last one
	    fetSetStep(fetNextStep);

	    // cancel any existing ZC detection
	    timerCancelAlarm1();

	    // allow normal commutation
	    state = ESC_STATE_STARTING;
	}
	// Continue normal startup with commutation
	else {
	    // allow normal commutation
	    state = ESC_STATE_STARTING;

	    fetStartCommutation(fetNextStep);
	}
    }

    // count up step of startup sequence
    startSeqCnt++;
}

void fetTest(void) {
    fetSetStep(1);

    __asm volatile ("cpsid f");
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

    __asm volatile ("cpsie i");
}

void fetSetConstants(void) {
    float switchFreq = p[SWITCH_FREQ];
    float startVoltage = p[START_VOLTAGE];
    float startDetects = p[GOOD_DETECTS_START];
    float disarmDetects = p[BAD_DETECTS_DISARM];
    float fetBraking = p[FET_BRAKING];
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

    fetSwitchFreq = switchFreq * 1000 * 2;
    fetPeriod = FET_AHB_FREQ/fetSwitchFreq;     // bus speed / switching frequency - depends on fetSwitchFreq
    fetSetBaseTime(fetPeriod);

    fetStartDetects = startDetects;
    fetDisarmDetects = disarmDetects;
    fetBrakingEnabled = (int8_t)fetBraking;
    fetServoMaxRate = servoMaxRate / RUN_FREQ * p[MOTOR_POLES] * 0.5f;

    if (p[DIRECTION] >= 0)
	fetStepDir = 1;
    else
	fetStepDir = -1;

    p[SWITCH_FREQ] = switchFreq;
    p[START_VOLTAGE] = startVoltage;
    p[GOOD_DETECTS_START] = startDetects;
    p[BAD_DETECTS_DISARM] = disarmDetects;
    p[FET_BRAKING] = fetBraking;
    p[SERVO_MAX_RATE] = servoMaxRate;
    p[DIRECTION] = fetStepDir;

    fetCreateSine();
}