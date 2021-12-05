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
#include "adc.h"
#include "fet.h"
#include "run.h"
#include "digital.h"
#include "timer.h"
#include "config.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "misc.h"

#ifdef ADC_FAST_SAMPLE
uint32_t adcRawData[ADC_CHANNELS*4];
#else
uint32_t adcRawData[ADC_CHANNELS*2];
#endif

float adcToAmps;
int16_t adcAdvance;
int32_t adcblankingMicros;
int32_t adcMaxPeriod;
int32_t adcMinPeriod;

int16_t histIndex;
int16_t histSize;
uint16_t histA[ADC_HIST_SIZE];
uint16_t histB[ADC_HIST_SIZE];
uint16_t histC[ADC_HIST_SIZE];

uint32_t avgA, avgB, avgC;
int32_t adcAmpsOffset;
volatile int32_t adcAvgAmps;
volatile int32_t adcMaxAmps;
volatile int32_t adcAvgVolts;

uint8_t adcStateA, adcStateB, adcStateC;

volatile uint32_t detectedCrossing;
volatile uint32_t crossingPeriod;
volatile int32_t adcCrossingPeriod;
uint32_t nextCrossingDetect;
uint32_t numLoops;

void adcCalibrateADC(ADC_TypeDef *ADCx) {
    // Enable ADC reset calibration register
    ADC_ResetCalibration(ADCx);

    // Check the end of ADC reset calibration register
    while(ADC_GetResetCalibrationStatus(ADCx))
	;

    // Start ADC calibration
    ADC_StartCalibration(ADCx);

    // Check the end of ADC calibration
    while(ADC_GetCalibrationStatus(ADCx))
	;
}

void adcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    int i;

    adcSetConstants();
    histSize = ADC_HIST_SIZE;

    // Use STM32's Dual Regular Simultaneous Mode capable of ~ 1.7M samples per second

    // NOTE: assume that RCC code has already placed all pins into Analog In mode during startup

    // DMA1 channel1 configuration (ADC1)
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1 + 0x4c;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcRawData[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = sizeof(adcRawData)/4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
    DMA_ClearITPendingBit(DMA1_IT_GL1 | DMA1_IT_TC1 | DMA1_IT_HT1);

    DMA_Cmd(DMA1_Channel1, ENABLE);

    // Enable the DMA1_Channel1 global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ADC1 configuration
//    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = sizeof(adcRawData)/4;
    ADC_Init(ADC1, &ADC_InitStructure);

#ifdef ADC_FAST_SAMPLE
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SAMPLE_TIME);	// SENSE_CURRENT
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SAMPLE_TIME);	// SENSE_CURRENT
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SAMPLE_TIME);	// SENSE_B
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 4, ADC_SAMPLE_TIME);	// SENSE_B
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SAMPLE_TIME);	// SENSE_VIN
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 6, ADC_SAMPLE_TIME);	// SENSE_VIN
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 7, ADC_SAMPLE_TIME);	// SENSE_B
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 8, ADC_SAMPLE_TIME);	// SENSE_B
#else
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SAMPLE_TIME);	// SENSE_CURRENT
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SAMPLE_TIME);	// SENSE_B
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SAMPLE_TIME);	// SENSE_VIN
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 4, ADC_SAMPLE_TIME);	// SENSE_B
#endif
    ADC_DMACmd(ADC1, ENABLE);

    // ADC2 configuration
//    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = sizeof(adcRawData)/4;
    ADC_Init(ADC2, &ADC_InitStructure);

#ifdef ADC_FAST_SAMPLE
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SAMPLE_TIME);	// SENSE_A
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SAMPLE_TIME);	// SENSE_A
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 3, ADC_SAMPLE_TIME);	// SENSE_C
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 4, ADC_SAMPLE_TIME);	// SENSE_C
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 5, ADC_SAMPLE_TIME);	// SENSE_A
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 6, ADC_SAMPLE_TIME);	// SENSE_A
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 7, ADC_SAMPLE_TIME);	// SENSE_C
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 8, ADC_SAMPLE_TIME);	// SENSE_C
#else
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SAMPLE_TIME);	// SENSE_A
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 2, ADC_SAMPLE_TIME);	// SENSE_C
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 3, ADC_SAMPLE_TIME);	// SENSE_A
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 4, ADC_SAMPLE_TIME);	// SENSE_C
#endif

    ADC_ExternalTrigConvCmd(ADC2, ENABLE);

    // enable and calibrate
    ADC_Cmd(ADC1, ENABLE);
    adcCalibrateADC(ADC1);

    ADC_Cmd(ADC2, ENABLE);
    adcCalibrateADC(ADC2);

    nextCrossingDetect = adcMaxPeriod;

    // setup injection sequence
    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedSequencerLengthConfig(ADC2, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SAMPLE_TIME);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SAMPLE_TIME);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);

    // Start ADC1 / ADC2 Conversions
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void adcSetCrossingPeriod(int32_t crossPer) {
    adcCrossingPeriod = crossPer<<15;
    crossingPeriod = crossPer;
}

static inline void adcGrowHist(void) {
    register int i;

    avgA += histA[histIndex];
    avgB += histB[histIndex];
    avgC += histC[histIndex];

    for (i = histSize; i > histIndex; i--) {
	histA[i] = histA[i-1];
	histB[i] = histB[i-1];
	histC[i] = histC[i-1];
    }

    histSize++;
}

static inline void adcShrinkHist(void) {
    register int i;

    for (i = histIndex; i < histSize-1; i++) {
	histA[i] = histA[i+1];
	histB[i] = histB[i+1];
	histC[i] = histC[i+1];
    }

    histSize--;

    if (histIndex == histSize)
	histIndex = 0;

    avgA -= histA[histIndex];
    avgB -= histB[histIndex];
    avgC -= histC[histIndex];
}

static inline void adcEvaluateHistSize(void) {
    int16_t sizeNeeded;

//  sizeNeeded = crossingPeriod/16/TIMER_MULT;
//    sizeNeeded = crossingPeriod/20/TIMER_MULT;
//  sizeNeeded = crossingPeriod/24/TIMER_MULT;
    sizeNeeded = crossingPeriod/32/TIMER_MULT;

    if (sizeNeeded > (histSize+1) && histSize < ADC_HIST_SIZE)
	adcGrowHist();
    else if (sizeNeeded < (histSize-1) && sizeNeeded > 1)
	adcShrinkHist();
}

#pragma GCC optimize ("-O1")
void DMA1_Channel1_IRQHandler(void) {
    register uint16_t *raw = (uint16_t *)adcRawData;
    register uint32_t valA, valB, valC, valCOMP;
    int ampsFlag = 0;
    uint32_t currentMicros;

    __asm volatile ("cpsid i");
    currentMicros = timerGetMicros();
    __asm volatile ("cpsie i");

#ifdef ADC_FAST_SAMPLE
    if ((DMA1->ISR & DMA1_FLAG_TC1) != RESET) {
        raw += (ADC_CHANNELS * 4);        // 4 16bit words each
	adcAvgVolts -= (adcAvgVolts - (int32_t)((raw[0]+raw[2])<<(ADC_VOLTS_PRECISION-1)))>>6;
    }
    else {
	adcAvgAmps -= (adcAvgAmps - (int32_t)((raw[0]+raw[2])<<(ADC_AMPS_PRECISION-1)))>>6;
    }
#else
    if ((DMA1->ISR & DMA1_FLAG_TC1) != RESET) {
        raw += (ADC_CHANNELS * 2);        // 2 16bit words each
	adcAvgVolts -= (adcAvgVolts - (int32_t)(raw[0]<<ADC_VOLTS_PRECISION))>>6;
    }
    else {
	adcAvgAmps -= (adcAvgAmps - (int32_t)(raw[0]<<ADC_AMPS_PRECISION))>>6;
    }
#endif

    DMA1->IFCR = DMA1_IT_GL1 | DMA1_IT_TC1 | DMA1_IT_HT1;

    if (runMode == SERVO_MODE)
	return;

    // blanking time after commutation
    if (!fetCommutationMicros || ((currentMicros >= fetCommutationMicros) ? (currentMicros - fetCommutationMicros) : (TIMER_MASK - fetCommutationMicros + currentMicros)) > adcblankingMicros) {
#ifdef ADC_FAST_SAMPLE
	histA[histIndex] = valA = (raw[1]+raw[3]);
	histB[histIndex] = valB = (raw[4]+raw[6]);
	histC[histIndex] = valC = (raw[5]+raw[7]);
#else
	histA[histIndex] = valA = raw[1];
	histB[histIndex] = valB = raw[2];
	histC[histIndex] = valC = raw[3];
#endif
	histIndex = (histIndex + 1) % histSize;

	avgA += valA - histA[histIndex];
	avgB += valB - histB[histIndex];
	avgC += valC - histC[histIndex];

	if ((avgA+avgB+avgC)/histSize > (ADC_MIN_COMP*3) && state != ESC_STATE_DISARMED) {
	    register int32_t periodMicros;

	    periodMicros = (currentMicros >= detectedCrossing) ? (currentMicros - detectedCrossing) : (TIMER_MASK - detectedCrossing + currentMicros);

	    if (periodMicros > nextCrossingDetect) {
		register int8_t nextStep = 0;

		if (!adcStateA && avgA >= (avgB+avgC)>>1) {
		    adcStateA = 1;
		    if (fetStepDir > 0)
			nextStep = 6;
		    else
			nextStep = 1;
		}
		else if (adcStateA && avgA <= (avgB+avgC)>>1) {
		    adcStateA = 0;
		    if (fetStepDir > 0)
			nextStep = 3;
		    else
			nextStep = 4;
		}
		else if (!adcStateB && avgB >= (avgA+avgC)>>1) {
		    adcStateB = 1;
		    if (fetStepDir > 0)
			nextStep = 4;
		    else
			nextStep = 5;
		}
		else if (adcStateB && avgB <= (avgA+avgC)>>1) {
		    adcStateB = 0;
		    if (fetStepDir > 0)
			nextStep = 1;
		    else
			nextStep = 2;
		}
		else if (!adcStateC && avgC >= (avgA+avgB)>>1) {
		    adcStateC = 1;
		    if (fetStepDir > 0)
			nextStep = 2;
		    else
			nextStep = 3;
		}
		else if (adcStateC && avgC <= (avgA+avgB)>>1) {
		    adcStateC = 0;
		    if (fetStepDir > 0)
			nextStep = 5;
		    else
			nextStep = 6;
		}

		if (nextStep && periodMicros > adcMinPeriod) {
		    if (periodMicros > adcMaxPeriod)
			periodMicros = adcMaxPeriod;

//		    crossingPeriod = (crossingPeriod*3 + periodMicros)/4;
//		    crossingPeriod = (crossingPeriod*5 + periodMicros)/6;
		    adcCrossingPeriod += ((periodMicros<<15) - adcCrossingPeriod)>>3;
		    crossingPeriod = adcCrossingPeriod>>15;
//		    adcCrossingPeriod += ((periodMicros<<15) - adcCrossingPeriod)>>4;
//		    crossingPeriod = adcCrossingPeriod>>15;
//		    crossingPeriod = (crossingPeriod*7 + periodMicros)/8;
//		    crossingPeriod = (crossingPeriod*15 + periodMicros)/16;

		    // schedule next commutation
		    fetStep = nextStep;
		    fetCommutationMicros = 0;
		    timerSetAlarm1(crossingPeriod/2 - (ADC_DETECTION_TIME*(histSize+2))/2 - ADC_COMMUTATION_ADVANCE, fetCommutate, crossingPeriod);

		    // record crossing time
		    detectedCrossing = currentMicros;

		    // resize history based on period
		    adcEvaluateHistSize();

		    // calculate next crossing detection time
    //		nextCrossingDetect = crossingPeriod*2/3;
		    nextCrossingDetect = crossingPeriod*3/4;
    //		nextCrossingDetect = crossingPeriod*6/8;

		    // record highest current draw for this run
		    if (adcAvgAmps > adcMaxAmps)
			adcMaxAmps = adcAvgAmps;
		}
	    }
	}
    }
}

// start injected conversion of current sensor
int32_t adcGetInstantCurrent(void) {
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != SET)
	;
    return (int32_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}

void adcSetConstants(void) {
    float shuntResistance = p[SHUNT_RESISTANCE];
    float advance = p[ADVANCE];
    float blankingMicros = p[BLANKING_MICROS];
    float minPeriod = p[MIN_PERIOD];
    float maxPeriod = p[MAX_PERIOD];

    // bounds checking
    if (shuntResistance > ADC_MAX_SHUNT)
	shuntResistance = ADC_MAX_SHUNT;
    else if (shuntResistance < ADC_MIN_SHUNT)
	shuntResistance = ADC_MIN_SHUNT;

    if (advance > ADC_MAX_ADVANCE)
	advance = ADC_MAX_ADVANCE;
    else if (advance < ADC_MIN_ADVANCE)
	advance = ADC_MIN_ADVANCE;

    if (blankingMicros > ADC_MAX_BLANKING_MICROS)
	blankingMicros = ADC_MAX_BLANKING_MICROS;
    else if (blankingMicros < ADC_MIN_BLANKING_MICROS)
	blankingMicros = ADC_MIN_BLANKING_MICROS;

    if (minPeriod > ADC_MAX_MIN_PERIOD)
	minPeriod = ADC_MAX_MIN_PERIOD;
    else if (minPeriod < ADC_MIN_MIN_PERIOD)
	minPeriod = ADC_MIN_MIN_PERIOD;

    if (maxPeriod > ADC_MAX_MAX_PERIOD)
	maxPeriod = ADC_MAX_MAX_PERIOD;
    else if (maxPeriod < ADC_MIN_MAX_PERIOD)
	maxPeriod = ADC_MIN_MAX_PERIOD;

    adcToAmps = ((ADC_TO_VOLTAGE / ((1<<(ADC_AMPS_PRECISION))+1)) / (ADC_SHUNT_GAIN * shuntResistance / 1000.0f));
    adcAdvance = 100.0f / (advance * (50.0f / 30.0f));
    adcblankingMicros = blankingMicros * TIMER_MULT;
    adcMinPeriod = minPeriod * TIMER_MULT;
    adcMaxPeriod = maxPeriod * TIMER_MULT;

    p[SHUNT_RESISTANCE] = shuntResistance;
    p[ADVANCE] = advance;
    p[BLANKING_MICROS] = blankingMicros;
    p[MIN_PERIOD] = minPeriod;
    p[MAX_PERIOD] = maxPeriod;
}
