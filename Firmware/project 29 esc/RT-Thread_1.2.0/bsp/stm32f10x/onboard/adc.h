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

#ifndef _ADC_H
#define _ADC_H

#include "timer.h"
#include "stm32f10x_adc.h"

#define ADC_FAST_SAMPLE

#ifdef ADC_FAST_SAMPLE
    #define ADC_SAMPLE_TIME	ADC_SampleTime_7Cycles5
    #define ADC_DETECTION_TIME	(uint16_t)((7.5+12.5)*4*TIMER_MULT/12)	    // 4 ADC groups w/7.5 clk sample @ 12Mhz ADC clock (in us)
#else
    #define ADC_SAMPLE_TIME	ADC_SampleTime_28Cycles5
    #define ADC_DETECTION_TIME	(uint16_t)((28.5+12.5)*2*TIMER_MULT/12)	    // 2 ADC groups w/28.5 clk sample @ 12Mhz ADC clock (in us)
#endif	// ADC_FAST_SAMPLE

#define ADC_CHANNELS            2
#define ADC_CLOCK               RCC_PCLK2_Div6              // 12Mhz

#define ADC_REF_VOLTAGE         3.3f
#define ADC_TO_VOLTAGE		(ADC_REF_VOLTAGE / (1<<12)) // 12 bit ADC resolution

#define ADC_AMPS_PRECISION	16
#define ADC_SHUNT_GAIN		50.9f

#define ADC_VOLTS_PRECISION	16
#define ADC_VOLTS_SLOPE		((10.0f + 1.5f) / 1.5f)    // Rtop = 10K, Rbot = 1.5K
#define ADC_TO_VOLTS		(ADC_TO_VOLTAGE * ADC_VOLTS_SLOPE / ((1<<(ADC_VOLTS_PRECISION))+1))

#define ADC_MIN_SHUNT		0.05	    // milli Ohms
#define ADC_MAX_SHUNT		1.0	    // milli Ohms
#define ADC_MIN_ADVANCE		0.1	    // electrical degrees
#define ADC_MAX_ADVANCE		30.0	    // electrical degrees
#define ADC_MIN_BLANKING_MICROS	0	    // us
#define ADC_MAX_BLANKING_MICROS 100	    // us
#define ADC_MIN_MIN_PERIOD	20	    // us
#define ADC_MAX_MIN_PERIOD	500	    // us
#define ADC_MIN_MAX_PERIOD	1000	    // us
#define ADC_MAX_MAX_PERIOD	20000	    // us

#define ADC_HIST_SIZE		64
#ifdef ADC_FAST_SAMPLE
#define ADC_MIN_COMP		30
#else
#define ADC_MIN_COMP		15
#endif
#define ADC_CROSSING_TIMEOUT	(250000*TIMER_MULT)

//#define ADC_COMMUTATION_ADVANCE	(0)				    // 0 deg
//#define ADC_COMMUTATION_ADVANCE	(crossingPeriod/16)		    // 3.75 deg
//#define ADC_COMMUTATION_ADVANCE	(crossingPeriod/8)		    // 7.5 deg
//#define ADC_COMMUTATION_ADVANCE	(crossingPeriod/4)		    // 15 deg
//#define ADC_COMMUTATION_ADVANCE	(crossingPeriod/2)		    // 30 deg
#define ADC_COMMUTATION_ADVANCE	(crossingPeriod/adcAdvance)		    // variable

extern float adcToAmps;
extern int16_t adcAdvance;
extern int32_t adcblankingMicros;
extern int32_t adcMaxPeriod;
extern int32_t adcMinPeriod;
extern int32_t adcAmpsOffset;
extern volatile int32_t adcAvgAmps;
extern volatile int32_t adcMaxAmps;
extern volatile int32_t adcAvgVolts;
extern int16_t histSize;
extern uint32_t avgA, avgB, avgC;
extern volatile uint32_t detectedCrossing;
extern volatile uint32_t crossingPeriod;
extern volatile int32_t adcCrossingPeriod;

extern void adcInit(void);
extern void adcSetConstants(void);
extern void adcSetCrossingPeriod(int32_t crossPer);
extern int32_t adcGetInstantCurrent(void);

#endif
