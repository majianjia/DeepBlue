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
#include "stdint.h"

#ifdef ADC_FAST_SAMPLE
	static uint32_t adcRawData[ADC_CHANNELS*4]; //adc的采样值.使用dma方式采样.采样完成后自动放入这里
	//这个变量是32位的
	//依次存放    0             1             2       3       | 4         5         6       7
	//ADC1        SENSE_CURRENT SENSE_CURRENT SENSE_B SENSE_B | SENSE_VIN SENSE_VIN SENSE_B SENSE_B  低16位
	//ADC2        SENSE_A       SENSE_A       SENSE_C SENSE_C | SENSE_A   SENSE_A   SENSE_C SENSE_C  高16位
	//            这里的SENSE_A  SENSE_B  SENSE_C 没有使用    |
#else
	static uint32_t adcRawData[ADC_CHANNELS*2];
#endif

float adcToAmps;    //ADC的电流测量值 转换成 实际的电流公式
static int16_t adcAdvance;
static int32_t adcblankingMicros;
int32_t adcMaxPeriod;//ADC最大的周期
static int32_t adcMinPeriod;//ADC最小的换相时间(us)


static int16_t histIndex;  //数组的索引值
int16_t histSize;          //数组的大小
static uint16_t histA[ADC_HIST_SIZE];//adc转换后的值,保准在这个数组里
static uint16_t histB[ADC_HIST_SIZE];
static uint16_t histC[ADC_HIST_SIZE];

uint32_t avgA, avgB, avgC;    //电机A B C相的电压ADC采集.平均值
int32_t adcAmpsOffset;        //当前ADC采集转换后的 传感器电流 电流偏移 (在停止运行模式下,电流的值.做偏移值)
volatile int32_t adcAvgAmps;  //当前ADC采集转换后的 传感器电流
volatile int32_t adcMaxAmps;  //运行中最大 传感器电流
volatile int32_t adcAvgVolts; //运行的电压

static uint8_t adcStateA, adcStateB, adcStateC;//当前的状态

volatile uint32_t detectedCrossing;
volatile uint32_t crossingPeriod;
volatile int32_t adcCrossingPeriod;
static uint32_t nextCrossingDetect;  //ADC换相时间(在中断中会计算出来,下一个换相的时间)

//重新对ADC内部校准
static void adcCalibrateADC(ADC_TypeDef *ADCx) 
{
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

void adcInit(void) 
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    adcSetConstants();
    histSize = ADC_HIST_SIZE;

    // Use STM32's Dual Regular Simultaneous Mode capable of ~ 1.7M samples per second

    // NOTE: assume that RCC code has already placed all pins into Analog In mode during startup

    // DMA1 channel1 configuration (ADC1)
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1 + 0x4c;   //从这个寄存器读
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcRawData[0];    //写入到这个内存
	DMA_InitStructure.DMA_BufferSize = sizeof(adcRawData)/4;            //传输数据量

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                     //从外设读
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;       //外设地址不递加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                //存储器地址递加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;//外设数据宽度32位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;        //存储器数据宽度32位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                        //循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                //通道优先级最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                           //非存储器到存储器模式
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
    DMA_ClearITPendingBit(DMA1_IT_GL1 | DMA1_IT_TC1 | DMA1_IT_HT1);
    DMA_Cmd(DMA1_Channel1, ENABLE);


    // Enable the DMA1_Channel1 global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    // ADC1 configuration
//    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;//混合的同步规则+注入同步模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;         //使用扫描模式

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                  //连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //SWSTART 软件触发模式
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              //数据右对齐

	ADC_InitStructure.ADC_NbrOfChannel = sizeof(adcRawData)/4;//规则通道序列长度 有8个转换通道
    ADC_Init(ADC1, &ADC_InitStructure);

#ifdef ADC_FAST_SAMPLE
	//有8个转换通道 都是规则转换序列
	//ADC_SAMPLE_TIME是AD的采样时间
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
    ADC_DMACmd(ADC1, ENABLE);//ADC1开启DMA模式


	// ADC2 configuration
	//ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;                  //混合的同步规则+注入同步模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;                           //使用扫描模式

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                     //连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;    //SWSTART 软件触发模式
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                 //数据右对齐

	ADC_InitStructure.ADC_NbrOfChannel = sizeof(adcRawData)/4;             //规则通道序列长度 有8个转换通道
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

    ADC_ExternalTrigConvCmd(ADC2, ENABLE);//使用外部事件启动转换

    // enable and calibrate
    ADC_Cmd(ADC1, ENABLE);
    adcCalibrateADC(ADC1);
    ADC_Cmd(ADC2, ENABLE);
    adcCalibrateADC(ADC2);

    nextCrossingDetect = adcMaxPeriod;

    // setup injection sequence
	// 设置注入序列
    ADC_InjectedSequencerLengthConfig(ADC1, 1);//注入序列只有1个转换
    ADC_InjectedSequencerLengthConfig(ADC2, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SAMPLE_TIME);//设置注入序列转换的通道
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_4, 1, ADC_SAMPLE_TIME);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);//注入序列 使用外部事件启动转换
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);//软件触发
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);

    // Start ADC1 / ADC2 Conversions
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//开始转换.并设置好外部触发模式
}

void adcSetCrossingPeriod(int32_t crossPer) {
    adcCrossingPeriod = crossPer<<15;
    crossingPeriod = crossPer;
}

static void adcGrowHist(void) {
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

static void adcShrinkHist(void) 
{
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

static void adcEvaluateHistSize(void) 
{
    int16_t sizeNeeded;

//  sizeNeeded = crossingPeriod/16/TIMER_MULT;
//    sizeNeeded = crossingPeriod/20/TIMER_MULT;
//  sizeNeeded = crossingPeriod/24/TIMER_MULT;
    sizeNeeded = crossingPeriod/32/TIMER_MULT;

//    if (sizeNeeded > (histSize+1) && histSize < ADC_HIST_SIZE)
    if (sizeNeeded > (histSize+1) && histSize < ADC_HIST_SIZE)
		adcGrowHist();  //增加
    else if (sizeNeeded < (histSize-1) && sizeNeeded > 1)
		adcShrinkHist();//减小
}

//dma1 ad采样完成中断
//#pragma GCC optimize ("-O1")
#pragma O1
void DMA1_Channel1_IRQHandler(void) 
{
	register uint16_t *raw = (uint16_t *)adcRawData;
	register uint32_t valA, valB, valC;
	uint32_t currentMicros;

	//__asm volatile ("cpsid i");
	//CPSID_I();
	__disable_irq();
	currentMicros = timerGetMicros();//获取当前时间
	//__asm volatile ("cpsie i");
	//CPSIE_I();
	__enable_irq();

#ifdef ADC_FAST_SAMPLE
	if ((DMA1->ISR & DMA1_FLAG_TC1) != RESET) {
		//转换完成了
		raw += (ADC_CHANNELS * 4);        // 4 16bit words each
		adcAvgVolts -= (adcAvgVolts - (int32_t)((raw[0]+raw[2]) << (ADC_VOLTS_PRECISION-1))) >> 6;
		//                                       VIN    VIN
	}
	else {
		//半传输完成中断 计算电流
		adcAvgAmps -= (adcAvgAmps - (int32_t)((raw[0]+raw[2])<<(ADC_AMPS_PRECISION-1)))>>6;
		//                                     SENSE_CURRENT
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

	if (runMode == SERVO_MODE)//运行在伺服模式 不需要AD采样
		return;


	// blanking time after commutation
	if (!fetCommutationMicros || 
		((currentMicros >= fetCommutationMicros) ? (currentMicros - fetCommutationMicros) : (TIMER_MASK - fetCommutationMicros + currentMicros)) > adcblankingMicros
		// 当前的时间 >= fet换向的时间 ?            当前时间-fet换向的时间  :     (0xFFFFffff-fet换向的时间+当前时间) > adcblankingMicros(反电动势)
		) 
	{
#ifdef ADC_FAST_SAMPLE
		histA[histIndex] = valA = (raw[1]+raw[3]);//SENSE_A
		histB[histIndex] = valB = (raw[4]+raw[6]);//SENSE_B
		histC[histIndex] = valC = (raw[5]+raw[7]);//SENSE_C
#else
		histA[histIndex] = valA = raw[1];
		histB[histIndex] = valB = raw[2];
		histC[histIndex] = valC = raw[3];
#endif
		histIndex = (histIndex + 1) % histSize;

		avgA += valA - histA[histIndex];
		avgB += valB - histB[histIndex];
		avgC += valC - histC[histIndex];


		if ((avgA+avgB+avgC)/histSize > (ADC_MIN_COMP*3) && state != ESC_STATE_DISARMED && state != ESC_STATE_NOCOMM) 
		{
			register int32_t periodMicros;//当前时间 - 上次换相时间 = 两次时间间隔

			//                                                    当前的时间 - 上次换相的时间
			periodMicros = (currentMicros >= detectedCrossing) ? (currentMicros - detectedCrossing) : (TIMER_MASK - detectedCrossing + currentMicros);//得到两次ad的间隔时间

			if (periodMicros > nextCrossingDetect) //超过了时间点,开始换向
			{
				register uint8_t nextStep = 0;

				//这里判断当前读取到的3相电压值
				if (!adcStateA && avgA >= (avgB+avgC)>>1)
				{
					//当前在STEP5 切换到STEP6
					adcStateA = 1;
					nextStep = 6;  //切换到STEP6
				}
				else if (adcStateA && avgA <= (avgB+avgC)>>1) 
				{
					//当前在STEP2 切换到STEP3
					adcStateA = 0;
					nextStep = 3;
				}
				else if (!adcStateB && avgB >= (avgA+avgC)>>1) 
				{
					//当前在STEP3 切换到STEP4
					adcStateB = 1;
					nextStep = 4;
				}
				else if (adcStateB && avgB <= (avgA+avgC)>>1) 
				{
					adcStateB = 0;
					nextStep = 1;

#ifdef ESC_DEBUG
					digitalTogg(tp);
#endif
				}
				else if (!adcStateC && avgC >= (avgA+avgB)>>1) 
				{
					adcStateC = 1;
					nextStep = 2;
				}
				else if (adcStateC && avgC <= (avgA+avgB)>>1) 
				{
					adcStateC = 0;
					nextStep = 5;
				}

				//进行无刷电机的换向
				if (nextStep && periodMicros > adcMinPeriod) //超过了最小换相时间(us, 默认50us)
				{
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
					fetStep = nextStep;       //切换到下一个换向
					fetCommutationMicros = 0; //电机换向的时间清零
					timerSetAlarm1(crossingPeriod/2 - (ADC_DETECTION_TIME*(histSize+2))/2 - ADC_COMMUTATION_ADVANCE, fetCommutate, crossingPeriod);


					// 记录下这次换相的时间
					// record crossing time
					detectedCrossing = currentMicros;

					// resize history based on period
					adcEvaluateHistSize();


					// 计算下一个换相时间
					// calculate next crossing detection time
					//		nextCrossingDetect = crossingPeriod*2/3;
					nextCrossingDetect = crossingPeriod*3/4;
					//		nextCrossingDetect = crossingPeriod*6/8;


					// record highest current draw for this run
					// 记录下最大的消耗电流
					if (adcAvgAmps > adcMaxAmps)
						adcMaxAmps = adcAvgAmps;
				}
			}
		}
	}    
}

#if 0//没有找到在哪里调用
// start injected conversion of current sensor
int32_t adcGetInstantCurrent(void) {
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) != SET)
		;
    return (int32_t)ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
}
#endif

void adcSetConstants(void) 
{
    float shuntResistance = p[SHUNT_RESISTANCE];
    float advance = p[ADVANCE];
    float blankingMicros = p[BLANKING_MICROS];
    float minPeriod = p[MIN_PERIOD];
    float maxPeriod = p[MAX_PERIOD];

    // bounds checking
	// 边界检查.不能超出一定的范围
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

	//计算出几个参数
    adcToAmps = ((ADC_TO_VOLTAGE / ((1<<(ADC_AMPS_PRECISION))+1)) / (ADC_SHUNT_GAIN * shuntResistance / 1000.0f));
    adcAdvance = 100.0f / (advance * (50.0f / 30.0f));
    adcblankingMicros = blankingMicros * TIMER_MULT;
    adcMinPeriod = minPeriod * TIMER_MULT;//adc的最小采样周期(时间us)
    adcMaxPeriod = maxPeriod * TIMER_MULT;//adc的最大采样周期

	//写回数组里面
    p[SHUNT_RESISTANCE] = shuntResistance;
    p[ADVANCE] = advance;
    p[BLANKING_MICROS] = blankingMicros;
    p[MIN_PERIOD] = minPeriod;
    p[MAX_PERIOD] = maxPeriod;
}
