/*
 * File      : thread_airpress.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-02    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include <drivers/spi.h>
#include "math.h"

#include "struct_all.h"
#include "time_measure.h"

#include "thread_position.h"
#include "thread_heartbeat.h"

#define HEARTBEAT_FREQUENCY  100

static struct rt_timer timer;
static struct rt_semaphore sem;

//ADC 
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)
unsigned short int adc1_raw[3];

//ADC1
struct _adc1
{
	float ch0;
	float ch1;
}adc1;

void heartbeat_timeout(void *parameter)
{
	rt_sem_release(&sem);

}

void heartbeat_sw_init(void)
{	
	rt_sem_init(&sem, "hbeat", 0, RT_IPC_FLAG_FIFO);	
	
	rt_timer_init(&timer, "hbeat", 
		heartbeat_timeout,
		RT_NULL, 
		RT_TICK_PER_SECOND / HEARTBEAT_FREQUENCY, //10ms 
		RT_TIMER_FLAG_PERIODIC); 
}

//
void heartbeat_hw_init(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc1_raw;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream4, ENABLE);


	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;// ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;  
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel4 configuration *************************************/
	/*
		ADC_SampleTime_3Cycles: Sample time equal to 3 cycles 
		ADC_SampleTime_15Cycles: Sample time equal to 15 cycles 
		ADC_SampleTime_28Cycles: Sample time equal to 28 cycles 
		ADC_SampleTime_56Cycles: Sample time equal to 56 cycles 
		ADC_SampleTime_84Cycles: Sample time equal to 84 cycles 
		ADC_SampleTime_112Cycles: Sample time equal to 112 cycles 
		ADC_SampleTime_144Cycles: Sample time equal to 144 cycles 
		ADC_SampleTime_480Cycles: Sample time equal to 480 cycles 
		
		ch16 temperature 
		ch17 reff
		ch18 vbat
	*/

	ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 1, ADC_SampleTime_480Cycles); //VBAT ???????? for F40X/F41X
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 2, ADC_SampleTime_480Cycles); //temp
	
	/* Enable VBAT channel */
	ADC_VBATCmd(ENABLE); 
	
	/* Temp sensor ON */
	ADC_TempSensorVrefintCmd(ENABLE);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);	
	
	//start
	ADC_SoftwareStartConv(ADC1);	
}

//
void heartbeat_init(void)
{
	heartbeat_sw_init();
	heartbeat_hw_init();
}

//
void thread_heartbeat(void * parameter)
{
	unsigned char cpu_major,cpu_minor;
	void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);
	
	heartbeat_init();
	
	while(1)
	{
		rt_sem_take(&sem, RT_TICK_PER_SECOND);
		
		//update VBAT & temperature
		adc1.ch0 = (float)adc1_raw[0] *3.3f / 4096.f;   
		adc1.ch1 = (float)adc1_raw[1] *3.3f / 4096.f;   		
		//Vsense = AD_VALUE * 3000mv/4096. because Avg_slope=2.5mv/0C, SO use 3000mv
		system_info.mcu.temperature.temp = (adc1.ch1 - 0.76f)/0.0025f + 25.0f;   
		system_info.backup_battery.v = adc1.ch0 * 2.f;
		
		//update mcu usage
		cpu_usage_get(&cpu_major, &cpu_minor);
		system_info.mcu.usage = cpu_major + cpu_minor/100.f;
		
		
		
	
	
	}
}
