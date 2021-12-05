/*
 * File      : thread_adc.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-09    majianjia   the first version
 */


#include "stm32f10x.h"
#include <rtthread.h>
#include  <string.h>
#include "thread_adc.h"
#include "time_measure.h"

#include "thread_main.h"
#include "db_can_message.h"

#define UPDATE_FREQUENCY  100
#define SAMPLE_RESISTER   0.0005f

#define BAT_1S_CHANNEL 		ADC_Channel_4
#define BAT_2S_CHANNEL 		ADC_Channel_3
#define BAT_3S_CHANNEL 		ADC_Channel_2
#define BAT_4S_CHANNEL 		ADC_Channel_1
#define BAT_5S_CHANNEL 		ADC_Channel_0

#define VCC5_CURR_CHANNEL 		ADC_Channel_7
#define BAT_CURR_CHANNEL 		ADC_Channel_8
#define BAT_VOLT_CHANNEL 		ADC_Channel_9

#define BAT_1S_SCALE 		(20.f/10.f)
#define BAT_2S_SCALE		(61.f/10.f)
#define BAT_3S_SCALE		(61.f/10.f)
#define BAT_4S_SCALE		(61.f/10.f)
#define BAT_5S_SCALE		(110.f/10.f)	

#define VCC5_CURR_SCALE		(1/1001.f) / SAMPLE_RESISTER 
#define BAT_CURR_SCALE 		(1/101.f) / SAMPLE_RESISTER	
#define BAT_VOLT_SCALE		(110.f/10.f)

#define VCC5_CURR_OFFSET     0
#define BAT_CURR_OFFSET     120
#define BAT_VOLT_OFFSET     0
 

short adc_raw[10];

int volt_rt[10];
int volt[10];

unsigned int battery_s = 0; //how many single cells in the battery

struct _adc adc;

static struct rt_timer timer;
static struct rt_semaphore sem;



//multiple axis sliding windows filter
#undef FILTER_WINDOWS
#define  FILTER_WINDOWS    UPDATE_FREQUENCY	 //1s
#define  FILTER_CHANNEL		10

static void sliding_windows_filter(int in[FILTER_CHANNEL], int out[FILTER_CHANNEL])
{
	static int buf[FILTER_WINDOWS][FILTER_CHANNEL];
	static int buf_pointer = 0;
	int weight = FILTER_WINDOWS;
	int count = FILTER_WINDOWS;
	int ch_count = FILTER_CHANNEL;
	int index = buf_pointer;
	int total_weight = 0;
	float integral[FILTER_CHANNEL] = {0};
	
	//add the newest date to the buffer
	ch_count = FILTER_CHANNEL;
	while(ch_count>0)
	{
		ch_count--;
		buf[index][ch_count] = in[ch_count];
	}


	//filter
	while(count--)
	{
		ch_count = FILTER_CHANNEL;
		while(ch_count>0)
		{
			ch_count--;
			integral[ch_count] += buf[index][ch_count] * weight;
		}
		
		total_weight += weight; //the newest data have the most significant weight
		weight--;				
				
		index++;
		if(index >= FILTER_WINDOWS)
			index = 0;
	}
	buf_pointer ++;
	if(buf_pointer >= FILTER_WINDOWS)
		buf_pointer = 0;
	
	//update the newest date
	ch_count = FILTER_CHANNEL;
	while(ch_count>0)
	{
		ch_count--;
		out[ch_count] = (integral[ch_count] / total_weight);
	}
}



void adc_hw_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
 //   NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    // NOTE: assume that RCC code has already placed all pins into Analog In mode during startup
	
    // DMA1 channel1 configuration (ADC1)
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1 + 0x4c;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_raw[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 10;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);
	
    // ADC1 configuration
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 10;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, BAT_1S_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, BAT_2S_CHANNEL, 2, ADC_SampleTime_239Cycles5);	
    ADC_RegularChannelConfig(ADC1, BAT_3S_CHANNEL, 3, ADC_SampleTime_239Cycles5);	
    ADC_RegularChannelConfig(ADC1, BAT_4S_CHANNEL, 4, ADC_SampleTime_239Cycles5);	
	ADC_RegularChannelConfig(ADC1, BAT_5S_CHANNEL, 5, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, VCC5_CURR_CHANNEL, 6, ADC_SampleTime_239Cycles5);	
    ADC_RegularChannelConfig(ADC1, BAT_CURR_CHANNEL, 7, ADC_SampleTime_239Cycles5);	
    ADC_RegularChannelConfig(ADC1, BAT_VOLT_CHANNEL, 8, ADC_SampleTime_239Cycles5);	
	
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 9, ADC_SampleTime_239Cycles5);	
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint,   10, ADC_SampleTime_239Cycles5);	
	
    ADC_DMACmd(ADC1, ENABLE);
	
	ADC_TempSensorVrefintCmd (ENABLE);

    // enable and calibrate
    ADC_Cmd(ADC1, ENABLE);
    ADC_StartCalibration(ADC1);

    // Start ADC1 / ADC2 Conversions
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void adc_timeout(void *parameter)
{
	rt_sem_release(&sem);
}
void adc_sw_init(void)
{
	rt_sem_init(&sem, "adc", 0, RT_IPC_FLAG_FIFO);	
	
	rt_timer_init(&timer, "adc", 
		adc_timeout,
		RT_NULL, 
		RT_TICK_PER_SECOND / UPDATE_FREQUENCY, //10ms 
		RT_TIMER_FLAG_PERIODIC); 
	rt_timer_start(&timer);
}


void thread_adc(void)
{
	unsigned int cycle = 0;
	
	adc_sw_init();
	adc_hw_init();

	while(1)
	{
		rt_sem_take(&sem, 2001);
		cycle++;
		
start_measure_time();
			
		//convert to voltage
		volt_rt[0] = adc_raw[0];
		volt_rt[1] = adc_raw[1];
		volt_rt[2] = adc_raw[2];
		volt_rt[3] = adc_raw[3];
		volt_rt[4] = adc_raw[4];
		volt_rt[5] = adc_raw[5];
		volt_rt[6] = adc_raw[6];
		volt_rt[7] = adc_raw[7];
		volt_rt[8] = adc_raw[8];
		volt_rt[9] = adc_raw[9];
		
		//filter
return_time();
		sliding_windows_filter(volt_rt, volt);
return_time();
		
		//output
		#define VALID_THREHOLD 100
		if(volt[0] > VALID_THREHOLD)
			adc.bat_1s = volt[0] / 4096.f*3.3f * BAT_1S_SCALE;
		if(volt[1] > VALID_THREHOLD)
			adc.bat_2s = volt[1] / 4096.f*3.3f * BAT_2S_SCALE -adc.bat_1s;
		if(volt[2] > VALID_THREHOLD)
			adc.bat_3s = volt[2] / 4096.f*3.3f * BAT_3S_SCALE -adc.bat_2s -adc.bat_1s;
		if(volt[3] > VALID_THREHOLD)
			adc.bat_4s = volt[3] / 4096.f*3.3f * BAT_4S_SCALE -adc.bat_3s -adc.bat_2s -adc.bat_1s;
		if(volt[4] > VALID_THREHOLD)
			adc.bat_5s = volt[4] / 4096.f*3.3f * BAT_5S_SCALE -adc.bat_4s -adc.bat_3s -adc.bat_2s -adc.bat_1s;


		adc.vcc5_curr = (volt[5]  - VCC5_CURR_OFFSET)/4096.f*3.3f  * VCC5_CURR_SCALE;
		adc.bat_curr = (volt[6] - BAT_CURR_OFFSET) /4096.f*3.3f  * BAT_CURR_SCALE;
		adc.bat_volt = (volt[7] - BAT_VOLT_OFFSET) /4096.f*3.3f  * BAT_VOLT_SCALE;
	
		adc.temp = (1.43f - (volt[8]) / 4096.f*3.3f)/0.0043f + 25.f;
		adc.refint = volt[9] / 4096.f*3.3f ;
return_time();
		
		{
			static CanTxMsg TxMessage;
			unsigned short int temp16;
					
			//send to canbus message 0, general battery status
			TxMessage.StdId = CAN_TYPE_MASTER | CAN_BATTERY | CAN_MESSAGE_INDEX0;
			TxMessage.ExtId = 0x00;
			TxMessage.RTR = CAN_RTR_DATA;
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.DLC = 8;
			
			//the first battery
			TxMessage.Data[0] = 1;
			
			//percentage
			TxMessage.Data[1] = 70;
			
			//voltage
			temp16 = adc.bat_volt * 100;
			memcpy(&TxMessage.Data[2], &temp16, 2);
			
			//current
			temp16 = adc.bat_curr * 100;
			memcpy(&TxMessage.Data[4], &temp16, 2);
			
			//capacity
			temp16 = 0;
			memcpy(&TxMessage.Data[6], &temp16, 2);
			
			//send out
			can_message_send(&TxMessage);	


			//send to canbus message 1, voltage for each battery
			TxMessage.StdId = CAN_TYPE_MASTER | CAN_BATTERY | CAN_MESSAGE_INDEX1;
			TxMessage.ExtId = 0x00;
			TxMessage.RTR = CAN_RTR_DATA;
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.DLC = 8;
			
			//the first battery
			TxMessage.Data[0] = 1;
			
			//1s
			TxMessage.Data[1] = adc.bat_1s * 50;
			
			//2s
			TxMessage.Data[2] = adc.bat_2s * 50;
			
			//3s
			TxMessage.Data[3] = adc.bat_3s * 50;
			
			//4s
			TxMessage.Data[4] = adc.bat_4s * 50;
			
			//5s
			TxMessage.Data[5] = adc.bat_5s * 50;	
			
			//6s
			TxMessage.Data[6] = 0;
				
			//7s
			TxMessage.Data[7] = 0;
			
			//send to bus
			can_message_send(&TxMessage);	

		}
		
		//set batteries cell's count
		if(rt_tick_get() < RT_TICK_PER_SECOND*4)
		{
			#define ONE_CELL_V_MAX 4.3
			#define ONE_CELL_V_MIN 3.5
			
			if(ONE_CELL_V_MIN*1 <= adc.bat_volt && adc.bat_volt < ONE_CELL_V_MAX*1)//
				battery_s = 1;
			if(ONE_CELL_V_MIN*2 <= adc.bat_volt && adc.bat_volt < ONE_CELL_V_MAX*2)//
				battery_s = 2;
			if(ONE_CELL_V_MIN*3 <= adc.bat_volt && adc.bat_volt < ONE_CELL_V_MAX*3)
				battery_s = 3;
			if(ONE_CELL_V_MIN*4 <= adc.bat_volt && adc.bat_volt < ONE_CELL_V_MAX*4)
				battery_s = 4;
			if(ONE_CELL_V_MIN*5 <= adc.bat_volt && adc.bat_volt < ONE_CELL_V_MAX*5)
				battery_s = 5;
		}
		else 
		//every 1s to see if battery is low voltage
		{
			extern void beep(unsigned int tune, unsigned int time);
			static unsigned int tick = 0;
			
			if(rt_tick_get()>= tick + RT_TICK_PER_SECOND && 
				adc.bat_volt < 3.5f*battery_s && adc.bat_volt > 2.7f*battery_s)
			{
				tick = rt_tick_get();
				beep(5, 200);				//send alarm
			}
		}

return_time();
	}



}
