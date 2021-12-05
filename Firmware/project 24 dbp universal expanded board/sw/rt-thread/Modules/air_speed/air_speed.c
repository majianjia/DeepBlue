/*
 * File      :air_speed.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013.6.21   majianjia   the first version
 */
#include "stm32f10x.h"
#include "ultrasonic_ranging.h"
#include "rtthread.h"
#include "math.h"
#include "thread_main.h"

#define MPXV_DEFAULTE_VOLT 5.F

#define ADC1_DR_Address     ((u32)0x4001244C)
unsigned short AD_Value[2];
float mpxv_volt_raw = 0;	//��������ѹ
float mpxv_out_raw = 0;		//�����ѹ
float mpxv_volt = 0;
float mpxv_out = MPXV_DEFAULTE_VOLT;		//�˲��������ѹ
float air_speed = 0;
float rt_air_speed = 0;	//ʵʱ����
float mpxv_p = 0;			
float p_diff = 0;		//ѹ��

int volatile adc1_raw[2] = {0,0};
int volatile adc1_raw_times = 0;
int volatile adc1_raw_busy_flag = 0; //�Ƿ��ڲ�������


//sem
static struct rt_semaphore sem;

//��ʱ�����ƿ�
static struct rt_timer tim_as;

//
void as_hw_init(void)
{
   	DMA_InitTypeDef DMA_InitStructure;
   	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;    
	NVIC_InitTypeDef NVIC_InitStructure;
	

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  	
	
	//
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;            
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;   	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	 
	//
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;            
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;   	
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	 

  	/* ADCCLK = PCLK2/4 */
  	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
  	/* Enable peripheral clocks ------------------------------------------------*/
  	/* Enable DMA1 clock */
 	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  	/* Enable ADC1 and GPIOC clock */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//DMA�ж�
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

 	DMA_DeInit(DMA1_Channel1);
  	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&AD_Value;
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  	DMA_InitStructure.DMA_BufferSize = 2;
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  	/* Enable DMA1 channel1 */
  	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
  
  	/* ADC1 configuration ------------------------------------------------------*/
	ADC_DeInit(ADC1);

  	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  	ADC_InitStructure.ADC_NbrOfChannel = 2;
  	ADC_Init(ADC1, &ADC_InitStructure);

  	/* ADC1 regular channel1 configuration */ 
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5);
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_239Cycles5);

  	/* Enable ADC1 DMA */
  	ADC_DMACmd(ADC1, ENABLE);
  
  	/* Enable ADC1 */
  	ADC_Cmd(ADC1, ENABLE);

  	/* Enable ADC1 reset calibration register */   
  	ADC_ResetCalibration(ADC1);
  	/* Check the end of ADC1 reset calibration register */
  	while(ADC_GetResetCalibrationStatus(ADC1));

  	/* Start ADC1 calibration */
  	ADC_StartCalibration(ADC1);
  	/* Check the end of ADC1 calibration */
  	while(ADC_GetCalibrationStatus(ADC1));
     
  	/* Start ADC1 Software Conversion */ 
  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
}

//�жϴ�����
void dma1_channel1_handle(void)
{
	//������ڲ�������������ֱ���˳�
	if(adc1_raw_busy_flag)
	{
		DMA_ClearITPendingBit(DMA1_IT_GL1);
		return;
	}
	
	adc1_raw[0] += AD_Value[0];
	adc1_raw[1] += AD_Value[1];
	adc1_raw_times ++;

	DMA_ClearITPendingBit(DMA1_IT_GL1);
}

//�����ʱ�����Ͳ����ź�
void timer_as_timeout(void *p)
{	
	//�����жϻ��������Ļ���ֵ����λ
	adc1_raw_busy_flag = 1; //����

	mpxv_volt_raw = 3.3f/4096.f * adc1_raw[0]/(float)adc1_raw_times *2.f;	//ת��Ϊ��ѹֵ
	mpxv_out_raw  =	3.3f/4096.f * adc1_raw[1]/(float)adc1_raw_times;		//�����������ѹ
	adc1_raw_times = 0;
	adc1_raw[0] = 0;
	adc1_raw[1] = 0;
	
	adc1_raw_busy_flag = 0; //�ͷŻ���
	
	rt_sem_release(&sem);
}

void as_sw_init(void)
{
	rt_sem_init(&sem,"air speed",0 ,RT_IPC_FLAG_FIFO);
	
	rt_timer_init(&tim_as,
					"air speed", 				/*name*/
					timer_as_timeout, 				/* ��ʱ����*/
					RT_NULL,					/* ���� */
					2, 							/* ��ʱ���� */
					RT_TIMER_FLAG_PERIODIC);/* ���ڶ�ʱ��*/	
	rt_timer_start(&tim_as);
}


//
void thread_as(void)
{	
	float p = 0;
	float p_default = 0;
	float mpxv_rt_p = 0;//ʵʱѹ��
	
	as_sw_init();
	as_hw_init();
	
	while(1)
	{	
		rt_sem_take(&sem, RT_WAITING_FOREVER);
						
		//ǰ3~5��
		if((seconds<5) && (seconds>3))
		{
			{
				static double p_default_int = 0;
				static int p_default_int_times = 0;
				
				mpxv_p = (mpxv_out_raw - 0.5f * mpxv_volt_raw)/(0.2f * mpxv_volt_raw);
				
				//����
				p_default_int += mpxv_p;
				p_default_int_times ++;
				
				//�������
				p_default = p_default_int / p_default_int_times;
			}
		}
		//�������ֱ�Ӽ���
		else if(seconds>5)
		{	
			//����ʵʱѹ��
			mpxv_rt_p = (mpxv_out_raw - 0.5f * mpxv_volt_raw)/(0.2f * mpxv_volt_raw);
			p = (p_default - mpxv_rt_p) * 1000.f; //1V/1KP		
			if(p > 0)
				rt_air_speed = sqrt((2.f* p /1.205f));		//�������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
			else
				rt_air_speed = -sqrt((2.f* (-p) /1.205f));
			
			//��¼��ѹ��
			p_diff = p;
			
			//�˲���������
			{
				#define AS_AVG_WINDOWS 64
				
				static float as[AS_AVG_WINDOWS];
				static int as_avg_times = 0;	//��¼����λ��
				
				int count = AS_AVG_WINDOWS;		// ���ٵ��˲����ڴ�С
				int index = 0;		// �˲�����
				int weight = AS_AVG_WINDOWS;	// Ȩֵ
				int total_weight = 0;			// ��Ȩֵ
				double as_out_int = 0; 
				
				//������㿪ʼ��������
				index = as_avg_times;
				
				//�����µ�����
				as[index] = rt_air_speed;
			
				//�����¼����������ֵ
				while(count--)
				{
					as_out_int += as[index] * (float)weight;
					
					total_weight += weight; //Ȩֵ
					weight--;				//Ȩֵ
					
					index++;
					if(index >= AS_AVG_WINDOWS)
						index = 0;
				}
				
				//д���µ�ֵ
				air_speed = as_out_int / (double)total_weight;
				
				//�л�����һ��
				as_avg_times ++;
				if(as_avg_times >= AS_AVG_WINDOWS)
					as_avg_times = 0;
			}
		}
			
	}

}
