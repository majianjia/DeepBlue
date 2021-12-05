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
float mpxv_volt_raw = 0;	//传感器电压
float mpxv_out_raw = 0;		//输出电压
float mpxv_volt = 0;
float mpxv_out = MPXV_DEFAULTE_VOLT;		//滤波后输出电压
float air_speed = 0;
float rt_air_speed = 0;	//实时空速
float mpxv_p = 0;			
float p_diff = 0;		//压差

int volatile adc1_raw[2] = {0,0};
int volatile adc1_raw_times = 0;
int volatile adc1_raw_busy_flag = 0; //是否在操作数据


//sem
static struct rt_semaphore sem;

//定时器控制块
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
	
	//DMA中断
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

//中断处理函数
void dma1_channel1_handle(void)
{
	//如果正在操作缓冲区，则直接退出
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

//软件定时器发送测量信号
void timer_as_timeout(void *p)
{	
	//计算中断积累下来的积分值并复位
	adc1_raw_busy_flag = 1; //互斥

	mpxv_volt_raw = 3.3f/4096.f * adc1_raw[0]/(float)adc1_raw_times *2.f;	//转换为电压值
	mpxv_out_raw  =	3.3f/4096.f * adc1_raw[1]/(float)adc1_raw_times;		//传感器输出电压
	adc1_raw_times = 0;
	adc1_raw[0] = 0;
	adc1_raw[1] = 0;
	
	adc1_raw_busy_flag = 0; //释放互斥
	
	rt_sem_release(&sem);
}

void as_sw_init(void)
{
	rt_sem_init(&sem,"air speed",0 ,RT_IPC_FLAG_FIFO);
	
	rt_timer_init(&tim_as,
					"air speed", 				/*name*/
					timer_as_timeout, 				/* 超时函数*/
					RT_NULL,					/* 参数 */
					2, 							/* 定时长度 */
					RT_TIMER_FLAG_PERIODIC);/* 周期定时器*/	
	rt_timer_start(&tim_as);
}


//
void thread_as(void)
{	
	float p = 0;
	float p_default = 0;
	float mpxv_rt_p = 0;//实时压差
	
	as_sw_init();
	as_hw_init();
	
	while(1)
	{	
		rt_sem_take(&sem, RT_WAITING_FOREVER);
						
		//前3~5秒
		if((seconds<5) && (seconds>3))
		{
			{
				static double p_default_int = 0;
				static int p_default_int_times = 0;
				
				mpxv_p = (mpxv_out_raw - 0.5f * mpxv_volt_raw)/(0.2f * mpxv_volt_raw);
				
				//积分
				p_default_int += mpxv_p;
				p_default_int_times ++;
				
				//计算零点
				p_default = p_default_int / p_default_int_times;
			}
		}
		//其他情况直接计算
		else if(seconds>5)
		{	
			//计算实时压差
			mpxv_rt_p = (mpxv_out_raw - 0.5f * mpxv_volt_raw)/(0.2f * mpxv_volt_raw);
			p = (p_default - mpxv_rt_p) * 1000.f; //1V/1KP		
			if(p > 0)
				rt_air_speed = sqrt((2.f* p /1.205f));		//计算表速                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
			else
				rt_air_speed = -sqrt((2.f* (-p) /1.205f));
			
			//记录下压差
			p_diff = p;
			
			//滤波后计算空速
			{
				#define AS_AVG_WINDOWS 64
				
				static float as[AS_AVG_WINDOWS];
				static int as_avg_times = 0;	//记录数组位置
				
				int count = AS_AVG_WINDOWS;		// 空速的滤波窗口大小
				int index = 0;		// 滤波次数
				int weight = AS_AVG_WINDOWS;	// 权值
				int total_weight = 0;			// 总权值
				double as_out_int = 0; 
				
				//从这个点开始更新数据
				index = as_avg_times;
				
				//放入新的数据
				as[index] = rt_air_speed;
			
				//计算记录下来的数据值
				while(count--)
				{
					as_out_int += as[index] * (float)weight;
					
					total_weight += weight; //权值
					weight--;				//权值
					
					index++;
					if(index >= AS_AVG_WINDOWS)
						index = 0;
				}
				
				//写入新的值
				air_speed = as_out_int / (double)total_weight;
				
				//切换到下一组
				as_avg_times ++;
				if(as_avg_times >= AS_AVG_WINDOWS)
					as_avg_times = 0;
			}
		}
			
	}

}
