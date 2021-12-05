/*---------------------------------------------------------------*
*time_measure.h for AFC V0.2
*注意：
*	   此.c文件仅用于测量系统计算用时。
*占用硬件：		
*开始测量：void start_measure_time(void)
*记录时间：unsigned int return_timer(void)
*停止测量: void stop_measure_time(void)	   
*测量时间：0~65535us 0~65535ms
*2011.10.20
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "time_measure.h"

//时间容量(最大时间点数)
#define TIME_CAP 		8

//测量精度 ms 或者 us 不需要的注释掉即可（注意：计数最大值都是65535）
#define TIME_RANGE_US
//#define TIME_RANGE_MS

//使用的定时器
#define TIMER_USED 				 TIM6
#define RCC_APB1Periph_TIM_USED  RCC_APB1Periph_TIM6

//输入定时器的时钟（系统时钟）单位MHz
#define TIMER_FREQUENCY 84

//总结构体
struct _time_meas
{
	unsigned int point[TIME_CAP];			//时间点
	unsigned int distant[TIME_CAP];			//此次测量的时间点间隔
	unsigned int distant_max[TIME_CAP];		//系统启动以来distant的最大值
	
	unsigned int period;					//第二次回到初始点的时间
	unsigned int period_max;				//period 的最大值

	unsigned int total;						//此次从开始测量到最后一个时间点的时间长度
	unsigned int total_max;					//系统启动以来，total的最大值

	unsigned int count;						//不用理会

}time_meas;

//设置测量起始点
void start_measure_time(void)
{
	static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	int i ;
	
	//记录下周期
	time_meas.period  =  TIMER_USED->CNT;
	if(time_meas.period > time_meas.period_max) 
		time_meas.period_max = time_meas.period ;

	//复位计数值
	time_meas.count = 0;

	//计算上一次的时间间隔
	for(i=0; i<TIME_CAP; i++)
	{
		//第一个，直接时间间隔直接等于第一个点的值
		if(i == 0)
		{
			time_meas.distant[0] = time_meas.point[0];
			continue;
		}
		//无效点，则跳出
		if(time_meas.point[i] < time_meas.point[i-1])break;
		
		time_meas.distant[i] = time_meas.point[i] - time_meas.point[i-1];		
	}
	//记录下最大总时间
	time_meas.total = time_meas.point[i-1];
	if(time_meas.total_max < time_meas.total)
		time_meas.total_max = time_meas.total;

	//提取distant最大值
	for(i=0; i<TIME_CAP; i++)
	{
		if(time_meas.distant_max[i] < time_meas.distant[i])
			time_meas.distant_max[i] = time_meas.distant[i];	
	}

	//设定定时器
	TIM_DeInit(TIMER_USED);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_USED, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;

#ifdef TIME_RANGE_US
	TIM_TimeBaseStructure.TIM_Prescaler = TIMER_FREQUENCY  -1 ;	
#endif

#ifdef TIME_RANGE_MS
	TIM_TimeBaseStructure.TIM_Prescaler = TIMER_FREQUENCY * 1000 -1 ;	
#endif

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMER_USED, &TIM_TimeBaseStructure);
	TIM_Cmd(TIMER_USED, ENABLE);
	/*在system_stm32f4xx.c中设置的APB1 Prescaler = 4 ,可知
	APB1时钟为168M/4*2,因为如果APB1分频不为1，则定时时钟*2
	*/

}

//提取测量值
unsigned int return_time(void)
{
	time_meas.point[time_meas.count] = TIMER_USED->CNT;
	time_meas.count ++;

   	return time_meas.point[time_meas.count];
}

//停止测量
void stop_measure_time(void)
{
	int i ;

	//复位计数值
	time_meas.count = 0;

	//计算上一次的时间间隔
	for(i=0; i<TIME_CAP; i++)
	{
		//第一个，直接时间间隔直接等于第一个点的值
		if(i == 0)
		{
			time_meas.distant[0] = time_meas.point[0];
			continue;
		}
		//无效点，则跳出
		if(time_meas.point[i] < time_meas.point[i-1])break;
		
		time_meas.distant[i] = time_meas.point[i] - time_meas.point[i-1];		
	}
	//记录下最大总时间
	time_meas.total = time_meas.point[i-1];
	if(time_meas.total_max < time_meas.total)time_meas.total_max = time_meas.total;

	//提取最大值
	for(i=0; i<TIME_CAP; i++)
	{
		if(time_meas.distant_max[i] < time_meas.distant[i])
			time_meas.distant_max[i] = time_meas.distant[i];	
	}

   	//停止测量
	TIM_Cmd(TIMER_USED, DISABLE);
}

	 

