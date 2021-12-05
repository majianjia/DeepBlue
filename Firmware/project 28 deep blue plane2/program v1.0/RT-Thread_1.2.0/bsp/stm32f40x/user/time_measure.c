/*---------------------------------------------------------------*
*time_measure.h for AFC V0.2
*ע�⣺
*	   ��.c�ļ������ڲ���ϵͳ������ʱ��
*ռ��Ӳ����		
*��ʼ������void start_measure_time(void)
*��¼ʱ�䣺unsigned int return_timer(void)
*ֹͣ����: void stop_measure_time(void)	   
*����ʱ�䣺0~65535us 0~65535ms
*2011.10.20
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "time_measure.h"

//ʱ������(���ʱ�����)
#define TIME_CAP 		8

//�������� ms ���� us ����Ҫ��ע�͵����ɣ�ע�⣺�������ֵ����65535��
#define TIME_RANGE_US
//#define TIME_RANGE_MS

//ʹ�õĶ�ʱ��
#define TIMER_USED 				 TIM6
#define RCC_APB1Periph_TIM_USED  RCC_APB1Periph_TIM6

//���붨ʱ����ʱ�ӣ�ϵͳʱ�ӣ���λMHz
#define TIMER_FREQUENCY 84

//�ܽṹ��
struct _time_meas
{
	unsigned int point[TIME_CAP];			//ʱ���
	unsigned int distant[TIME_CAP];			//�˴β�����ʱ�����
	unsigned int distant_max[TIME_CAP];		//ϵͳ��������distant�����ֵ
	
	unsigned int period;					//�ڶ��λص���ʼ���ʱ��
	unsigned int period_max;				//period �����ֵ

	unsigned int total;						//�˴δӿ�ʼ���������һ��ʱ����ʱ�䳤��
	unsigned int total_max;					//ϵͳ����������total�����ֵ

	unsigned int count;						//�������

}time_meas;

//���ò�����ʼ��
void start_measure_time(void)
{
	static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	int i ;
	
	//��¼������
	time_meas.period  =  TIMER_USED->CNT;
	if(time_meas.period > time_meas.period_max) 
		time_meas.period_max = time_meas.period ;

	//��λ����ֵ
	time_meas.count = 0;

	//������һ�ε�ʱ����
	for(i=0; i<TIME_CAP; i++)
	{
		//��һ����ֱ��ʱ����ֱ�ӵ��ڵ�һ�����ֵ
		if(i == 0)
		{
			time_meas.distant[0] = time_meas.point[0];
			continue;
		}
		//��Ч�㣬������
		if(time_meas.point[i] < time_meas.point[i-1])break;
		
		time_meas.distant[i] = time_meas.point[i] - time_meas.point[i-1];		
	}
	//��¼�������ʱ��
	time_meas.total = time_meas.point[i-1];
	if(time_meas.total_max < time_meas.total)
		time_meas.total_max = time_meas.total;

	//��ȡdistant���ֵ
	for(i=0; i<TIME_CAP; i++)
	{
		if(time_meas.distant_max[i] < time_meas.distant[i])
			time_meas.distant_max[i] = time_meas.distant[i];	
	}

	//�趨��ʱ��
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
	/*��system_stm32f4xx.c�����õ�APB1 Prescaler = 4 ,��֪
	APB1ʱ��Ϊ168M/4*2,��Ϊ���APB1��Ƶ��Ϊ1����ʱʱ��*2
	*/

}

//��ȡ����ֵ
unsigned int return_time(void)
{
	time_meas.point[time_meas.count] = TIMER_USED->CNT;
	time_meas.count ++;

   	return time_meas.point[time_meas.count];
}

//ֹͣ����
void stop_measure_time(void)
{
	int i ;

	//��λ����ֵ
	time_meas.count = 0;

	//������һ�ε�ʱ����
	for(i=0; i<TIME_CAP; i++)
	{
		//��һ����ֱ��ʱ����ֱ�ӵ��ڵ�һ�����ֵ
		if(i == 0)
		{
			time_meas.distant[0] = time_meas.point[0];
			continue;
		}
		//��Ч�㣬������
		if(time_meas.point[i] < time_meas.point[i-1])break;
		
		time_meas.distant[i] = time_meas.point[i] - time_meas.point[i-1];		
	}
	//��¼�������ʱ��
	time_meas.total = time_meas.point[i-1];
	if(time_meas.total_max < time_meas.total)time_meas.total_max = time_meas.total;

	//��ȡ���ֵ
	for(i=0; i<TIME_CAP; i++)
	{
		if(time_meas.distant_max[i] < time_meas.distant[i])
			time_meas.distant_max[i] = time_meas.distant[i];	
	}

   	//ֹͣ����
	TIM_Cmd(TIMER_USED, DISABLE);
}

	 

