/*
 * File      : thread_calendar.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-1-23    majianjia   the first version
 */
#include "stm32f4xx.h"
#include <rtthread.h>

#include "struct_all.h"
#include "thread_calendar.h"
#include "time_measure.h"
#include "thread_canbus.h"
#include "string.h"


__IO uint32_t AsynchPrediv = 0, SynchPrediv = 0;

void RTC_TimeRegulate(void)
{
	uint32_t tmp_hh = 11, tmp_mm = 59, tmp_ss = 39;
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	
	RTC_TimeStructure.RTC_H12   = RTC_H12_AM;
	RTC_TimeStructure.RTC_Hours = tmp_hh;
	RTC_TimeStructure.RTC_Minutes = tmp_mm;
	RTC_TimeStructure.RTC_Seconds = tmp_ss;
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
	
	RTC_DateStructure.RTC_WeekDay = 3;
	RTC_DateStructure.RTC_Month   = 1;
	RTC_DateStructure.RTC_Date 	  = 23;
	RTC_DateStructure.RTC_Year 	  = 13;
	RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);

	/* Indicator for the RTC configuration */
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0x23F2);
	
	tmp_hh = 0xFF;
	tmp_mm = 0xFF;
	tmp_ss = 0xFF;

	/* Disable the Alarm A */
//	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
}

void RTC_Config(void)
{
	int try_times = 1000;
	RTC_InitTypeDef RTC_InitStructure;
	
	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to RTC */
	PWR_BackupAccessCmd(ENABLE);

	/* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait till LSE is ready */  
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
		rt_thread_delay(1);
		try_times--;
		if(try_times <= 0)return;		
	}

	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	SynchPrediv = 0xFF;
	AsynchPrediv = 0x7F;

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();
	
	/* Configure the RTC data register and RTC prescaler */
	RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
	RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);
}

//硬件初始化
void calendar_hw_init(void)
{	
	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x23F2)
	{  
		/* RTC configuration  */
		RTC_Config();

		/* Configure the time register */
		RTC_TimeRegulate(); 
	}
	else
	{
		/* Check if the Power On Reset flag is set */
		if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
		{

		}
		/* Check if the Pin Reset flag is set */
		else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
		{
		 
		}

	}

}



RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;

//更新系统时间 以后将由RTC中断调用此函数
void system_time_update(void)
{
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		
	system_info.time.date = RTC_DateStructure.RTC_Date;
	system_info.time.month= RTC_DateStructure.RTC_Month;
	system_info.time.year = RTC_DateStructure.RTC_Year;
	system_info.time.weekday = RTC_DateStructure.RTC_WeekDay;
	
	system_info.time.second = RTC_TimeStructure.RTC_Seconds;
	system_info.time.minute = RTC_TimeStructure.RTC_Minutes;
	system_info.time.hour   = RTC_TimeStructure.RTC_Hours;

	//如果RTC时间不同于gps的UTC时间 
	if(( gps.time.flag == 1) && (gps.time.minute != RTC_TimeStructure.RTC_Minutes))
	{
 		PWR_BackupAccessCmd(ENABLE);
 		RTC_WriteProtectionCmd(DISABLE);
 		RTC_EnterInitMode();
 		
 		RTC_TimeStructure.RTC_Hours = gps.time.hour;
 		RTC_TimeStructure.RTC_Minutes = gps.time.minute;
 		RTC_TimeStructure.RTC_Seconds = gps.time.second;
 		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
 		
 		RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Monday;
 		RTC_DateStructure.RTC_Month   = gps.time.month;
 		RTC_DateStructure.RTC_Date 	  = gps.time.date;
 		RTC_DateStructure.RTC_Year 	  = gps.time.year;
 		RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
 		
 		RTC_ExitInitMode();
 		RTC_WriteProtectionCmd(ENABLE);
 		PWR_BackupAccessCmd(DISABLE);
	}
		
}

//专门为FATFS返回时间
unsigned int return_time_for_fatfs(void)
{
	/*
	bit31:25 Year from 1980 (0..127)
	bit24:21 Month (1..12)
	bit20:16 Day in month(1..31)
	bit15:11 Hour (0..23)
	bit10:5 Minute (0..59)
	bit4:0 Second / 2 (0..29) 
	*/
	unsigned int t = 0;
	t |= ((unsigned int)system_info.time.year + 2000 - 1980) << 25;
	t |= (unsigned int)system_info.time.month << 21;
	t |= (unsigned int)system_info.time.date << 16;
	t |= (unsigned int)system_info.time.hour << 11;
	t |= (unsigned int)system_info.time.minute << 5;
	t |= (unsigned int)(system_info.time.second / 2);
	
	return t;
}


//thread_calendar 线程主体
void thread_calendar(void)
{
	int second_bef = 61;
	unsigned int temp;
	CanTxMsg TxMessage;
	
	//时钟初始化
	calendar_hw_init();
	
	//主循环
	while(1)
	{
		rt_thread_delay(1);

		system_time_update();		
		
		//update start up time
		if(second_bef != system_info.time.second )
		{
			second_bef = system_info.time.second;
			//system_info.startup_second ++; //increase in thread_second
			
			//将时间发送到总线
			TxMessage.StdId = CAN_TYPE_MASTER | CAN_TIME_BROADCAST | CAN_MESSAGE_INDEX0;
			TxMessage.ExtId = 0x00;
			TxMessage.RTR = CAN_RTR_DATA;
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.DLC = 8;
			TxMessage.Data[0] = system_info.time.milisecond;
			TxMessage.Data[1] = system_info.time.second;	
			TxMessage.Data[2] = system_info.time.minute;	
			TxMessage.Data[3] = system_info.time.hour;	
			temp = system_info.startup_second;
			memcpy(&TxMessage.Data[4], &temp, 4);
		
			can_message_send(&TxMessage);		
		}
		
		//send date every 5seconds
		if(system_info.startup_second % 5 == 0)
		{
			//将时间发送到总线
			TxMessage.StdId = CAN_TYPE_MASTER | CAN_DATE | CAN_MESSAGE_INDEX0;
			TxMessage.ExtId = 0x00;
			TxMessage.RTR = CAN_RTR_DATA;
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.DLC = 4;
			TxMessage.Data[0] = system_info.time.date;
			TxMessage.Data[1] = system_info.time.weekday;	
			TxMessage.Data[2] = system_info.time.month;	
			TxMessage.Data[3] = system_info.time.year;	
			
			can_message_send(&TxMessage);
		}
	}

}

//
