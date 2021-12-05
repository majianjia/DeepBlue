/*
 * File      : thread_log.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-07    majianjia   the first version
 */

#include "stm32f4xx.h"
#include <rtthread.h>

#include "struct_all.h"
#include "thread_log.h"
#include "time_measure.h"

#include "stdio.h"
#include <dfs_posix.h>

//工作目录一般以时间命名
//格式  20120105-180341  2012年1月5日 18点03分41秒
static char log_dir[32] = {0};
 
//返回文件的时间格式
char* time_printf(void)
{
	static char s[32];
	
	//以时间为准命名目录
	sprintf(s,"20%02d%02d%02d-%02d%02d%02d", 
							system_info.time.year, 
							system_info.time.month,
							system_info.time.date,
							system_info.time.hour,
							system_info.time.minute,
							system_info.time.second);	

	return s;
}




////记录空气数据
//void entry_thread_air_log(void* parameter)
//{
//	const char file_name[] = "air infomation.log";
//	const char instruction[] = 
//	"This is a air information log from Deep Blue Plane.\n"
//	"DATE & TIME \tAir press\tTemperature\tAltitude\tAVG\n";
//	
//	int fd = -1;
//	int	size;
//	int second_bef = 1;
//	char buf[64];
//	int press_avg = 0;
//	float altitude_avg = 0;
//	int avg_count = 0;
//	int air_info_count_bef = 0;
//	char dir[64];
//	
//	//复制工作区 目录
//	strcpy(dir, log_dir);
//	//打开记录文件
//	strcat(dir, file_name);
//	//try to open the working file
//	while(fd < 0)
//	{
//		int count = 0;		
//		fd = open(dir, O_WRONLY | O_CREAT, 0);
//		rt_thread_delay(RT_TICK_PER_SECOND);
//		
//		if ((count ++ )> 60) //60 seconds
//		{
//			rt_kprintf("air log: open file for write failed\n");
//			return;
//		}
//	}
//	
//	rt_kprintf("air information log has been created.\n");

//	//写入说明
//	write(fd, instruction, strlen(instruction));
//	
//	//主循环
//	while(1)
//	{
//		while(second_bef >= system_info.startup_second )
//		{
//			if(air_info_count_bef != air_info.count)
//			{
//				air_info_count_bef = air_info.count;
//				press_avg += air_info.air_press;
//				altitude_avg += air_info.altitude;
//				avg_count ++;
//			}
//			rt_thread_delay(1);
//		}
//		second_bef = system_info.startup_second;
//		
//		sprintf(buf,"%s\t%6d\t%3.2f\t%5.3f\t%5.3f\t%3.2f\t%3.2f\t%d\n",
//										time_printf(), //时间格式
//										press_avg/avg_count, 
//										air_info.temperature.temp,
//										altitude_avg /avg_count,
//										air_info.rt_altitude,
//										air_info.rt_climb,
//										air_info.climb,
//										avg_count);	
//		avg_count = 0;
//		altitude_avg = 0;
//		press_avg = 0;
//													
//		size = write(fd, buf, strlen(buf));
//		if(size>0)
//		{
//			//rt_kprintf(buf);
//		}
//		else
//		{
//			rt_kprintf("air log: Write to file wrong !");
//			while(1)rt_thread_delay(1000);  //相当于停止这个线程
//		}
//		//大约1分钟后保存并重新打开
//		if(second_bef %64 == 0)
//		{
//			close(fd);
//			fd = open(dir, O_WRONLY | O_CREAT | O_APPEND, 0);//末尾重新打开		
//		}
//		
//	}

//}

///* thread_air_log */
//ALIGN(RT_ALIGN_SIZE)
//static char thread_air_log_stack[3072];
//static struct rt_thread thread_air_log_handle;
////air log initialization
//void thread_air_log_init(void)
//{
//	//log air
//	rt_thread_init(&thread_air_log_handle,
//                   "log air",
//                   entry_thread_air_log,
//                   RT_NULL,
//                   &thread_air_log_stack[0],
//                   sizeof(thread_air_log_stack),26,1);
//    rt_thread_startup(&thread_air_log_handle);
//}


////记录gps数据
//void entry_thread_gps_log(void* parameter)
//{	
//	int fd = -1;
//	int	size;
//	int second_bef = 1;
//	char buf[64];
//	char dir[64];
//	
//	const char file_name[] = "gps infomation.log";
//	const char instruction[] = 
//	"This is a GPS log from Deep Blue Plane.\n"
//	"DATE & TIME \tLatitude\tLongtitude\tAltitude\tSpeed\tCourse\tStars\n";
//	
//	//复制工作区 目录
//	strcpy(dir, log_dir);
//	//打开记录文件
//	strcat(dir, file_name);
//	
//	//try to open the working file
//	while(fd < 0)
//	{
//		int count = 0;		
//		fd = open(dir, O_WRONLY | O_CREAT, 0);
//		rt_thread_delay(RT_TICK_PER_SECOND);
//		
//		if ((count ++ )> 60) //60 seconds
//		{
//			rt_kprintf("gps log: open file for write failed\n");
//			return;
//		}
//	}
//	rt_kprintf("gps log has been created.\n");
//	//写入说明
//	write(fd, instruction, strlen(instruction));
//	
//	//主循环
//	while(1)
//	{
//		while(second_bef >= system_info.startup_second )
//		{
//			rt_thread_delay(5);
//		}
//		second_bef = system_info.startup_second;	   //mark for 1 second a cycle
//		
//		//等待直到数据有效
//		if(gps.flag == 0)
//			continue;
//					
//		sprintf(buf,"%s\t%f\t%f\t%5.3f\t%f\t%f\t%d\n",
//										time_printf(), //时间格式
//										gps.lat, 
//										gps.lon,
//										gps.altitude,
//										gps.speed,
//										gps.course,
//										gps.stars);	
//												
//		size = write(fd, buf, strlen(buf));
//		if(size>0)
//		{
//			//rt_kprintf(buf);
//		}
//		else
//		{
//			rt_kprintf("Write to file wrong !");
//			while(1)rt_thread_delay(1000);  //相当于停止这个线程
//		}
//		//大约20秒
//		if(second_bef %20 == 0)
//		{
//			close(fd);
//			fd = open(dir, O_WRONLY | O_CREAT | O_APPEND, 0);//末尾重新打开		
//		}
//		
//	}

//}

///* thread_gps_log */
//ALIGN(RT_ALIGN_SIZE)
//static char thread_gps_log_stack[3072];
//static struct rt_thread thread_gps_log_handle;
////gps log initialization
//void thread_gps_log_init(void)
//{
//	//gps
//	rt_thread_init(&thread_gps_log_handle,
//                   "log gps",
//                   entry_thread_gps_log,
//                   RT_NULL,
//                   &thread_gps_log_stack[0],
//                   sizeof(thread_gps_log_stack),26,1);
//    rt_thread_startup(&thread_gps_log_handle);
//}




//记录kml文件
void entry_thread_kml_log(void* parameter)
{	
	int fd = -1;
	int	size;
	int gps_count_bef = 1;
	char buf[512];
	char dir[64];
	unsigned int buf_offset = 0;		//指示上一次写到哪里
	
	unsigned long long file_byte_seek = 0;			//指示文件长度
	unsigned int head_len = 0;						//文件头尾的长度
	unsigned int tail_len = 0;
	const char file_name[] = "path.kml";
	const char head[] = 
	"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
	"<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"
	"\t<Document>\n"
	"<name>Paths</name>\n"
	"<description> DBP V0.4 KML LOG</description>\n"
	"<Style id=\"yellowLineGreenPoly\">\n"     
	"	<LineStyle>\n"       
	"		<color>7f00ffff</color>\n"      
	"		<width>4</width>\n"    
	"	</LineStyle>\n"    
	"	<PolyStyle>\n"       
	"		<color>5f00ff00</color>\n"    
	"	</PolyStyle>\n"   
	"</Style>\n"
	"<Placemark>\n"
	"	<name>Absolute Extruded</name>\n"     
	"	<description>Deep Blue Plane V0.4 Path</description>\n"      
	"	<styleUrl>#yellowLineGreenPoly</styleUrl> \n"    
	"	<LineString>\n"      
	"		<extrude>1</extrude>\n"       
	"		<tessellate>1</tessellate>\n"       //这一句是有没有竖线
	"		<altitudeMode>absolute</altitudeMode>\n"       
	"			<coordinates>\n";
	
	const char tail[] =    
	"			</coordinates>\n "    
	"	</LineString>\n"
	"	</Placemark>\n"
	"</Document>\n"
	"</kml>\n";
	
	//预先记录文件头尾长度
	head_len = strlen(head);
	tail_len = strlen(tail);
	
	//复制工作区 目录
	strcpy(dir, log_dir);
	//打开记录文件
	strcat(dir, file_name);
	
	//try to open the working file
	while(fd < 0)
	{
		int count = 0;		
		fd = open(dir, O_WRONLY | O_CREAT, 0);
		rt_thread_delay(RT_TICK_PER_SECOND);
		
		if ((count ++ )> 60) //60 seconds
		{
			rt_kprintf("kml: open file for write failed\n");
			return;
		}
	}

	rt_kprintf("google earth file'path.kml' has been created.\n");
	//写入文件头
	write(fd, head, head_len);
	file_byte_seek += head_len;
	
	//主循环
	while(1)
	{
		while(gps_count_bef >= gps.count)
		{
			rt_thread_delay(5);
		}
		gps_count_bef = gps.count;	   //mark for 1 second a cycle
		
		//等待直到数据有效
		if(gps.flag == 0)
			continue;
				
		sprintf(buf,"\t\t\t\t%f,%f,%f\n",
										gps.lon, 
										gps.lat,
										gps.altitude);	
		
		//将缓存区写入文件
		size = write(fd, buf, strlen(buf));
		file_byte_seek += size;				//记录写入长度	
		
		if(size>0)
		{
		//rt_kprintf(buf);
		}
		else
		{
			rt_kprintf("kml log: Write to file wrong!\n");
			while(1)
				rt_thread_delay(1000);  //相当于停止这个线程
		}
		
		
		//写入文件尾
		write(fd, tail, tail_len);
		close(fd);
		fd = open(dir, O_WRONLY | O_CREAT | O_APPEND, 0);//末尾重新打开		

		//覆盖 从文件头开始计算写入了多少有效数据，覆盖文件尾
		lseek(fd, file_byte_seek, DFS_SEEK_SET);
		
		
	}

}




/* thread_kml_log */
ALIGN(RT_ALIGN_SIZE)
static char thread_kml_log_stack[4096];
static struct rt_thread thread_kml_log_handle;
//gps log initialization
void thread_kml_log_init(void)
{
	//gps
	rt_thread_init(&thread_kml_log_handle,
                   "kml gps",
                   entry_thread_kml_log,
                   RT_NULL,
                   &thread_kml_log_stack[0],
                   sizeof(thread_kml_log_stack),26,1);
    rt_thread_startup(&thread_kml_log_handle);
}

/* thread_quick_log */
void entry_thread_quick_log(void* parameter)
{
	int fd = -1;
	int	size;
	int count_bef = 1;
	int systick = 0;
	char buf[512];
	char dir[64];
	const char file_name[] = "quick.log";
	const char instruction[] =
	"This is a quick log from Deep Blue Plane.\n"
	"millinsecond\t"
	"angle pitch\t"
	"angle roll\t"
	"angle yaw\t"
	"gyro pitch\t"
	"gyro roll\t"
	"gyro yaw\t"
	"acc x\t"
	"acc y\t"
	"acc z\t"
	"acc g\t"
	
	"mag x\t"
	"mag y\t"
	"mag z\t"
	"mag course\t"
	"airpress\t"
	"altitude\t"
	"climb\t"
	"ult altitude\t"
	"ult speed\t"
	
	"ppm ch1\t"
	"ppm ch2\t"
	"ppm ch3\t"
	"ppm ch4\t"
	"ppm ch5\t"
	"ppm ch6\t"
	
	"pwm ch1\t"
	"pwm ch2\t"
	"pwm ch3\t"
	"pwm ch4\t"
	"pwm ch5\t"
	"pwm ch6\t"
	
	"exp thr\t"
	"exp pitch\t"
	"exp roll\t"
	"exp yaw\t"
	"exp alt"
	"output pitch\t"
	"output roll\t"
	"output yaw\t"
	"output thr\n";
	
	//复制工作区 目录
	strcpy(dir, log_dir);
	//打开记录文件
	strcat(dir, file_name);
	
	//try to open the working file
	while(fd < 0)
	{	
		fd = open(dir, O_WRONLY | O_CREAT, 0);
		rt_thread_delay(RT_TICK_PER_SECOND);
	}

	rt_kprintf("quick log has been created.\n");
	//写入说明
	write(fd, instruction, strlen(instruction));
	
	//主循环
	while(1)
	{
		//10ms 
		while(systick +10 > rt_tick_get())
			rt_thread_delay(1);
		systick = rt_tick_get();

		{			
			sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t"
			"%4.3f\t%4.3f\t%4.3f\t%4.3f\t%7d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t"
			"%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t"
			"%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t"
			"%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t\n",
											(unsigned int)rt_tick_get(),
											out_angle.pitch,
 											out_angle.roll,
											out_angle.yaw,
											gyro.pitch,
											gyro.roll,
											gyro.yaw,
											acc.axis.x,
											acc.axis.y,
											acc.axis.z,	
											acc.g,
											
											mag.x,
											mag.y,
											mag.z,
											mag.course,
											air_info.air_press,
											air_info.altitude,
											air_info.climb,
											ult_data.altitude,
											ult_data.speed,
											
											ppm.input.ch1,
											ppm.input.ch2,
											ppm.input.ch3,
											ppm.input.ch4,
											ppm.input.ch5,
											ppm.input.ch6,
											
											ctrl.opwm.ch1,
											ctrl.opwm.ch2,
											ctrl.opwm.ch3,
											ctrl.opwm.ch4,
											ctrl.opwm.ch5,
											ctrl.opwm.ch6,
											
											expectation.throttle,
											expectation.angle.pitch,
											expectation.angle.roll,
											expectation.angle.yaw,
											expectation.altitude,
											ctrl.pitch.output.value,
											ctrl.roll.output.value,
											ctrl.yaw.output.value,
											ctrl.throttle.output.value
											);
		}
	
												
		size = write(fd, buf, strlen(buf));
		if(size>0)
		{
			//rt_kprintf(buf);
		}
		else
		{
			rt_kprintf("quick log write to file wrong !");
			while(1)rt_thread_delay(1000);  //相当于停止这个线程
		}

		if(system_info.startup_second %5 == 0)
		{
			close(fd);
			fd = open(dir, O_WRONLY | O_CREAT | O_APPEND, 0);//末尾重新打开		
		}
		
	}

}

/* thread_quick_log */
ALIGN(RT_ALIGN_SIZE)
static char thread_quick_log_stack[4096];
static struct rt_thread thread_quick_log_handle;
//gps log initialization
void thread_quick_log_init(void)
{
	rt_thread_init(&thread_quick_log_handle,
                   "log quick",
                   entry_thread_quick_log,
                   RT_NULL,
                   &thread_quick_log_stack[0],
                   sizeof(thread_quick_log_stack),25,1);
    rt_thread_startup(&thread_quick_log_handle);
}


/* thread_slow_log */
void entry_thread_slow_log(void* parameter)
{
	int fd = -1;
	int	size;
	int count_bef = 1;
	int second = 0;
	char buf[256];
	char dir[64];
	const char file_name[] = "slow.log";
	const char instruction[] =
	"This is a slow log from Deep Blue Plane.\n"
	"second\t"
	"time\t"
	"cpu\t"
	"airtemp\t"
	"battery.v\t"
	"battery.a\t"
	"gps.lng\t"
	"gps.lat\t"
	"gps.star\t"
	"ctrl.stat\t"
	"wp.lng\t"
	"wp.lat\t"
	"wp.seaalt\t"
	"airspeed\n";
	
	//复制工作区 目录
	strcpy(dir, log_dir);
	//打开记录文件
	strcat(dir, file_name);
	
	//try to open the working file
	while(fd < 0)
	{	
		fd = open(dir, O_WRONLY | O_CREAT, 0);
		rt_thread_delay(RT_TICK_PER_SECOND);
	}

	rt_kprintf("slow log has been created.\n");
	//写入说明
	write(fd, instruction, strlen(instruction));
	
	//主循环
	while(1)
	{
		//1s 
		while(second +1 > system_info.startup_second)
			rt_thread_delay(1);
		second =  system_info.startup_second;

		{			
			sprintf(buf,"%6d\t%s\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t"
			"%f\t%f\t%d\t%d\t%f\t%f\t%f\t%f\t\n",
											system_info.startup_second,
											time_printf(),
 											system_info.mcu.usage,
											air_info.temperature.temp,
											system_info.power_battery.v,
											system_info.power_battery.a,
			
											gps.lon,
											gps.lat,
											gps.stars,
											control.state,
											navigation.current.longitude,
											navigation.current.latiude,
											navigation.current.sea_altitude,
											air_speed.speed);
		}
	
												
		size = write(fd, buf, strlen(buf));
		if(size>0)
		{
			//rt_kprintf(buf);
		}
		else
		{
			rt_kprintf("slow log write to file wrong !");
			while(1)rt_thread_delay(1000);  //相当于停止这个线程
		}

		if(system_info.startup_second %5 == 1)
		{
			close(fd);
			fd = open(dir, O_WRONLY | O_CREAT | O_APPEND, 0);//末尾重新打开		
		}
		
	}

}

/* thread_slow_log */
ALIGN(RT_ALIGN_SIZE)
static char thread_slow_log_stack[4096];
static struct rt_thread thread_slow_log_handle;
//gps log initialization
void thread_slow_log_init(void)
{
	rt_thread_init(&thread_slow_log_handle,
                   "log slow",
                   entry_thread_slow_log,
                   RT_NULL,
                   &thread_slow_log_stack[0],
                   sizeof(thread_slow_log_stack),25,1);
    rt_thread_startup(&thread_slow_log_handle);
}



///* thread_test_log */
//void entry_thread_test_log(void* parameter)
//{
//	int fd = -1;
//	int	size;
//	int count_bef = 1;
//	char buf[64];
//	char dir[64];
//	const char file_name[] = "test.log";
//	const char instruction[] =
//	"This is a test log from Deep Blue Plane.\n"
//	"DATA Count\tacc.x\tacc.y\tacc.z\n";
//	
//	//复制工作区 目录
//	strcpy(dir, log_dir);
//	//打开记录文件
//	strcat(dir, file_name);
//	
//	//try to open the working file
//	while(fd < 0)
//	{
//		int count = 0;		
//		fd = open(dir, O_WRONLY | O_CREAT, 0);
//		rt_thread_delay(RT_TICK_PER_SECOND);
//		
//		if ((count ++ )> 60) //60 seconds
//		{
//			rt_kprintf("test log: open file for write failed\n");
//			return;
//		}
//	}

//	rt_kprintf("test log has been created.\n");
//	//写入说明
//	write(fd, instruction, strlen(instruction));
//	
//	//主循环
//	while(1)
//	{
//		rt_thread_delay(10);
//		sprintf(buf,"%6d\t%6d\t%4.3f\t%4.3f\t%4.3f\t\n",
// 										//gyro.count, 
//										(unsigned int)rt_tick_get(),
//										ult_data.flag,
// 										ult_data.altitude,
//										ult_data.speed,
// 										expectation.throttle
//										);
		
//		sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\n",
// 										//gyro.count, 
//										(unsigned int)rt_tick_get(),
// 										acc.axis.x, 
// 										acc.axis.y,
// 										acc.axis.z,
// 										acc.axis_rt.x,
// 										acc.axis_rt.y,
// 										acc.axis_rt.z
//										);
		

//		sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\n",
// 										//gyro.count, 
//										(unsigned int)rt_tick_get(),
// 										out_angle.pitch,
// 										out_angle.roll,
// 										out_angle.yaw,
//										gyro.pitch,
// 										gyro.roll,
// 										gyro.yaw,
//										acc.axis.x,
//										acc.axis.y,
//										acc.axis.z,
//										air_info.altitude,
//										air_info.climb
//										);
//		
//		rt_thread_delay(1);
//		sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\n",
// 										//gyro.count, 
//										(unsigned int)rt_tick_get(),
// 										gyro.pitch,
// 										gyro.roll,
// 										gyro.yaw,
//										gyro.rt_pitch,
// 										gyro.rt_roll,
// 										gyro.rt_yaw
//										);
									
 		
// 		sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\n",
// 										//gyro.count, 
//										(unsigned int)rt_tick_get(),
// 										acc.axis.x, 
// 										acc.axis.y,
// 										acc.axis.z,
// 										acc.axis_rt.x,
// 										acc.axis_rt.y,
// 										acc.axis_rt.z,
// 										gyro.pitch,
// 										gyro.roll,
// 										gyro.yaw,
//										gyro.rt_pitch,
// 										gyro.rt_roll,
// 										gyro.rt_yaw
//										);	
		
//		while((air_speed.count) <= count_bef)
//		{
//			rt_thread_delay(1);
//		}
//		count_bef = air_speed.count;
//		
//		sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\n",
//										air_speed.count, 
//										air_speed.speed, 
//										air_speed.rt_speed,
//										air_speed.press_diff
//										);	

//		sprintf(buf,"%6d\t%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\n",
//											(unsigned int)rt_tick_get(),
//											air_info.air_press,
//											air_info.altitude,
//											air_info.rt_altitude,
//											air_info.climb,
//											air_info.rt_climb,
//											air_info.temperature.temp
//											);
//			
//		{
//			extern double acc_alt;
//			extern double acc_climb;
//			
//			sprintf(buf,"%6d\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t\n",
//											(unsigned int)rt_tick_get(),
//											air_info.altitude,
//											acc_alt,
//											air_info.climb,
//											acc_climb,
//											system_info.power_battery.v,
//											system_info.power_battery.a
//											);
//		}
//	
//												
//		size = write(fd, buf, strlen(buf));
//		if(size>0)
//		{
//			//rt_kprintf(buf);
//		}
//		else
//		{
//			rt_kprintf("Test log write to file wrong !");
//			while(1)rt_thread_delay(1000);  //相当于停止这个线程
//		}

//		if(system_info.startup_second %5 == 0)
//		{
//			close(fd);
//			fd = open(dir, O_WRONLY | O_CREAT | O_APPEND, 0);//末尾重新打开		
//		}
//		
//	}

//}

///* thread_test_log */
//ALIGN(RT_ALIGN_SIZE)
//static char thread_test_log_stack[4096];
//static struct rt_thread thread_test_log_handle;
////gps log initialization
//void thread_test_log_init(void)
//{
//	rt_thread_init(&thread_test_log_handle,
//                   "log test",
//                   entry_thread_test_log,
//                   RT_NULL,
//                   &thread_test_log_stack[0],
//                   sizeof(thread_test_log_stack),26,1);
//    rt_thread_startup(&thread_test_log_handle);
//}

////解码配置文件
//void ini_file_decode(void)
//{
//	int fd,size;
//	int count_bef = 1;
//	char buf[64];
//	const char file_name[] = "/config.ini";




//}


//新建一个目录 返回目录
void working_dir_make(void)
{	
	//以时间为准命名目录
	sprintf(log_dir,"/sd/20%02d%02d%02d-%02d%02d%02d/", 
							system_info.time.year, 
							system_info.time.month,
							system_info.time.date,
							system_info.time.hour,
							system_info.time.minute,
							system_info.time.second);	

	//wait until /sd has been mount
	while(mkdir(log_dir, 0x777) < 0)
	{
		rt_thread_delay(RT_TICK_PER_SECOND /10);
	}

	
}

//初始化
void log_init(void)
{
	
	//初始化工作区
	working_dir_make();
	
//	//气压计记录
//	thread_air_log_init();
//	//GPS
//	thread_gps_log_init();
//	//KML
	thread_kml_log_init();
	
	thread_quick_log_init();
	
	thread_slow_log_init();
	
	//测试
//	thread_test_log_init();

}

//log 主程序
void thread_log(void)
{	
	//wating for the first update of the system times
	rt_thread_delay(1000);
	
	log_init();
	
	while(1)
	{
		rt_thread_delay(5);

	}
}
