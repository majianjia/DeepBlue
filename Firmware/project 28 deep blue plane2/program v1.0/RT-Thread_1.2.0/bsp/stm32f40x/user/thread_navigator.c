/*
 * File      : thread_navigator.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-04    majianjia   the first version
 */
 
#include "stm32f4xx.h"
#include <rtthread.h>
#include "math.h"

#include "struct_all.h"
#include "thread_navigator.h"

extern void led_green_flash(int time_on, int time_off,  unsigned int flag);
extern void led_green_set(int a);
	
	
#define NAVIGATOR_UPDATE_RATE 100
#define ROLL_LIMIT		15
#define PITCH_LIMIT 	15
//the lilt limit in auto pilot mode
#define TILT_LIMIT_MAX 	5
//the throttle limit in auto pilot mode
#define THR_DELTA_LIMIT_MAX  200


//current waypoint
struct _way_point current_point;
struct _way_point target_point;

//
struct _vector 
{
	float angle; 	//the difference of flight course and expect course -180~+180
	float height; 	//the difference between current altitude and expect altitude
	float distance; //the distance of current point to taget point
}current_vector;



#define PI 3.1415926535898 
#define EARTH_RADIUS  6378137
double rad(double angle)
{
	return (angle * PI / 180.0);
}
double degree(double rad)
{
	return (rad / PI * 180.0);
}


double get_distance(double lot1, double lat1, double lot2, double lat2)
{
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = rad(lot1) - rad(lot2);
	double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
	s = s * EARTH_RADIUS;
//	s = round(s * 10000) / 10000;
	return s;
}


double get_course(double dest_lng, double dest_lat, double src_lng, double src_lat)
{
    //start point(fStartPtx, fStartPty)  end point(dest_lat,dest_lng)
    // course direction.
    //input 
    //course :degree

	#define FALSE 0
	#define TRUE  1
	
	const double e = 0.081813369; 
	
    int go_east_flag = FALSE, go_north_flag = FALSE;
	double delta_fy   = dest_lat-src_lat;
    double delta_lnmg = dest_lng-src_lng;

	//if input is illegal(invalid)
    if(src_lat>90.0f||src_lat<-90.0f||src_lng>180.0f
        ||src_lng<-180.0f||dest_lat>90.0f||dest_lat<-90.0f
        ||dest_lng>180.0f||dest_lng<-180.0f)
    {
        return -1;
    }
	
    //latitude over 180
    if(delta_lnmg < -180.0)
        delta_lnmg += 360.0;
    if(delta_lnmg > 180.0)
        delta_lnmg -= 360.0;
	

	//delta_lnmg > 0.0 WEST---> EAST  delta_lnmg < 0.0 EAST ---> WEST
    if(delta_lnmg >= 0.0)
        go_east_flag = TRUE;
    else
        go_east_flag = FALSE;
	
    //delta_fy > 0.0 SOUTH ---> NORTH  delta_fy < 0.0 NORTH ---> SOUTH
    if(delta_fy >= 0.0)
        go_north_flag = TRUE;
    else
        go_north_flag = FALSE;
     
    if(delta_fy == 0)
    {
        if(delta_lnmg == 0)
			return 0;
        return go_east_flag?90:270;
    }
	
	{
		double d1=7915.7045*(e/2*log10((1-e*sin(src_lat*PI/180))/(1+e*sin(src_lat*PI/180)))
			+log10(tan((45+src_lat/2)*PI/180.0)));//纬度渐长率
		 
		double d2=7915.7045*(e/2*log10((1-e*sin(dest_lat*PI/180))/(1+e*sin(dest_lat*PI/180)))
			+log10(tan((45+dest_lat/2)*PI/180.0)));//纬度渐长率

		double delta_d = d2 - d1;//纬度渐长率(minute)
		
		double course = atan(delta_lnmg * 60 / delta_d) * 180 / PI;
		if(!go_east_flag && go_north_flag)	course = 360 + course;
		if(!go_east_flag && !go_north_flag)	course = 180 + course;
		if(go_east_flag && !go_north_flag)	course = 180 + course;
		
		return course;
	}
}

//get vecter from two point							 	 _______________________
//														|						|
//														V						|	
struct _vector vecter_from_waypoint( struct _way_point dest, struct _way_point src)
{
	struct _vector out;
	
	//lenght 
	out.distance = get_distance(dest.longitude, dest.latiude, src.longitude, src.latiude);
	
	//course
	out.angle = get_course(dest.longitude, dest.latiude, src.longitude, src.latiude);
	
	//height
	out.height = dest.sea_altitude - src.sea_altitude;
	
	
	return out;
}


//
void update_current_point(void)
{
	//if gps data is good
	if(gps.flag != 0)
	{
		current_point.longitude = gps.lon;
		current_point.latiude = gps.lat;
	}
	//use air press to calculate the sea level alitiude
	if(air_info.flag != 0)
		current_point.sea_altitude = air_info.altitude;
	
	//if ultrasonic ranging data is good
	if(ult_data.flag != 0)
		current_point.floor_altitude = ult_data.altitude;
}

void target_position_from_current(void)
{
	target_point.latiude = current_point.latiude;
	target_point.longitude = current_point.longitude;
}

void target_altitude_from_current(void)
{
	target_point.sea_altitude = current_point.sea_altitude;
	target_point.floor_altitude = current_point.floor_altitude;
}





struct _pid litl_pid ;
struct _pid ult_pid ;
struct _pid airpress_pid ;

void litl_pid_init(void)
{
	litl_pid.kp = 0.4;
	litl_pid.ki = 0;
	litl_pid.kd = 0;
	
	litl_pid.output.value = 0;

} 

void ult_pid_init(void)
{
	ult_pid.kp = 55;
	ult_pid.ki = 0;
	ult_pid.kd = 20;
	
	ult_pid.output.value = 0;

}

void airpress_pid_init(void)
{
	airpress_pid.kp = 15;
	airpress_pid.ki = 0;
	airpress_pid.kd = 6;
	
	airpress_pid.output.value = 0;

}




//calculate the litl of quadcopter which the direction is the course of vector
void litl_pid_computer()
{
	double litl;
	//the litl value is the result of the distance from current waypoint to taget waypoint 
	litl = current_vector.distance * litl_pid.kp;
	
	if(litl > TILT_LIMIT_MAX)litl = TILT_LIMIT_MAX;
	if(litl < -TILT_LIMIT_MAX)litl = -TILT_LIMIT_MAX;
		
	litl_pid.output.value = litl;
}



float ult_pid_computer(float exp_height, float current_height,  float current_speed)
{
	float height = (exp_height - current_height);
	float delta = 0;
	
	#define HEIGHT_LIMIT 1 //meters
	if(height > HEIGHT_LIMIT) height = HEIGHT_LIMIT;
	if(height < -HEIGHT_LIMIT) height = -HEIGHT_LIMIT;
	
	#define SPEED_LIMIT 1 //meters
	if(current_speed > SPEED_LIMIT) current_speed = SPEED_LIMIT;
	if(current_speed < -SPEED_LIMIT) current_speed = -SPEED_LIMIT;
	
	//the litl value is the result of the distance from current waypoint to taget waypoint 
	delta = (height * ult_pid.kp - current_speed * ult_pid.kd);	
	
	if(delta > THR_DELTA_LIMIT_MAX)	 delta = THR_DELTA_LIMIT_MAX;
	if(delta < -THR_DELTA_LIMIT_MAX) delta = -THR_DELTA_LIMIT_MAX;
		
	ult_pid.output.value = delta;
	
	return ult_pid.output.value;
}

float airpress_pid_computer(float exp_height, float current_height,  float current_speed)
{
	float height = (exp_height - current_height);
	float delta = 0;
	
	#undef HEIGHT_LIMIT
	#define HEIGHT_LIMIT 3 //meters
	if(height > HEIGHT_LIMIT) height = HEIGHT_LIMIT;
	if(height < -HEIGHT_LIMIT) height = -HEIGHT_LIMIT;
	
	#undef SPEED_LIMIT
	#define SPEED_LIMIT 3 //meters
	if(current_speed > SPEED_LIMIT) current_speed = SPEED_LIMIT;
	if(current_speed < -SPEED_LIMIT) current_speed = -SPEED_LIMIT;
	
	//the litl value is the result of the distance from current waypoint to taget waypoint 
	delta = (height * airpress_pid.kp - current_speed * airpress_pid.kd);	
	
	if(delta > THR_DELTA_LIMIT_MAX)	 delta = THR_DELTA_LIMIT_MAX;
	if(delta < -THR_DELTA_LIMIT_MAX) delta = -THR_DELTA_LIMIT_MAX;
		
	airpress_pid.output.value = delta;
	
	return airpress_pid.output.value;
}

double acc_alt = 0;
double acc_climb = 0;
double acc_zero = 1.024 *9.8;

void acc_altitude_intergral(int flag)
{
	double z = acc.axis.z * 9.8;
	
	//zero 
	if((flag == DISABLE) || (system_info.startup_second < 5))
	{
		acc_alt = air_info.altitude;
		acc_climb = air_info.climb;
		
		//check if data is good
//		acc_zero = z;
//		if(acc_zero > 10 || acc_zero < 9.6)
//			acc_zero = 9.8;
		
		return;
	}

	//if start intergral
	if(flag == ENABLE)
	{
		double temp;
		double diff;
		
		//if(((z - acc_zero) < 0.1) && ((z - acc_zero) > -0.1))
		//	return;
		
		temp = acc_climb + (z - acc_zero) / NAVIGATOR_UPDATE_RATE;
		acc_climb = temp*0.991 +  air_info.climb * 0.009;

		temp = acc_alt + 0.5 * (z - acc_zero) / (NAVIGATOR_UPDATE_RATE*NAVIGATOR_UPDATE_RATE) 
					+ acc_climb / NAVIGATOR_UPDATE_RATE ;
		
		//diff = temp - acc_alt
		acc_alt = temp * 0.991 + air_info.altitude * 0.009; 
	} 


}



//高度PID计算
float altitude_pid_computer(int ppm)
{	
	static float exp_throttle = -500;
	static float ult_lock_height = 0;
	static float airpress_lock_height = 0;
	static float steady_throttle = -500;
	
	//系统刚刚初始化，高度等于当前高度
	if(system_info.startup_second < 3)
	{
		//expectation.altitude  = air_info.altitude;  
		acc_altitude_intergral(DISABLE);
	}
	else
	{
		//throttle auto
		if(control.state & CONTROL_STATE_AP_THROTTLE)
		{	
			//if ultrasonic data is good
			if(ult_data.flag != 0 && ult_data.altitude < 2.0f   &&    1)
			{
				float throttle_delta = 0;
				
				//use acc intergral
				acc_altitude_intergral(ENABLE);
				
				//throttle_delta = ult_pid_computer(ult_lock_height, ult_data.altitude, ult_data.speed);
				throttle_delta = ult_pid_computer(0.5, ult_data.altitude, ult_data.speed);
		
				exp_throttle = steady_throttle+throttle_delta;
				
				if(exp_throttle > 200)
					exp_throttle = 200;
				if(exp_throttle < -500)
					exp_throttle = -500;
				
				return (float)exp_throttle;
			}
			// ultra sonic not good
			else
			{
				float throttle_delta = 0;
				
				//
				acc_altitude_intergral(ENABLE);
				
				//throttle_delta = airpress_pid_computer(airpress_lock_height, air_info.altitude, air_info.climb);
				//used acc integral to 
				throttle_delta = airpress_pid_computer(airpress_lock_height, acc_alt, acc_climb);
				
				exp_throttle = steady_throttle+throttle_delta;
				
				if(exp_throttle > 200)
					exp_throttle = 200;
				if(exp_throttle < -500)
					exp_throttle = -500;
				
				return (float)exp_throttle;
					
			}

		}
		//manual
		else
		{	
			//acc altitude disable
			//acc_altitude_intergral(DISABLE);
			acc_altitude_intergral(ENABLE);
			
			//update current altitude for hold altitude mode
			ult_lock_height = ult_data.altitude;
			airpress_lock_height = acc_alt;//air_info.altitude;
			
			//update exp_throttle from current ppm input throttle
			exp_throttle = ppm;
			
			//mark steady throttle from now ppm 
			steady_throttle = exp_throttle;
		}
	}

	return (float)exp_throttle;
}

float pitch_computer(int ppm)
{
	static float stable_pitch = 0;	
	
	//auto
	if(control.state & CONTROL_STATE_AP_PITCH)
	{	
		//get the pid result and the mapping of pitch
		double diff_yaw;
		double vector_angle;
		//double pitch;

		//0~360 conver to 180 ~ -180
		vector_angle = current_vector.angle;
		if(vector_angle > 180)vector_angle -= 360; 
		
		//get the difference between vecter angle and output angle yaw
		diff_yaw = (vector_angle - (out_angle.yaw - plane.angle_offset.yaw));
		if(diff_yaw  >  180.0)diff_yaw  -= 360.0;			
		if(diff_yaw  <= -180.0)diff_yaw += 360.0;
		
		//get the mapping result	
		stable_pitch = -cos(rad(diff_yaw))  * litl_pid.output.value;;
		
		return (float)stable_pitch;
	}
	//manual
	else
	{				
		stable_pitch = ppm * 0.08f;//ppm output 
		
		if(stable_pitch  >  PITCH_LIMIT)stable_pitch = PITCH_LIMIT;			//角度变换
		if(stable_pitch  < -PITCH_LIMIT)stable_pitch = -PITCH_LIMIT;		
	}
	
	return (float)stable_pitch;	
}


//roll期望角度的PID计算 
float roll_computer(int ppm)
{
	static float stable_roll = 0;			//稳定时设置的角度

	//roll auto
	if(control.state & CONTROL_STATE_AP_ROLL)
	{	
		//get the pid result and the mapping of roll
		//(always negative because copter should litl reverse side of the distance)
		double diff_yaw;
		double vector_angle;
		
		//0~360 conver to 180 ~ -180
		vector_angle = current_vector.angle;
		if(vector_angle > 180)vector_angle -= 360; 
		
		//get the difference between vecter angle and output angle yaw
		diff_yaw = (vector_angle - (out_angle.yaw - plane.angle_offset.yaw));
		if(diff_yaw  >  180.0)diff_yaw  -= 360.0;			
		if(diff_yaw  <= -180.0)diff_yaw += 360.0;
		
		//get the mapping result	
		stable_roll = sin(rad(diff_yaw)) * litl_pid.output.value;
		
		return (float)stable_roll;
	}
	//roll manual
	else
	{				
		stable_roll = ppm * 0.08f;//ppm output 
		
		if(stable_roll  >  ROLL_LIMIT)stable_roll = ROLL_LIMIT;			//角度变换
		if(stable_roll  < -ROLL_LIMIT)stable_roll = -ROLL_LIMIT;		
	}
	
	return (float)stable_roll;	
}

//yaw期望角度的PID计算
float yaw_computer(int ppm)
{
	static float stable_course = 0;			//稳定时设置的角度
	
	//ppm zero dead zone
	if(ppm < 40 && ppm > -40)
		ppm = 0;
	
	//
	if((system_info.startup_second < 5 )&& stable_course == 0)
	{
		stable_course = out_angle.yaw;
	}

	//auto
	if(control.state & CONTROL_STATE_AP_COURSE)
	{	
		return (float)stable_course;
	}
	//manual
	else
	{			
		//ppm have output then
		if(ppm != 0)
		{
			stable_course = out_angle.yaw + ppm * 0.08f ;/// (float)NAVIGATOR_UPDATE_RATE;//ppm output 
		
			if(stable_course  >  180.0)stable_course  -= 360.0;			//角度变换
			if(stable_course  <= -180.0)stable_course += 360.0;		
		}
		
		//no throttle output 
		if(expectation.throttle < -400)
			stable_course = out_angle.yaw;
	}
	
	return (float)stable_course;
}

void check_ppm_altitude_hold_sw(int ppm)
{
	//if altitude hold on
	if(ppm >200)
	{
		//only form first time
		if(!(control.state & (CONTROL_STATE_AP_THROTTLE)))
		{
			target_altitude_from_current();
			control.state |= CONTROL_STATE_AP_THROTTLE;
		}
		
		
	}
	//OFF clear these bits
	else
	{
		if((control.state & (CONTROL_STATE_AP_THROTTLE)))
		{
			control.state &= ~CONTROL_STATE_AP_THROTTLE;
		}
	} 


}

//check the ppm auto pilot switch
void check_ppm_ap_sw(int ppm)
{	
	//if AP on
	if(ppm >200)
	{
		//only form first time
		if(!(control.state & (CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_COURSE | CONTROL_STATE_AP_ROLL)))
		{
			target_position_from_current();
		
			control.state |= (CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_COURSE | CONTROL_STATE_AP_ROLL);
		
		}
		//led on
		led_green_set(1);
		
	}
	//OFF clear these bits
	else
	{
		if((control.state & (CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_COURSE | CONTROL_STATE_AP_ROLL)))
		{
			control.state &= ~(CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_COURSE | CONTROL_STATE_AP_ROLL);
		
		}
		//FLASH LED
		led_green_flash(200, 2000, ENABLE);
		
	}

}




//计算期望值
void expectation_computer(void)
{
	expectation.angle.pitch = pitch_computer(-ppm.input.ch2);
	expectation.angle.yaw 	= yaw_computer(ppm.input.ch1);
	expectation.angle.roll 	= roll_computer(ppm.input.ch4);
	expectation.throttle 	= altitude_pid_computer(ppm.input.ch3);
}

//struct _way_point target_test, current_test;

//if transmiter off
unsigned int trans_flag = 1;

void key_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Configure  */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

double intx,inty,intz;
double speedx,speedy,speedz;

void position_reset(void)
{
	intx = 0;
	inty = 0;
	intz = 0;
	speedx = 0;
	speedy = 0;
	speedz = 0;
}

void position_integral(void)
{
	speedx += (acc.axis.x /(double)NAVIGATOR_UPDATE_RATE);
	speedy += (acc.axis.y /(double)NAVIGATOR_UPDATE_RATE);
	speedz += ((acc.axis.z-1) /(double)NAVIGATOR_UPDATE_RATE);
	
	intx += (speedx + 0.5* acc.axis.x * acc.axis.x) /(double)NAVIGATOR_UPDATE_RATE;
	inty += (speedy + 0.5* acc.axis.y * acc.axis.y) /(double)NAVIGATOR_UPDATE_RATE;	
	intz += (speedz + 0.5* (acc.axis.z-1) * (acc.axis.z-1)) /(double)NAVIGATOR_UPDATE_RATE;
}


double exp_x;
double exp_y;


//任务执行器
void thread_navigator(void)
{	
	key_init();
	
	litl_pid_init();
	ult_pid_init();
	airpress_pid_init();
	
	//if key low
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == RESET)
	{
		trans_flag = 0;
		while((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == RESET))
			rt_thread_delay(10);
	}
	
	while(1)
	{
		rt_thread_delay(RT_TICK_PER_SECOND/NAVIGATOR_UPDATE_RATE);
		
		//update current point
		update_current_point();
		
		//if RC MENULER is on
		if(trans_flag == 1)
		{
			//just for test
			check_ppm_altitude_hold_sw(ppm.input.ch5);
			check_ppm_ap_sw(ppm.input.ch6);
			
				
			//current_vector = vecter_from_waypoint(target_test, current_test);//for test
			current_vector = vecter_from_waypoint(target_point, current_point);
			
			//get litl pid value
			litl_pid_computer();
				
			//calculate the expected angle
			expectation_computer();
		}
		//if off
		else
		{
			static int lock_flag = 0;
			static int start_flag = 0;
			static float lock_x, lock_y, lock_z;
			static float pre_x, pre_y;
			static unsigned int time_start = 0;
			static unsigned int precess_time = 0;
			static float lock_throttle = -500;
			float diff_x, diff_y;
			float speed_x, speed_y;
			
				
			if(!(control.state & (CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_COURSE | CONTROL_STATE_AP_ROLL | CONTROL_STATE_AP_THROTTLE)))
			{
				control.state &= ~(CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_COURSE | CONTROL_STATE_AP_ROLL | CONTROL_STATE_AP_THROTTLE);
				//led on
				led_green_set(1);
			}
			
			//position_integral();
			
			//key down
			while(start_flag == 0)
			{
				if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == RESET)
				{
					while((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == RESET))
						rt_thread_delay(10);
					//if start
					start_flag = 1;
					//lock start time
					time_start = system_info.startup_second;
					
					//lock heading
					lock_z = out_angle.yaw;
					
					//throttle down
					expectation.throttle  = -500;
				}
				
			}
			
			//get the precess time
			precess_time = system_info.startup_second - time_start;
			
			//wait 5second from key down, then start engine
			//up
			if(precess_time >=3 && precess_time < 10)
			{	
				if(ult_data.altitude > 0.05)
				{
					expectation.throttle = lock_throttle + ult_pid_computer(0.5, ult_data.altitude, ult_data.speed);
				}
				else
				{
					lock_throttle += 1;
					if(lock_throttle > 0) lock_throttle = 0;
					
					expectation.throttle = lock_throttle;
				}
				
			}
			//down 1
			else if(precess_time >= 10 && precess_time <11)
			{
				if(ult_data.altitude > 0.05)
				{
					expectation.throttle = lock_throttle + ult_pid_computer(0.3, ult_data.altitude, ult_data.speed);
				}
				else
				{
					lock_throttle -= 1;
					if(lock_throttle < -500) lock_throttle = -500;
					expectation.throttle = lock_throttle;
				}
	
			}
			//down 2
			else if(precess_time >= 11 && precess_time <12)
			{
				if(ult_data.altitude > 0.05)
				{
					expectation.throttle = lock_throttle + ult_pid_computer(0.1, ult_data.altitude, ult_data.speed);
				}
				else
				{
					lock_throttle -= 1;
					if(lock_throttle < -500) lock_throttle = -500;
					expectation.throttle = lock_throttle;
				}
			}
			//down 3
			else if(precess_time >= 12 )
			{
				if(ult_data.altitude > 0.05)
				{
					expectation.throttle = lock_throttle + ult_pid_computer(0.0, ult_data.altitude, ult_data.speed);
					lock_throttle -=1;
				}
				else
				{
					lock_throttle -= 100;
					if(lock_throttle < -500) lock_throttle = -500;
					expectation.throttle = lock_throttle;
				}
			}
			
			
			
			
			//height enough
			if(ult_data.altitude > 0.30)
			{
				//first time  lock the position
				if(lock_flag == 0)
				{
					lock_x = opt_flow.x;
					lock_y = opt_flow.y;
					lock_flag = 1;
				}
				#define P  0.1
				#define D  0.01
				
				diff_x = lock_x - opt_flow.x;
				diff_y = lock_y - opt_flow.y;
				
				speed_x = opt_flow.x - pre_x;
				speed_y = opt_flow.y - pre_y;
				
				pre_x = opt_flow.x;
				pre_y = opt_flow.y;
				
				exp_x =  diff_x * P + speed_x * D;
				exp_y =  diff_y * P + speed_y * D;
				
				expectation.angle.pitch  = exp_y;
				expectation.angle.roll   = exp_x;
				expectation.angle.yaw    = lock_z;
			
			}
			//not 
			else
			{
				lock_flag = 0;
			
			}
		}
	

	}

}

