/*---------------------------------------------------------------*
*struct_all.h for AFC V0.2
*
*用于全局变量的保存，所有的关键的全局变量将在这里定义
*通常他们只有一个线程写入，一个或多个线程任意读取
*
*           **** 所有结构体原型位于 struct.c ****
*2011.10.12
*2013.1.23
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/

#ifndef __STRUCT_ALL_H__
#define __STRUCT_ALL_H__

/* flag 值 */
#define DATA_NOT_READY  0x00
#define DATA_READY	 	0x01

/* ahrs_frequency 姿态计算周期 */
#define AHRS_FREQUENCY 1000

/* PI */
#define PI 3.1415926535898


/* 时间 */
struct _time{
	unsigned char flag;
	
	unsigned char time_zone;		//时区
	unsigned char second;
	unsigned char minute;
	unsigned char hour;
	unsigned char weekday;        	//day of week
	unsigned char date;			//date of month
	unsigned char month;
	unsigned char year;
	unsigned int milisecond;
};

/* 温度 */
struct _temperature
{
	float temp;
};

/* MCU 信息*/
struct _mcu
{
	struct _temperature temperature;
	float usage;
};

//电池信息
struct _battery
{
	unsigned char flag;
	unsigned int  count;

	struct _temperature temperature;
	float v;					//电压	
	float a;   					//电流
	float flow;   				//已消耗电流
	float rt;      				//剩余时间
};
	
/* 系统信息 */
struct _system_info
{
	unsigned int cycle;  //循环次数	
	unsigned int startup_second;	
	
	struct _mcu mcu;
	struct _time time;
	struct _battery power_battery;
	struct _battery backup_battery;
};
extern struct _system_info system_info;

//  欧拉角 
struct _euler_angle
{
  	double roll;
	double pitch;
	double yaw;
};

//
struct _coordinate
{
	float x;
	float y;
	float z;
};


/* 陀螺仪数据输出结构体 */
struct _gyro 
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	struct _temperature temperature;
	double yaw;
	double roll;
	double pitch;
	
	double rad_yaw;
	double rad_roll;
	double rad_pitch;
	
	double rt_yaw;
	double rt_roll;
	double rt_pitch;
	
};
extern struct _gyro gyro;

/* 加速度计数据输出结构体 单位 g */
struct _acc
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	struct _temperature temperature;
	
	struct _coordinate axis;    //坐标 after filter
	float g;
	
	struct _coordinate axis_rt; //real time
	
	struct _euler_angle angle;
};
extern struct _acc acc;

//ppm通道
struct _ppm_channel
{
	unsigned int flag;
	unsigned int count;
	int ch1;
	int ch2;
	int ch3;
	int ch4;
	int ch5;
	int ch6;
	int ch7;
	int ch8;
	int ch9;
	int ch10;
	int ch11;
	int ch12;
	int ch13;
	int ch14;
	int ch15;
	int ch16;
};

/* 接收机PPM信号 */
struct 	_ppm
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	unsigned int valid_count;
	unsigned int duration;		//有效电平宽度
	int ppm_level_inversion;	//ppm输入电平反转
	
	int ppm_zero;				//零点长度 一般是1500us
	
	struct _ppm_channel can;		//input by canbus
	struct _ppm_channel input;		//input by onboard ppm interface
	struct _ppm_channel offset;		//输出 offset
	struct _ppm_channel inversion;	//输出电平反转

};
extern struct _ppm ppm;


/* GPS 信息 */
struct _gps
{

	unsigned char flag;
	unsigned int count;
	
	unsigned char status;		//定位状态
	unsigned char stars;		//卫星数目
		
	double lat;      			//latitude
	double lon;					//longitude
	
	float altitude;				//海拔
	float course;				//航向
	float speed;				//速度 节
	
	struct _time time;          //时间信息
};
extern struct _gps gps;

/* 角度最终计算值 */
struct _out_angle
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	double yaw;
	double roll;
	double pitch;
	
	double rad_yaw;
	double rad_roll;
	double rad_pitch;
};
extern struct _out_angle out_angle;


/* 陀螺仪的修正值 */
struct	_gyro_offset
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	int yaw;
	int roll;
	int pitch;
};
extern struct _gyro_offset gyro_offset;

/* 磁阻传感器输出角度值 */
struct _mag
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	struct _temperature temperature;
	
	float x;
	float y;
	float z;
	
	float course;
};
extern struct _mag mag ;

//加速度计修正值
struct	_acc_offset
{
	unsigned char flag;			//数据是否可用
	unsigned int count;			//显示第几次更新数据

	int x;
	int y;
	int z;
};
extern struct _acc_offset acc_offset;

//超声波测距输出结构体
struct _ult_data
{
	unsigned int flag;
	unsigned int  count;

	float  distance;	   		//测量出的实际距离(m)
	float  altitude;			//通过角度计算后得到的高度 （此值为飞行器与水平地面的实际距离）
	float  speed;
	
	struct _temperature temperature;  //气温
};
extern struct _ult_data ult_data;

//空气信息
struct _air_info
{
	unsigned char flag;
	unsigned int  count;

	struct _temperature temperature;  //气温
	
	unsigned int air_press;	        //气压
	unsigned int offset_press;
	float  altitude;				//海拔
	float  offset_altitude;			//offset comes from gps
	float  rt_altitude;
	float  climb;					//爬升率 M/S
	float  rt_climb;
};
extern struct _air_info air_info;

//空速
struct _air_speed
{
	unsigned char flag;
	unsigned int  count;
	
	struct _temperature temperature;  //气温
	
	float press_diff;
	float speed;
	float rt_speed;
};
extern struct _air_speed air_speed;

//pid计算结果
struct _pid_output
{
	unsigned char flag;
	unsigned int  count;	
	
	float sens;
	float value;

};
extern struct _pid_output output;		//输出

//控制算法PID
struct _pid 
{
	unsigned char flag;
	unsigned int  count;
	
	float kp;
	float ki;
	float kd;
	float integral;
	float integral_max;				//integral max
	float increment;
	float increment_max;				//single increment max limit
	float increment_min;				//single increment min limit
	

	struct _pid_output output;
};
extern struct _pid pid;


//控制算法
struct _ctrl
{
	struct _pid pitch;
	struct _pid roll;
	struct _pid yaw;
	struct _pid throttle;
	
	struct _ppm_channel opwm;

};
extern struct _ctrl ctrl;


//飞行器期望状态
struct _expectation
{
	unsigned char flag;
	unsigned int  count;

	struct _euler_angle angle;
	float throttle;				//期望油门 0~100
	float altitude;				//高度
	
};
extern struct _expectation expectation;

//飞机相关
struct _plane
{
	unsigned char flag;
	unsigned int  count;
	
	struct _euler_angle angle_offset;


};
extern struct _plane plane;


//a struct for descripting a position of geography
struct _way_point
{
	struct _way_point *next;//next waypoint
	unsigned int id;		//the id of waypoint
	double longitude; 		//jingdu
	double latiude;	  		//weidu
	double sea_altitude;  	//haibagaodu
	double floor_altitude; 	//from ultrasionic ranging module
};

struct _navigation
{
	struct _way_point *list_head;		//waypoint list
	struct _way_point *list_current;
	struct _way_point *list_end;
	
	struct _way_point current;			//current point

};
extern struct _navigation navigation;

//optical flow
struct _opt_flow
{
	unsigned char flag;
	unsigned int count;
	
	float x;
	float y;
	
};
extern struct _opt_flow opt_flow;




//飞行器控制方式
#define CONTROL_STATE_PASSBY 		(0x0001 << 0)
#define CONTROL_STATE_STABLE 		(0x0001 << 1)
#define CONTROL_STATE_WP 			(0x0001 << 2)
#define CONTROL_STATE_HOVER 		(0x0001 << 3)
#define CONTROL_STATE_GOHOME		(0x0001 << 4)

#define CONTROL_STATE_AP_PITCH		(0x0001 << 5)
#define CONTROL_STATE_AP_ROLL		(0x0001 << 6)
#define CONTROL_STATE_AP_COURSE 	(0x0001 << 7)
#define CONTROL_STATE_AP_THROTTLE 	(0x0001 << 8)
#define CONTROL_STATE_AP_ALL    (CONTROL_STATE_AP_PITCH | CONTROL_STATE_AP_ROLL | CONTROL_STATE_AP_COURSE |CONTROL_STATE_AP_THROTTLE)

struct _control
{
	unsigned char flag;
	unsigned int  count;
	
	unsigned int state;				//控制状态：passby 增稳 航线 盘旋 回家	
	
};
extern struct _control control;



#endif


