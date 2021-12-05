/*---------------------------------------------------------------*
*struct_all.h for AFC V0.2
*
*����ȫ�ֱ����ı��棬���еĹؼ���ȫ�ֱ����������ﶨ��
*ͨ������ֻ��һ���߳�д�룬һ�������߳������ȡ
*
*           **** ���нṹ��ԭ��λ�� struct.c ****
*2011.10.12
*2013.1.23
*by majianjia in fantasticlab.blog.163.com
*----------------------------------------------------------------*/

#ifndef __STRUCT_ALL_H__
#define __STRUCT_ALL_H__

/* flag ֵ */
#define DATA_NOT_READY  0x00
#define DATA_READY	 	0x01

/* ahrs_frequency ��̬�������� */
#define AHRS_FREQUENCY 1000

/* PI */
#define PI 3.1415926535898


/* ʱ�� */
struct _time{
	unsigned char flag;
	
	unsigned char time_zone;		//ʱ��
	unsigned char second;
	unsigned char minute;
	unsigned char hour;
	unsigned char weekday;        	//day of week
	unsigned char date;			//date of month
	unsigned char month;
	unsigned char year;
	unsigned int milisecond;
};

/* �¶� */
struct _temperature
{
	float temp;
};

/* MCU ��Ϣ*/
struct _mcu
{
	struct _temperature temperature;
	float usage;
};

//�����Ϣ
struct _battery
{
	unsigned char flag;
	unsigned int  count;

	struct _temperature temperature;
	float v;					//��ѹ	
	float a;   					//����
	float flow;   				//�����ĵ���
	float rt;      				//ʣ��ʱ��
};
	
/* ϵͳ��Ϣ */
struct _system_info
{
	unsigned int cycle;  //ѭ������	
	unsigned int startup_second;	
	
	struct _mcu mcu;
	struct _time time;
	struct _battery power_battery;
	struct _battery backup_battery;
};
extern struct _system_info system_info;

//  ŷ���� 
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


/* ��������������ṹ�� */
struct _gyro 
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

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

/* ���ٶȼ���������ṹ�� ��λ g */
struct _acc
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

	struct _temperature temperature;
	
	struct _coordinate axis;    //���� after filter
	float g;
	
	struct _coordinate axis_rt; //real time
	
	struct _euler_angle angle;
};
extern struct _acc acc;

//ppmͨ��
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

/* ���ջ�PPM�ź� */
struct 	_ppm
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

	unsigned int valid_count;
	unsigned int duration;		//��Ч��ƽ���
	int ppm_level_inversion;	//ppm�����ƽ��ת
	
	int ppm_zero;				//��㳤�� һ����1500us
	
	struct _ppm_channel can;		//input by canbus
	struct _ppm_channel input;		//input by onboard ppm interface
	struct _ppm_channel offset;		//��� offset
	struct _ppm_channel inversion;	//�����ƽ��ת

};
extern struct _ppm ppm;


/* GPS ��Ϣ */
struct _gps
{

	unsigned char flag;
	unsigned int count;
	
	unsigned char status;		//��λ״̬
	unsigned char stars;		//������Ŀ
		
	double lat;      			//latitude
	double lon;					//longitude
	
	float altitude;				//����
	float course;				//����
	float speed;				//�ٶ� ��
	
	struct _time time;          //ʱ����Ϣ
};
extern struct _gps gps;

/* �Ƕ����ռ���ֵ */
struct _out_angle
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

	double yaw;
	double roll;
	double pitch;
	
	double rad_yaw;
	double rad_roll;
	double rad_pitch;
};
extern struct _out_angle out_angle;


/* �����ǵ�����ֵ */
struct	_gyro_offset
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

	int yaw;
	int roll;
	int pitch;
};
extern struct _gyro_offset gyro_offset;

/* ���贫��������Ƕ�ֵ */
struct _mag
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

	struct _temperature temperature;
	
	float x;
	float y;
	float z;
	
	float course;
};
extern struct _mag mag ;

//���ٶȼ�����ֵ
struct	_acc_offset
{
	unsigned char flag;			//�����Ƿ����
	unsigned int count;			//��ʾ�ڼ��θ�������

	int x;
	int y;
	int z;
};
extern struct _acc_offset acc_offset;

//�������������ṹ��
struct _ult_data
{
	unsigned int flag;
	unsigned int  count;

	float  distance;	   		//��������ʵ�ʾ���(m)
	float  altitude;			//ͨ���Ƕȼ����õ��ĸ߶� ����ֵΪ��������ˮƽ�����ʵ�ʾ��룩
	float  speed;
	
	struct _temperature temperature;  //����
};
extern struct _ult_data ult_data;

//������Ϣ
struct _air_info
{
	unsigned char flag;
	unsigned int  count;

	struct _temperature temperature;  //����
	
	unsigned int air_press;	        //��ѹ
	unsigned int offset_press;
	float  altitude;				//����
	float  offset_altitude;			//offset comes from gps
	float  rt_altitude;
	float  climb;					//������ M/S
	float  rt_climb;
};
extern struct _air_info air_info;

//����
struct _air_speed
{
	unsigned char flag;
	unsigned int  count;
	
	struct _temperature temperature;  //����
	
	float press_diff;
	float speed;
	float rt_speed;
};
extern struct _air_speed air_speed;

//pid������
struct _pid_output
{
	unsigned char flag;
	unsigned int  count;	
	
	float sens;
	float value;

};
extern struct _pid_output output;		//���

//�����㷨PID
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


//�����㷨
struct _ctrl
{
	struct _pid pitch;
	struct _pid roll;
	struct _pid yaw;
	struct _pid throttle;
	
	struct _ppm_channel opwm;

};
extern struct _ctrl ctrl;


//����������״̬
struct _expectation
{
	unsigned char flag;
	unsigned int  count;

	struct _euler_angle angle;
	float throttle;				//�������� 0~100
	float altitude;				//�߶�
	
};
extern struct _expectation expectation;

//�ɻ����
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




//���������Ʒ�ʽ
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
	
	unsigned int state;				//����״̬��passby ���� ���� ���� �ؼ�	
	
};
extern struct _control control;



#endif


