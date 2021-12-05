/*
 * File      : thread_controller.c
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-06-04    majianjia   the first version
 */
 
#include "stm32f4xx.h"
#include <rtthread.h>
#include "finsh.h"

#include "struct_all.h"
#include "thread_controller.h" 
#include "thread_canbus.h"

#define CTRL_FREQUENCY 1000

static struct rt_semaphore sem;
static struct rt_timer timer;
static void timer_timeout(void *p)
{
	rt_sem_release(&sem);
}
static void timer_init(void)
{
	//periodic sem for controller thread update
	rt_sem_init(&sem, "ctrl", 0, RT_IPC_FLAG_FIFO);
	
	//periodic timer for the controller's update rate
	rt_timer_init(&timer, "ctrl", timer_timeout, RT_NULL, 
					RT_TICK_PER_SECOND/CTRL_FREQUENCY , //the rate is calculate by system tick and update frequency
					RT_TIMER_FLAG_PERIODIC);
	rt_timer_start(&timer);
}


void xcopter_pid_init(void)
{
	ctrl.pitch.kp = 2.3;
	ctrl.pitch.ki = 0;
	ctrl.pitch.kd = 0.5;
	ctrl.pitch.integral_max = 150;
	ctrl.pitch.increment_max = 300;
	ctrl.pitch.increment_min = 5;
	ctrl.pitch.output.sens = 1;
	ctrl.pitch.output.value = 0;
	
	ctrl.roll.kp = 2.3;
	ctrl.roll.ki = 0;
	ctrl.roll.kd = 0.5;
	ctrl.roll.integral_max = 100;
	ctrl.roll.increment_max = 20;
	ctrl.roll.increment_min = 5;
	ctrl.roll.output.sens = 1;
	ctrl.roll.output.value = 0;
	
	ctrl.yaw.kp = 3;
	ctrl.yaw.ki = 0;
	ctrl.yaw.kd = 0.8;
	ctrl.yaw.integral_max = 100;
	ctrl.yaw.increment_max = 100;
	ctrl.yaw.increment_min = 1;
	ctrl.yaw.output.sens = 1;
	ctrl.yaw.output.value = 0;
	
}


//��ʼ�����
void controller_sw_init(void)
{
	
	xcopter_pid_init();
	
}

// TIM4 TIM3 TIM1 for 12channel
void controller_hw_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

    //GPIO Periph clock enable 
    RCC_AHB1PeriphClockCmd(	  RCC_AHB1Periph_GPIOA 
							| RCC_AHB1Periph_GPIOC
							| RCC_AHB1Periph_GPIOD
							, ENABLE);
	
	//PWM 1 2 3 4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	//PWM 5 6 7 8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	
	//PWM  9 10 11 12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);	
	

	TIM_DeInit(TIM4);//��ʼ��TIM4�Ĵ���
	TIM_DeInit(TIM3);//��ʼ��TIM3�Ĵ���
	TIM_DeInit(TIM1);//��ʼ��TIM1�Ĵ���
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 20 * 1000 -1; //50HZ
//	TIM_TimeBaseStructure.TIM_Period = 3 * 1000 -1; //333HZ
	/*��system_stm32f4xx.c�����õ�APB1 Prescaler = 4 ,��֪
	*APB1ʱ��Ϊ168M/4*2,��Ϊ���APB1��Ƶ��Ϊ1����ʱʱ��x2
	*/
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //APB 84M / ���50HZ����
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 168-1; //TIM1 APB2 168M / ���50HZ����
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/*��������Ƚϣ�����ռ�ձ�Ϊ50%��PWM����*/
	TIM_OCStructInit(&TIM_OCInitStructure);  			//����ȱʡֵ�����к���ֻ��TIM1��TIM8��Ч
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 	//PWM1Ϊ����ռ�ձ�ģʽ��PWM2Ϊ������ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //HighΪռ�ձȸ߼���	
	TIM_OCInitStructure.TIM_Pulse = 1500; 				//����CCR��ռ�ձ���ֵ��
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);			//
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);			//
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);			//
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);			//
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);			//
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);			//
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);			//
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);			//
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);			//
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);			//
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);			//
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);			//

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);	//CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);	//CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);	
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);	
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);	//CCR�Զ�װ��Ĭ��Ҳ�Ǵ򿪵�
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); 				//TIM ARR�Զ�װ��Ĭ���Ǵ򿪵ģ����Բ�����
	TIM_ARRPreloadConfig(TIM3, ENABLE); 
	TIM_ARRPreloadConfig(TIM1, ENABLE); 

	TIM_Cmd(TIM4, ENABLE); 								//ʹ��TIM��ʱ��
	TIM_Cmd(TIM3, ENABLE); 								//ʹ��TIM��ʱ��
	TIM_Cmd(TIM1, ENABLE); 								//ʹ��TIM��ʱ��
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);					//TIM1��Ҫ��һ��
	
}

void controller_init(void)
{
	controller_sw_init();
	controller_hw_init();

}


//can update
void xcopter_can_output(unsigned int a ,unsigned int b, unsigned int c ,unsigned int d)
{
	CanTxMsg TxMessage;
	
	//send new output value to moto
	TxMessage.StdId = CAN_TYPE_MASTER | CAN_CTRL_GROUP0 | CAN_MESSAGE_INDEX0;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;
	
	//copy data to message
	TxMessage.Data[0] = a >> 8;
	TxMessage.Data[1] = a;
	TxMessage.Data[2] = b >> 8;
	TxMessage.Data[3] = b;
	TxMessage.Data[4] = c >> 8;
	TxMessage.Data[5] = c;
	TxMessage.Data[6] = d >> 8;
	TxMessage.Data[7] = d;
	
//	//copy data to message
//	memcpy(&(TxMessage.Data[0]), &a, sizeof(a));	
//	memcpy(&(TxMessage.Data[2]), &b, sizeof(b));	
//	memcpy(&(TxMessage.Data[4]), &c, sizeof(c));	
//	memcpy(&(TxMessage.Data[6]), &d, sizeof(d));		
	
	can_message_send(&TxMessage);

}

//pwm update
void pwm_output_update(int channel ,int pulse)
{
	//CCR�Ĵ���ֵ��Χֻ���� 0~0xffff
	if(pulse > 0xffff)pulse = 0xffff;
	
	//ֱ��д��Ĵ�������PWM����
	switch(channel)
	{
		case 1:	TIM4->CCR1 = pulse;break;		//channel define at thread_controller.h
		case 2:	TIM4->CCR2 = pulse;break;
		case 3:	TIM4->CCR3 = pulse;break;
		case 4:	TIM4->CCR4 = pulse;break;
		
		case 5:	TIM3->CCR1 = pulse;break;
		case 6:	TIM3->CCR2 = pulse;break;
		case 7:	TIM3->CCR3 = pulse;break;
		case 8:	TIM3->CCR4 = pulse;break;
		
		case 9:	TIM1->CCR1 = pulse;break;
		case 10:TIM1->CCR2 = pulse;break;
		case 11:TIM1->CCR3 = pulse;break;
		case 12:TIM1->CCR4 = pulse;break;
			
		default:break;	
	}	

}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(pwm_output_update, param pules channel);
#endif



////�����
////�����㷨�����������������������ֱ�����
//void fly_wing_control_mixer(int control_state)
//{
//	int left_sever;
//	int right_sever;
//	int throttle;
//	static int roll_sign = 1;
//	static int pitch_sign= 1;
//	static int yaw_sign = 1;
//	
//	
//	//���Ʒ�ʽ���Զ����в���״̬����Ϊ pitch���� yaw����) 
//	//����ж�Ӧ�����ȣ��򸲸�passbyģʽ�ļ�������
//	if(control_state & CONTROL_STATE_AP_ALL)
//	{
//		//yaw roll ����yaw����
//		if(control_state & CONTROL_STATE_AP_COURSE)
//		{
//			//��ת ���ţ����һ�㶼�Ƿ�װ �������ppm������ֵ
//// 			left_sever  = roll_sign * pid.roll_sens * (pid.output.roll + ppm.offset.aileron);	
//// 			right_sever = roll_sign * pid.roll_sens * (pid.output.roll + ppm.offset.aileron);

// 			//������������PWMֵ
//			left_sever  = roll_sign * pid.roll_sens * (pid.output.roll);	
//			right_sever = roll_sign * pid.roll_sens * (pid.output.roll);	
//		
//		}
//		else
//		{
//			left_sever  = roll_sign * pid.roll_sens * ppm.input.aileron;	//��ת ���ţ����һ�㶼�Ƿ�װ
//			right_sever = roll_sign * pid.roll_sens * ppm.input.aileron;
//		}
//		
//		//PITCH
//		if(control_state & CONTROL_STATE_AP_PITCH)
//		{
//			//���PID�����ĸ��� ����
//			left_sever += pitch_sign * pid.pitch_sens * (pid.output.pitch+ppm.offset.elevator);
//			right_sever-= pitch_sign * pid.pitch_sens * (pid.output.pitch+ppm.offset.elevator);	
//			
//		}	
//		else
//		{
//			left_sever += pitch_sign * pid.pitch_sens * ppm.input.elevator;//���PID�����ĸ��� ����
//			right_sever-= pitch_sign * pid.pitch_sens * ppm.input.elevator;	
//		}
//	}
//	//���Ʒ�ʽ����·ģʽ �����ж�˥����ֱ�����
//	else //if(control_state & CONTROL_STATE_PASSBY)
//	{	
//		left_sever  = roll_sign * pid.roll_sens * ppm.input.aileron;	//��ת ���ţ����һ�㶼�Ƿ�װ
//		right_sever = roll_sign * pid.roll_sens * ppm.input.aileron;
//		
//		left_sever += pitch_sign * pid.pitch_sens * ppm.input.elevator;//���� ����
//		right_sever-= pitch_sign * pid.pitch_sens * ppm.input.elevator;
// 	}	

//	//�޷�
//	#define SEVER_LIMIT_RANG 500
//	if(left_sever  > SEVER_LIMIT_RANG)left_sever = SEVER_LIMIT_RANG;
//	if(left_sever  <-SEVER_LIMIT_RANG)left_sever = -SEVER_LIMIT_RANG;
//	if(right_sever > SEVER_LIMIT_RANG)right_sever = SEVER_LIMIT_RANG;
//	if(right_sever <-SEVER_LIMIT_RANG)right_sever = -SEVER_LIMIT_RANG;

//	left_sever += ppm.ppm_zero;		//���ϻ�����Ȼ�����
//	right_sever+= ppm.ppm_zero;
//	
//	//throttle = expectation.throtlle * 8; 	//��Լ800us�Ŀ��
//	throttle = ppm.input.throttle + ppm.ppm_zero;			//��ֱ�����
//	
//	//���
//	pwm_output_update(PWM_CH1 ,left_sever);
//	pwm_output_update(PWM_CH2 ,right_sever);
//	pwm_output_update(PWM_CH3 ,throttle);
//}

////���沼�̶ֹ���ɻ�
////�����㷨�����������������������ֱ�����
//void conventional_plane_control_mixer(int control_state)
//{
//	int left_aileron_servo;    //ch1
//	int right_aileron_servo;   //ch2
//	int left_flag_servo;	   //ch5	
//	int right_flag_servo;	   //ch6	
//	int elevator_servo;		   //ch4 ������
//	int rudder_servo;		   //ch5
//	int throttle;			   //ch3
//	
//	//gimbal  ch11 ch12
//	//gear    ch7
//	
//	static int roll_sign = 1;	//����
//	static int pitch_sign= 1;
//	static int yaw_sign = 1;	
//	
//	//���Ʒ�ʽ���Զ����в���״̬����Ϊ pitch���� yaw����) 
//	//����ж�Ӧ�����ȣ��򸲸�passbyģʽ�ļ�������
//	if(control_state & CONTROL_STATE_AP_ALL)
//	{
//		//yaw roll ����yaw����
//		if(control_state & CONTROL_STATE_AP_COURSE)
//		{
//			//��ת ���ţ����һ�㶼�Ƿ�װ �������ppm������ֵ
//// 			left_sever  = roll_sign * pid.roll_sens * (pid.output.roll + ppm.offset.aileron);	
//// 			right_sever = roll_sign * pid.roll_sens * (pid.output.roll + ppm.offset.aileron);

// 			//������������PWMֵ
//			left_aileron_servo  = roll_sign * pid.roll_sens * (pid.output.roll);	
//			right_aileron_servo = roll_sign * pid.roll_sens * (pid.output.roll);	
//		
//		}
//		else
//		{
//			left_aileron_servo  = roll_sign * pid.roll_sens * ppm.input.aileron;	//��ת ���ţ����һ�㶼�Ƿ�װ
//			right_aileron_servo = roll_sign * pid.roll_sens * ppm.input.aileron;
//		}
//		
//		//PITCH
//		if(control_state & CONTROL_STATE_AP_PITCH)
//		{
//			//���PID�����ĸ��� ����
//			elevator_servo = pitch_sign * pid.pitch_sens * (pid.output.pitch + ppm.offset.elevator);	
//		}	
//		else
//		{
//			elevator_servo = pitch_sign * pid.pitch_sens * ppm.input.elevator;//���PID�����ĸ��� ����
//		}
//	}
//	//���Ʒ�ʽ����·ģʽ �����ж�˥����ֱ�����
//	else //if(control_state & CONTROL_STATE_PASSBY)
//	{	
//		left_aileron_servo  = roll_sign * pid.roll_sens * ppm.input.aileron;	//��ת ���ţ����һ�㶼�Ƿ�װ
//		right_aileron_servo = roll_sign * pid.roll_sens * ppm.input.aileron;
//		
//		elevator_servo += pitch_sign * pid.pitch_sens * ppm.input.elevator;//���� ����
// 	}	


//}




// PID�����㷨
// ���������ļ�
void pid_computer(void)
{
	double diff_yaw;
	double diff_pitch;
	double diff_roll;
	
	//pitch *************************** 
	//the difference between expectation and the current angle
	diff_pitch = expectation.angle.pitch - (out_angle.pitch - plane.angle_offset.pitch);
	
	//the integral increment of the diffference
	ctrl.pitch.increment = diff_pitch * ctrl.pitch.ki;
	
	//limit for the max increment
	if(ctrl.pitch.increment > ctrl.pitch.increment_max)
		ctrl.pitch.increment = ctrl.pitch.increment_max;
	else if(ctrl.pitch.increment < -ctrl.pitch.increment_max)
		ctrl.pitch.increment = -ctrl.pitch.increment_max;
	
	//limit for the mini increment. (EXCEPT ZERO)
	if(ctrl.pitch.increment > -ctrl.pitch.increment_min && ctrl.pitch.increment < ctrl.pitch.increment_min)
	{
		if(ctrl.pitch.increment > 0)
			ctrl.pitch.increment = ctrl.pitch.increment_min;
		else if(ctrl.pitch.increment < 0)
			ctrl.pitch.increment = -ctrl.pitch.increment_min;
	}
	
	//add the increment to integral. convert the gain to 1 second
	ctrl.pitch.integral += ctrl.pitch.increment /CTRL_FREQUENCY;
	
	//limit the max value of integral 
	if(ctrl.pitch.integral > ctrl.pitch.integral_max)
		ctrl.pitch.integral = ctrl.pitch.integral_max;
	if(ctrl.pitch.integral < -ctrl.pitch.integral_max)
		ctrl.pitch.integral = -ctrl.pitch.integral_max;
	
	//use the value above to calculate output value. 
	ctrl.pitch.output.value = ctrl.pitch.kp * diff_pitch + ctrl.pitch.integral - ctrl.pitch.kd * gyro.pitch;
	
	
	
	
	//roll ************************
	//the difference between expectation and the current angle
	diff_roll = expectation.angle.roll - (out_angle.roll - plane.angle_offset.roll);
	
	//the integral increment of the diffference
	ctrl.roll.increment = diff_roll * ctrl.roll.ki;
	
	//limit for the max increment
	if(ctrl.roll.increment > ctrl.roll.increment_max)
		ctrl.roll.increment = ctrl.roll.increment_max;
	else if(ctrl.roll.increment < -ctrl.roll.increment_max)
		ctrl.roll.increment = -ctrl.roll.increment_max;
	
	//limit for the mini increment. 
	if(ctrl.roll.increment > -ctrl.roll.increment_min && ctrl.roll.increment < ctrl.roll.increment_min)
	{
		if(ctrl.roll.increment > 0)
			ctrl.roll.increment = ctrl.roll.increment_min;
		else if(ctrl.roll.increment < 0)
			ctrl.roll.increment = -ctrl.roll.increment_min;
	}
	
	//add the increment to integral. convert the gain to 1 second
	ctrl.roll.integral += ctrl.roll.increment /CTRL_FREQUENCY;
	
	//limit the max value of integral 
	if(ctrl.roll.integral > ctrl.roll.integral_max)
		ctrl.roll.integral = ctrl.roll.integral_max;
	if(ctrl.roll.integral < -ctrl.roll.integral_max)
		ctrl.roll.integral = -ctrl.roll.integral_max;
	
	//use the value above to calculate output value. 
	ctrl.roll.output.value = ctrl.roll.kp * diff_roll + ctrl.roll.integral - ctrl.roll.kd * gyro.roll;
	
	
	
	
	//yaw ****************
	diff_yaw = (expectation.angle.yaw - (out_angle.yaw - plane.angle_offset.yaw));
	if(diff_yaw  >  180.0)diff_yaw  -= 360.0;			//�Ƕȱ任
	if(diff_yaw  <= -180.0)diff_yaw += 360.0;	
	
	#define ANGLE_YAW_LIMIT 40.0//��������� +-40��
	if(diff_yaw > ANGLE_YAW_LIMIT)									//�Ƕ��޷�
		diff_yaw = ANGLE_YAW_LIMIT;
	if(diff_yaw < -ANGLE_YAW_LIMIT)									//�Ƕ��޷�
		diff_yaw = -ANGLE_YAW_LIMIT;
	
	//the integral increment of the diffference
	ctrl.yaw.increment = diff_yaw * ctrl.yaw.ki;
	
	//limit for the max increment
	if(ctrl.yaw.increment > ctrl.yaw.increment_max)
		ctrl.yaw.increment = ctrl.yaw.increment_max;
	else if(ctrl.yaw.increment < -ctrl.yaw.increment_max)
		ctrl.yaw.increment = -ctrl.yaw.increment_max;
	
	//limit for the mini increment. 
	if(ctrl.yaw.increment > -ctrl.yaw.increment_min && ctrl.yaw.increment < ctrl.yaw.increment_min)
	{
		if(ctrl.yaw.increment > 0)
			ctrl.yaw.increment = ctrl.yaw.increment_min;
		else if(ctrl.yaw.increment < 0)
			ctrl.yaw.increment = -ctrl.yaw.increment_min;
	}
	
	//add the increment to integral. convert the gain to 1 second
	ctrl.yaw.integral += ctrl.yaw.increment /CTRL_FREQUENCY;
	
	//limit the max value of integral 
	if(ctrl.yaw.integral > ctrl.yaw.integral_max)
		ctrl.yaw.integral = ctrl.yaw.integral_max;
	if(ctrl.yaw.integral < -ctrl.yaw.integral_max)
		ctrl.yaw.integral = -ctrl.yaw.integral_max;

	//use the value above to calculate output value. 
	ctrl.yaw.output.value = ctrl.yaw.kp * diff_yaw + ctrl.yaw.integral - ctrl.yaw.kd * (gyro.yaw);

}

//reset integratal for low throttle 
void pid_reset(void)
{
	ctrl.pitch.integral = 0;
	ctrl.roll.integral = 0;
	ctrl.yaw.integral = 0;
}

//����X�������������
//�����㷨�����������������������ֱ�����
void xcopter_control_mixer(int control_state)
{
	int left_front_moto = 0;    //ch1
	int right_front_moto = 0;   //ch2	
	int right_back_moto = 0;	//ch3
	int left_back_moto = 0;		//ch4
	int throttle = expectation.throttle + ppm.ppm_zero;
	
	static int roll_sign = 1;	//����
	static int pitch_sign= 1;
	static int yaw_sign  = 1;	
	
	if(throttle > 1800)
	{
		throttle = 1800;
	}

	left_front_moto -= yaw_sign * (ctrl.yaw.output.value);
	right_back_moto -= yaw_sign * (ctrl.yaw.output.value);
	
	right_front_moto += yaw_sign * (ctrl.yaw.output.value);
	left_back_moto   += yaw_sign * (ctrl.yaw.output.value);		


	left_front_moto  += roll_sign * (ctrl.roll.output.value - ppm.offset.ch1); //
	left_back_moto   += roll_sign * (ctrl.roll.output.value - ppm.offset.ch1);
	right_front_moto -= roll_sign * (ctrl.roll.output.value - ppm.offset.ch1);
	right_back_moto  -= roll_sign * (ctrl.roll.output.value - ppm.offset.ch1);


	left_front_moto  += pitch_sign * (ctrl.pitch.output.value - ppm.offset.ch2);	
	right_front_moto += pitch_sign * (ctrl.pitch.output.value - ppm.offset.ch2);	
	left_back_moto   -= pitch_sign * (ctrl.pitch.output.value - ppm.offset.ch2);	
	right_back_moto  -= pitch_sign * (ctrl.pitch.output.value - ppm.offset.ch2);		
	
	
	//�޷�
	#define MOTO_LIMIT_RANG 500

	if(left_front_moto  > MOTO_LIMIT_RANG) left_front_moto  = MOTO_LIMIT_RANG;
	if(left_front_moto  <-MOTO_LIMIT_RANG) left_front_moto  = -MOTO_LIMIT_RANG;
	if(right_front_moto > MOTO_LIMIT_RANG) right_front_moto = MOTO_LIMIT_RANG;
	if(right_front_moto <-MOTO_LIMIT_RANG) right_front_moto = -MOTO_LIMIT_RANG;
	if(left_back_moto  > MOTO_LIMIT_RANG)	left_back_moto 	= MOTO_LIMIT_RANG;
	if(left_back_moto  <-MOTO_LIMIT_RANG)	left_back_moto 	= -MOTO_LIMIT_RANG;
	if(right_back_moto > MOTO_LIMIT_RANG)	right_back_moto = MOTO_LIMIT_RANG;
	if(right_back_moto <-MOTO_LIMIT_RANG)	right_back_moto = -MOTO_LIMIT_RANG;	

	//add the throttle
	left_front_moto += throttle;		
	left_back_moto  += throttle;
	right_front_moto += throttle;
	right_back_moto += throttle;
	
	//output limit
	#define MOTO_LIMIT_MIN 1000
	#define MOTO_LIMIT_MAX 1900
	
	if(left_front_moto  > MOTO_LIMIT_MAX) 	left_front_moto  = MOTO_LIMIT_MAX;
	if(left_front_moto  < MOTO_LIMIT_MIN) 	left_front_moto  = MOTO_LIMIT_MIN;	
	if(right_front_moto > MOTO_LIMIT_MAX) 	right_front_moto = MOTO_LIMIT_MAX;
	if(right_front_moto < MOTO_LIMIT_MIN) 	right_front_moto = MOTO_LIMIT_MIN;
	if(left_back_moto  > MOTO_LIMIT_MAX) 	left_back_moto 	= MOTO_LIMIT_MAX;
	if(left_back_moto  < MOTO_LIMIT_MIN)	left_back_moto 	= MOTO_LIMIT_MIN;
	if(right_back_moto > MOTO_LIMIT_MAX)	right_back_moto = MOTO_LIMIT_MAX;
	if(right_back_moto < MOTO_LIMIT_MIN)	right_back_moto = MOTO_LIMIT_MIN;	
	
	
	//if throttle is zero, then stop output value
	if(throttle < 1050)
	{
		left_front_moto = MOTO_LIMIT_MIN;
		right_front_moto = MOTO_LIMIT_MIN;
		left_back_moto = MOTO_LIMIT_MIN;
		right_back_moto = MOTO_LIMIT_MIN;
	}
	//not used pid result in small throttle, output throttle directly
	else if(throttle < 1150)
	{
		left_front_moto = throttle;
		right_front_moto = throttle;
		left_back_moto = throttle;
		right_back_moto = throttle;
	
		//low throttle then reset pid
		pid_reset();
	}
	
	//���
	pwm_output_update(PWM_CH1 ,left_front_moto);
	pwm_output_update(PWM_CH2 ,right_front_moto);
	pwm_output_update(PWM_CH3 ,left_back_moto);
	pwm_output_update(PWM_CH4 ,right_back_moto);
	
	//can output
	xcopter_can_output(left_front_moto ,right_front_moto, right_back_moto ,left_back_moto);
	
	//mark output
	ctrl.opwm.ch1 = left_front_moto;
	ctrl.opwm.ch2 = right_front_moto;
	ctrl.opwm.ch3 = left_back_moto;
	ctrl.opwm.ch4 = right_back_moto;
}



//������������
void thread_controller(void)
{
	timer_init();
	
	controller_init();
	
	while(1)
	{
		//waiting for new period 
		if(rt_sem_take(&sem, 20) == -RT_ETIMEOUT);
				
		//pid 
		pid_computer();
		
		//mixer and output
		xcopter_control_mixer(control.state);		
	}


}
