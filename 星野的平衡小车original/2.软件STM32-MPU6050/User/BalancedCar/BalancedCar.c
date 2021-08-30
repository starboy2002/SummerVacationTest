#include "BalancedCar.h"
#include "Motor.h"
#include "usart.h"
#include "mpu6050.h"
#include "stm32f10x_gpio.h"
#include "math.h" 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f10x.h"

int CarState = STOP; //  1ǰ2��3��4��0ֹͣ

/*�ǶȻ����Ʋ�������*/
float BST_fCarAngle;
float AngleControlOut;

/*-----�ǶȻ����ٶȻ�PID���Ʋ���-----*///���²ο�Ϊ�ص���Բο���ͬ��ص�ѹ�йأ������õ��ٵ���
float  BST_fCarAngle_P =100;//	191.3 //����Сʱ�����Ұڣ�����ʱ����  ����������ܹ�վ�� P=91.3�����ڸ�С�����˶�����ʹ��
float  BST_fCarAngle_D =0.31;	// 0.001 0.002 0.004 0.008 0.0010 0.011	 ��Сʱ��Ӧ��������ʱ�����

float  BST_fCarSpeed_P=6;
float  BST_fCarSpeed_I=0.2;

/******������Ʋ���******/
float BST_fSpeedControlOut;						   //�ٶȿ���PWM
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

/******�ٶȿ��Ʋ���******/
s16   BST_s16LeftMotorPulse;					  //����������
s16	  BST_s16RightMotorPulse;					   //�ҵ��������

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  //50ms��������ֵ
s32   BST_s32RightMotorPulseSigma;				 //50ms�ҵ������ֵ

float BST_fCarSpeed;							 //�������̵ó��ĳ���
float BST_fCarSpeedOld;

float BST_fCarPosition;						   //��������ͨ������õ���С��λ��


float BST_fBluetoothSpeed;						//�������Ƴ���


int BST_u8SpeedControlCount;


float PS2_Speed;
float PS2_Direction;			    //����ƽ���������ʹ��


void AngleControl(void)	 
{
	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL��������Ƕ���Ԥ��С����б�Ƕ�ֵ�Ĳ�ó��Ƕ�   
	BST_fAngleControlOut =  BST_fCarAngle * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D ;	  //�Ƕ�PD����							   
}

void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	  u16 u16LeftMotorValue;
	  u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� ��PB14 15 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
    {	
	   GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    	 //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� ��PB14 15 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� ��PB12 13 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														//���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� ��PB12 13 �ֱ����TB6612fng����оƬ���߼�0 1�ɿ���������ת��ת��
    {
	    GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
     
      s16RightVoltage = s16RightVoltage;
    }
		
	   u16RightMotorValue= (u16)s16RightVoltage;
	   u16LeftMotorValue = (u16)s16LeftVoltage;


	TIM_SetCompare3(TIM2,u16LeftMotorValue);			  //TIM2�� u16RightMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
	TIM_SetCompare4(TIM2,u16RightMotorValue);			  //TIM3�� u16LeftMotorValue�Աȣ�����ͬ��ת���Σ�����PWMռ�ձ�
}

void MotorOutput(void)																					  //���PWM�������
{	   
			

	BST_fLeftMotorOut  = BST_fAngleControlOut +BST_fSpeedControlOutNew + PS2_Direction;//+directionl - BST_fBluetoothDirectionNew;			//����ת��PWM�����ں�ƽ��Ƕȡ��ٶ����	
    BST_fRightMotorOut = BST_fAngleControlOut +BST_fSpeedControlOutNew - PS2_Direction;//-directionl+ BST_fBluetoothDirectionNew;			//�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����

		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
    
}

void GetMotorPulse(void)              //�ɼ�����ٶ�����
{ 
	uint16_t u16TempLeft;
	uint16_t u16TempRight;
	
	u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3��ʱ���������
 	u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4��ʱ���������
	
	TIM_SetCounter(TIM3,0);//TIM3->CNT = 0;
	TIM_SetCounter(TIM4,0);//TIM4->CNT = 0;   //����
	BST_s16LeftMotorPulse=u16TempLeft;
	BST_s16RightMotorPulse=(u16TempRight);
		
	BST_s32LeftMotorPulseSigma  +=BST_s16LeftMotorPulse;		 //����ֵ���� 40ms����ֵ
	BST_s32RightMotorPulseSigma +=BST_s16RightMotorPulse; 	 //����ֵ���� 40ms����ֵ
}

void SpeedControl(void)
{
  
 
	BST_fCarSpeed = (-BST_s32LeftMotorPulseSigma  - BST_s32RightMotorPulseSigma );// * 0.5 ;		  //���ҵ��������ƽ��ֵ��ΪС����ǰ����
	BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  //ȫ�ֱ��� ע�⼰ʱ����		
	BST_fCarSpeedOld *= 0.7;
	BST_fCarSpeedOld +=BST_fCarSpeed*0.3;
	
	BST_fCarPosition += BST_fCarSpeedOld; 		 //·��  ���ٶȻ���	   1/11 3:03
	BST_fCarPosition += PS2_Speed;   //�ں����������ٶ�   //�ں����������ٶ�
	


	//������������//
	if((s32)BST_fCarPosition > CAR_POSITION_MAX)    BST_fCarPosition = CAR_POSITION_MAX;
	if((s32)BST_fCarPosition < CAR_POSITION_MIN)    BST_fCarPosition = CAR_POSITION_MIN;
	
																								  
	BST_fSpeedControlOutNew = (BST_fCarSpeedOld -CAR_SPEED_SET ) * BST_fCarSpeed_P + (BST_fCarPosition - CAR_POSITION_SET ) * BST_fCarSpeed_I; //�ٶ�PI�㷨 �ٶ�*P +λ��*I=�ٶ�PWM���
}

/*PS2�ֱ�״̬�����˶�*/
void CarStateOut(void)
{
	switch (CarState)
	{
		case STOP: //ֹͣ
		{
			PS2_Speed = 0;
			PS2_Direction=0;
		} break; 					   

		case RUN: //��ǰ�ٶ� 250  
		{	
			PS2_Speed =   3000 ;
		}break;	   

		case LEFT://��ת 
		{
			PS2_Direction= -400; 
		}break;  
		
		case RIGHT: //��ת
		{
			PS2_Direction= 400; 

		}break;	
		
		case BACK: //�����ٶ� -250
		{
			PS2_Speed = (-3000);
		}break;		
		
		default: PS2_Speed = 0; break; 					   //ֹͣ
	}
}


