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

int SpeedLevelFlag_Forward = 1;
int SpeedLevelFlag_Turn = 1;

int CAR_SPEED_MAX = 4000;
int	CAR_SPEED_MIN = -4000;

/*�ǶȻ����ٶȻ�PID���Ʋ�������*/
float  Angle_P =100;
float  Angle_D =0.31;	

float  Speed_P=6;
float  Speed_I=0.2;

/*�ǶȻ����Ʋ�������*/
float CarAngle;
float AngleControlOut;

/*�ٶȻ����Ʋ�������*/
float CarSpeed;							 
float CarSpeed_Old;

s16   LeftMotorPulse;					   //�������������з��ţ�
s16	  RightMotorPulse;					   //�ҵ�����������з��ţ�

s32   LeftMotorPulse_Add;				   //50ms��������ֵ
s32   RightMotorPulse_Add;				   //50ms�ҵ������ֵ

float CarDistance;						   //����õ���С��λ��
float SpeedControlOut;

int SpeedControlCount;

/*���������Ʋ�������*/

float LeftMotorOut;
float RightMotorOut;

/*PS2�ֱ����Ʋ�������*/
float PS2_Speed;
float PS2_Direction;			    

float TempPS2_Direction=400; 
/**
 *  @brief      calculate the angle link with PD arithmetic.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void AngleControl(void)	 
{
	CarAngle = Roll - CAR_ZERO_ANGLE;		   
	AngleControlOut =  CarAngle * Angle_P + gyro[0] * Angle_D ;	  //�Ƕ�PD����							   
}

/**
 *  @brief      setting the value which is used for the two motors and the direction.
 *  @param[out] none
 *  @param[in]  leftvoltage and rightvoltage,which is calculated by the other function.
 *  @return     none.
 */
void MotorOutputAndDirection(s16 LeftValue,s16 RightValue)
{
	  u16 LeftMotorOutput;
	  u16 RightMotorOutput;
	
    if(LeftValue<0)			            //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ��
    {	
	  GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      LeftValue = (-LeftValue);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 ); //������PWM���Ϊ��ʱ PB14��Ϊ�� PB15��Ϊ�� 
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      LeftValue = LeftValue;
    }

    if(RightValue<0)
    {									 //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� 
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      RightValue = (-RightValue);
    }
    else								 //���ҵ��PWM���Ϊ��ʱ PB12��Ϊ�� PB13��Ϊ�� 
    {
	  GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
      RightValue = RightValue;
    }
		
	RightMotorOutput= (u16)RightValue;
	LeftMotorOutput = (u16)LeftValue;

	TIM_SetCompare3(TIM2,LeftMotorOutput);	 //��������PWMռ�ձ�
	TIM_SetCompare4(TIM2,RightMotorOutput);	 //��������PWMռ�ձ�
}

/**
 *  @brief      enable the output of the two motor through Tim2 with pwm value.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void MotorOutput(void)																					 
{	   
	LeftMotorOut  = AngleControlOut +SpeedControlOut + PS2_Direction;//+directionl - BST_fBluetoothDirectionNew;			//����ת��PWM�����ں�ƽ��Ƕȡ��ٶ����	
    RightMotorOut = AngleControlOut +SpeedControlOut - PS2_Direction;//-directionl+ BST_fBluetoothDirectionNew;			//�ҵ��ת��PWM�����ں�ƽ��Ƕȡ��ٶ����

	if((s16)LeftMotorOut  > MOTOR_OUTPUT_MAX)	LeftMotorOut  = MOTOR_OUTPUT_MAX;
	if((s16)LeftMotorOut  < MOTOR_OUTPUT_MIN)	LeftMotorOut  = MOTOR_OUTPUT_MIN;
	if((s16)RightMotorOut > MOTOR_OUTPUT_MAX)	RightMotorOut = MOTOR_OUTPUT_MAX;
	if((s16)RightMotorOut < MOTOR_OUTPUT_MIN)	RightMotorOut = MOTOR_OUTPUT_MIN;
	
    MotorOutputAndDirection((s16)LeftMotorOut,(s16)RightMotorOut);
}

/**
 *  @brief      getting the value of two counters Tim3 and Tim4 
    which refered to the speed of two motors.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void GetMotorPulse(void)              
{ 
	uint16_t TempLeft;
	uint16_t TempRight;
	
	TempLeft = TIM_GetCounter(TIM3);   //  TIM3��ʱ���������
 	TempRight= TIM_GetCounter(TIM4);	//	 TIM4��ʱ���������
	
	TIM_SetCounter(TIM3,0);//TIM3->CNT = 0;
	TIM_SetCounter(TIM4,0);//TIM4->CNT = 0;   //����
	LeftMotorPulse=TempLeft;
	RightMotorPulse=TempRight;
	
	LeftMotorPulse_Add  +=LeftMotorPulse;		 //����ֵ���� 40ms����ֵ
	RightMotorPulse_Add +=RightMotorPulse; 	 	 //����ֵ���� 40ms����ֵ
}

/**
 *  @brief      calculate the output of speed link with PI arithmetic.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void SpeedControl(void)
{
	CarSpeed = (-LeftMotorPulse_Add  - RightMotorPulse_Add ); //���ҵ������������ֵ��ΪС����ǰ����
	LeftMotorPulse_Add =RightMotorPulse_Add = 0;	          		
	CarSpeed_Old *= 0.7;						              //����һ���ռ����ٶȽ���һ�׵�ͨ�˲�����
	CarSpeed_Old +=CarSpeed*0.3;
	
	CarDistance += CarSpeed_Old; 		 //�ٶȵĻ��֣���λ��
	CarDistance += PS2_Speed;    		 //����PS�ֱ����Ƶ��ٶ�
	
	//�����ٶ�����
	if((s32)CarDistance > CAR_SPEED_MAX)    CarDistance = CAR_SPEED_MAX;
	if((s32)CarDistance < CAR_SPEED_MIN)    CarDistance = CAR_SPEED_MIN;
																								  
	SpeedControlOut = (CarSpeed_Old -CAR_SPEED_SET ) * Speed_P + (CarDistance - CAR_POSITION_SET ) * Speed_I; //�ٶ�PI�㷨 �ٶ�*P +λ��*I=�ٶ�PWM���
}

/**
 *  @brief      used for the PS2 handle to control the state of the car.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void CarStateOut(void)
{
	switch (CarState)
	{
		case STOP: //ֹͣ
		{
			PS2_Speed = 0;
			PS2_Direction=0;
		} break; 					   

		case RUN: //��ǰ�ٶ�   
		{	
			PS2_Speed = 1000 ;
		}break;	   

		case LEFT://��ת 
		{
			PS2_Direction= -TempPS2_Direction; 
		}break;  
		
		case RIGHT: //��ת
		{
			PS2_Direction= TempPS2_Direction; 

		}break;	
		
		case BACK: //�����ٶ� 
		{
			PS2_Speed = (-1000);
		}break;		
		
		default: PS2_Speed = 0; 
		break; 		
	}
}


