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

int CarState = STOP; //  1前2后3左4右0停止

/*角度环控制参数声明*/
float BST_fCarAngle;
float AngleControlOut;

/*-----角度环和速度环PID控制参数-----*///以下参考为重点调试参考，同电池电压有关，建议充好电再调试
float  BST_fCarAngle_P =100;//	191.3 //调大小时会左右摆，调大时会振动  请调到基本能够站立 P=91.3是用于给小车在运动过程使用
float  BST_fCarAngle_D =0.31;	// 0.001 0.002 0.004 0.008 0.0010 0.011	 调小时反应慢，调大时会干扰

float  BST_fCarSpeed_P=6;
float  BST_fCarSpeed_I=0.2;

/******电机控制参数******/
float BST_fSpeedControlOut;						   //速度控制PWM
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

/******速度控制参数******/
s16   BST_s16LeftMotorPulse;					  //左电机脉冲数
s16	  BST_s16RightMotorPulse;					   //右电机脉冲数

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  //50ms左电机叠加值
s32   BST_s32RightMotorPulseSigma;				 //50ms右电机叠加值

float BST_fCarSpeed;							 //测速码盘得出的车速
float BST_fCarSpeedOld;

float BST_fCarPosition;						   //测速码盘通过计算得到的小车位移


float BST_fBluetoothSpeed;						//蓝牙控制车速


int BST_u8SpeedControlCount;


float PS2_Speed;
float PS2_Direction;			    //用于平缓输出车速使用


void AngleControl(void)	 
{
	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   //DMP ROLL滚动方向角度与预设小车倾斜角度值的差得出角度   
	BST_fAngleControlOut =  BST_fCarAngle * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D ;	  //角度PD控制							   
}

void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	  u16 u16LeftMotorValue;
	  u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 //当左电机PWM输出为负时 PB14设为正 PB15设为负 （PB14 15 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
    {	
	   GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    	 //当左电机PWM输出为正时 PB14设为负 PB15设为正 （PB14 15 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 //当右电机PWM输出为负时 PB12设为正 PB13设为负 （PB12 13 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														//当右电机PWM输出为正时 PB12设为负 PB13设为正 （PB12 13 分别控制TB6612fng驱动芯片，逻辑0 1可控制左电机正转反转）
    {
	    GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
     
      s16RightVoltage = s16RightVoltage;
    }
		
	   u16RightMotorValue= (u16)s16RightVoltage;
	   u16LeftMotorValue = (u16)s16LeftVoltage;


	TIM_SetCompare3(TIM2,u16LeftMotorValue);			  //TIM2与 u16RightMotorValue对比，不相同则翻转波形，调节PWM占空比
	TIM_SetCompare4(TIM2,u16RightMotorValue);			  //TIM3与 u16LeftMotorValue对比，不相同则翻转波形，调节PWM占空比
}

void MotorOutput(void)																					  //电机PWM输出函数
{	   
			

	BST_fLeftMotorOut  = BST_fAngleControlOut +BST_fSpeedControlOutNew + PS2_Direction;//+directionl - BST_fBluetoothDirectionNew;			//左电机转向PWM控制融合平衡角度、速度输出	
    BST_fRightMotorOut = BST_fAngleControlOut +BST_fSpeedControlOutNew - PS2_Direction;//-directionl+ BST_fBluetoothDirectionNew;			//右电机转向PWM控制融合平衡角度、速度输出

		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
    
}

void GetMotorPulse(void)              //采集电机速度脉冲
{ 
	uint16_t u16TempLeft;
	uint16_t u16TempRight;
	
	u16TempLeft = TIM_GetCounter(TIM3);   //  TIM3定时器计算调用
 	u16TempRight= TIM_GetCounter(TIM4);	//	 TIM4定时器计算调用
	
	TIM_SetCounter(TIM3,0);//TIM3->CNT = 0;
	TIM_SetCounter(TIM4,0);//TIM4->CNT = 0;   //清零
	BST_s16LeftMotorPulse=u16TempLeft;
	BST_s16RightMotorPulse=(u16TempRight);
		
	BST_s32LeftMotorPulseSigma  +=BST_s16LeftMotorPulse;		 //脉冲值叠加 40ms叠加值
	BST_s32RightMotorPulseSigma +=BST_s16RightMotorPulse; 	 //脉冲值叠加 40ms叠加值
}

void SpeedControl(void)
{
  
 
	BST_fCarSpeed = (-BST_s32LeftMotorPulseSigma  - BST_s32RightMotorPulseSigma );// * 0.5 ;		  //左右电机脉冲数平均值作为小车当前车速
	BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零		
	BST_fCarSpeedOld *= 0.7;
	BST_fCarSpeedOld +=BST_fCarSpeed*0.3;
	
	BST_fCarPosition += BST_fCarSpeedOld; 		 //路程  即速度积分	   1/11 3:03
	BST_fCarPosition += PS2_Speed;   //融合蓝牙给定速度   //融合蓝牙给定速度
	


	//积分上限设限//
	if((s32)BST_fCarPosition > CAR_POSITION_MAX)    BST_fCarPosition = CAR_POSITION_MAX;
	if((s32)BST_fCarPosition < CAR_POSITION_MIN)    BST_fCarPosition = CAR_POSITION_MIN;
	
																								  
	BST_fSpeedControlOutNew = (BST_fCarSpeedOld -CAR_SPEED_SET ) * BST_fCarSpeed_P + (BST_fCarPosition - CAR_POSITION_SET ) * BST_fCarSpeed_I; //速度PI算法 速度*P +位移*I=速度PWM输出
}

/*PS2手柄状态反馈运动*/
void CarStateOut(void)
{
	switch (CarState)
	{
		case STOP: //停止
		{
			PS2_Speed = 0;
			PS2_Direction=0;
		} break; 					   

		case RUN: //向前速度 250  
		{	
			PS2_Speed =   3000 ;
		}break;	   

		case LEFT://左转 
		{
			PS2_Direction= -400; 
		}break;  
		
		case RIGHT: //右转
		{
			PS2_Direction= 400; 

		}break;	
		
		case BACK: //后退速度 -250
		{
			PS2_Speed = (-3000);
		}break;		
		
		default: PS2_Speed = 0; break; 					   //停止
	}
}


