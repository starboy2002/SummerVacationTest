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

int SpeedLevelFlag_Forward = 1;
int SpeedLevelFlag_Turn = 1;

int CAR_SPEED_MAX = 4000;
int	CAR_SPEED_MIN = -4000;

/*角度环和速度环PID控制参数声明*/
float  Angle_P =100;
float  Angle_D =0.31;	

float  Speed_P=6;
float  Speed_I=0.2;

/*角度环控制参数声明*/
float CarAngle;
float AngleControlOut;

/*速度环控制参数声明*/
float CarSpeed;							 
float CarSpeed_Old;

s16   LeftMotorPulse;					   //左电机脉冲数（有符号）
s16	  RightMotorPulse;					   //右电机脉冲数（有符号）

s32   LeftMotorPulse_Add;				   //50ms左电机叠加值
s32   RightMotorPulse_Add;				   //50ms右电机叠加值

float CarDistance;						   //计算得到的小车位移
float SpeedControlOut;

int SpeedControlCount;

/*电机输出控制参数声明*/

float LeftMotorOut;
float RightMotorOut;

/*PS2手柄控制参数声明*/
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
	AngleControlOut =  CarAngle * Angle_P + gyro[0] * Angle_D ;	  //角度PD控制							   
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
	
    if(LeftValue<0)			            //当左电机PWM输出为负时 PB14设为正 PB15设为负
    {	
	  GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      LeftValue = (-LeftValue);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 ); //当左电机PWM输出为正时 PB14设为负 PB15设为正 
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      LeftValue = LeftValue;
    }

    if(RightValue<0)
    {									 //当右电机PWM输出为负时 PB12设为正 PB13设为负 
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      RightValue = (-RightValue);
    }
    else								 //当右电机PWM输出为正时 PB12设为负 PB13设为正 
    {
	  GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
      RightValue = RightValue;
    }
		
	RightMotorOutput= (u16)RightValue;
	LeftMotorOutput = (u16)LeftValue;

	TIM_SetCompare3(TIM2,LeftMotorOutput);	 //调节左轮PWM占空比
	TIM_SetCompare4(TIM2,RightMotorOutput);	 //调节右轮PWM占空比
}

/**
 *  @brief      enable the output of the two motor through Tim2 with pwm value.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void MotorOutput(void)																					 
{	   
	LeftMotorOut  = AngleControlOut +SpeedControlOut + PS2_Direction;//+directionl - BST_fBluetoothDirectionNew;			//左电机转向PWM控制融合平衡角度、速度输出	
    RightMotorOut = AngleControlOut +SpeedControlOut - PS2_Direction;//-directionl+ BST_fBluetoothDirectionNew;			//右电机转向PWM控制融合平衡角度、速度输出

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
	
	TempLeft = TIM_GetCounter(TIM3);   //  TIM3定时器计算调用
 	TempRight= TIM_GetCounter(TIM4);	//	 TIM4定时器计算调用
	
	TIM_SetCounter(TIM3,0);//TIM3->CNT = 0;
	TIM_SetCounter(TIM4,0);//TIM4->CNT = 0;   //清零
	LeftMotorPulse=TempLeft;
	RightMotorPulse=TempRight;
	
	LeftMotorPulse_Add  +=LeftMotorPulse;		 //脉冲值叠加 40ms叠加值
	RightMotorPulse_Add +=RightMotorPulse; 	 	 //脉冲值叠加 40ms叠加值
}

/**
 *  @brief      calculate the output of speed link with PI arithmetic.
 *  @param[out] none
 *  @param[in]  none.
 *  @return     none.
 */
void SpeedControl(void)
{
	CarSpeed = (-LeftMotorPulse_Add  - RightMotorPulse_Add ); //左右电机脉冲数叠加值作为小车当前车速
	LeftMotorPulse_Add =RightMotorPulse_Add = 0;	          		
	CarSpeed_Old *= 0.7;						              //对上一次收集的速度进行一阶低通滤波处理
	CarSpeed_Old +=CarSpeed*0.3;
	
	CarDistance += CarSpeed_Old; 		 //速度的积分，即位移
	CarDistance += PS2_Speed;    		 //加入PS手柄控制的速度
	
	//设置速度上限
	if((s32)CarDistance > CAR_SPEED_MAX)    CarDistance = CAR_SPEED_MAX;
	if((s32)CarDistance < CAR_SPEED_MIN)    CarDistance = CAR_SPEED_MIN;
																								  
	SpeedControlOut = (CarSpeed_Old -CAR_SPEED_SET ) * Speed_P + (CarDistance - CAR_POSITION_SET ) * Speed_I; //速度PI算法 速度*P +位移*I=速度PWM输出
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
		case STOP: //停止
		{
			PS2_Speed = 0;
			PS2_Direction=0;
		} break; 					   

		case RUN: //向前速度   
		{	
			PS2_Speed = 1000 ;
		}break;	   

		case LEFT://左转 
		{
			PS2_Direction= -TempPS2_Direction; 
		}break;  
		
		case RIGHT: //右转
		{
			PS2_Direction= TempPS2_Direction; 

		}break;	
		
		case BACK: //后退速度 
		{
			PS2_Speed = (-1000);
		}break;		
		
		default: PS2_Speed = 0; 
		break; 		
	}
}


