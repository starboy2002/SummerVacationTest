/**
  ******************************************************************************
  * @file    main.c
  * @author  LYC
  * @version V1.0
  * @date    2014-04-22
  * @brief   MPU6050 软件IIC测试
  ******************************************************************************
  * @attention
  * 实验平台:野火 霸道 STM32 开发板 
  ******************************************************************************
  */
  
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./systick/bsp_SysTick.h"
#include "./led/bsp_led.h"
#include "delay.h"
#include "I2C.h"
#include "mpu6050.h"
#include "usart.h"
#include "encode.h"
#include "BalancedCar.h"
#include "Motor.h"
#include "ps2.h"

#define TASK_ENABLE 0
extern unsigned int Task_Delay[NumOfTask];
/*
 * t : 定时时间 
 * Ticks : 多少个时钟周期产生一次中断 
 * f : 时钟频率 72000000
 * t = Ticks * 1/f = (72000000/100000) * (1/72000000) = 10us 
 */ 

/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{
	int PS2_KEY = 0, X1=0,Y1=127,X2=0,Y2=0,Red=1;
	
	int num = 0;
	
    //初始化systick	
	SystemInit();
	
	//PS2手柄初始化
	PS2_Init();							  
	
	//手柄设置
	PS2_SetInit();
	
	//定时器2PWM输出初始化
	TIM2_PWM_Init();					  
	
	//电机IO口初始化
	MOTOR_GPIO_Config();				  
	
	/* 串口1通信初始化 */
	USART1_Config();
	
	//电机TIM3编码器初始化
	encoder_Tim3_init();
	
	//电机TIM4编码器初始化
	encoder_Tim4_init();
	
	//I2C初始化
	i2cInit();
	
	delay_ms(10);	
    
	//MPU6050初始化
	MPU6050_Init();
	
	//控制参数初始化
	//Car_ParametersInit();   
	delay_ms(500); 
	
	//系统定时器初始化
	SysTick_Init();     
	
	//使能系统定时器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
		while(1)
		{
				MPU6050_Pose();	
				num++;
				if(num == 200)
				{
					num =0;
					PS2_KEY = PS2_DataKey();	 //手柄按键捕获处理
					switch(PS2_KEY)
					{
						case PSB_SELECT: 	PrintChar("PSB_SELECT \n");  break;
						case PSB_L3:     	CarState = STOP;  PrintChar("PSB_L3 \n");  break;  
						case PSB_R3:     	CarState = STOP;	 PrintChar("PSB_R3 \n");  break;  
						case PSB_START:  	PrintChar("PSB_START \n");  break;  
						case PSB_PAD_UP: 	CarState = RUN;   PrintChar("PSB_PAD_UP \n");  break;  
						case PSB_PAD_RIGHT:	CarState = RIGHT; PrintChar("PSB_PAD_RIGHT \n");  break;
						case PSB_PAD_DOWN:	CarState = BACK;  PrintChar("PSB_PAD_DOWN \n");  break; 
						case PSB_PAD_LEFT:	CarState = LEFT;  PrintChar("PSB_PAD_LEFT \n");  break; 
						case PSB_L2:      	break;//PrintChar("PSB_L2 \n");  break; 
						case PSB_R2:      	break;//PrintChar("PSB_R2 \n");  break; 
						case PSB_L1:      	break;//PrintChar("PSB_L1 \n");  break; 
						case PSB_R1:      	break;//PrintChar("PSB_R1 \n");  break;     
						case PSB_TRIAngle:	CarState = STOP;  break; 
						case PSB_CIRCLE:  	PrintChar("PSB_CIRCLE \n");  break; 
						case PSB_CROSS:   	PrintChar("PSB_CROSS \n");  break; 
						case PSB_SQUARE:  	PrintChar("PSB_SQUARE \n");  break;
						default: CarState = STOP; break; 
					}
					Red = PS2_RedLight();
					printf("redlight:%d\n",Red);
					X1 = PS2_AnologData(PSS_LX);
					Y1 = PS2_AnologData(PSS_LY);
					X2 = PS2_AnologData(PSS_RX);
					Y2 = PS2_AnologData(PSS_RY);
					if(Red == 1)
					{
						PS2_Speed = 0;
						PS2_Direction = 0;
						CarStateOut();
					}
					else
					{	
						PS2_Speed = -23*(Y1-127);
						PS2_Direction = 2*(X2-128);
					}
				}
				
		}
	}   


/*********************************************END OF FILE**********************/
