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

int main(void)
{
	int PS2_KEY = 0;			//用于接收PS2手柄按键数据存储
	int X1=0,Y1=0,X2=0,Y2=0;    //用于存储PS2遥感数据
	int Red=1;					//用于存储PS2模式选择数据
	int num = 0;				//进行PS2手柄接收数据时间间隔变量
	
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
	  
	delay_ms(500); 
	
	//系统定时器初始化
	SysTick_Init();     
	
	//使能系统定时器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
		while(1)
		{
			MPU6050_GetData();	
			num++;
			if(num == 200)
			{
				num =0;
				PS2_KEY = PS2_DataKey();	 //手柄按键捕获处理
				switch(PS2_KEY)
				{
					case PSB_SELECT: 	
						//PrintChar("PSB_SELECT \n");  
					    break;
					
					case PSB_L3:     	
						CarState = STOP;  
					    //PrintChar("PSB_L3 \n");  
					    break;  
					
					case PSB_R3:     	
						CarState = STOP;  
						//PrintChar("PSB_R3 \n");  
						break;  
					
					case PSB_START:  	
						//PrintChar("PSB_START \n");  
						break;  
					
					case PSB_PAD_UP: 	
						CarState = RUN;   
						//PrintChar("PSB_PAD_UP \n");  
						break;  
					
					case PSB_PAD_RIGHT:	
						CarState = RIGHT; 
						//PrintChar("PSB_PAD_RIGHT \n"); 
						break;
					
					case PSB_PAD_DOWN:	
						CarState = BACK;  
						//PrintChar("PSB_PAD_DOWN \n");  
						break; 
					
					case PSB_PAD_LEFT:	
						CarState = LEFT;  
						//PrintChar("PSB_PAD_LEFT \n");  
						break; 
					
					case PSB_L2:      	
						CAR_SPEED_MAX-=500;
						CAR_SPEED_MIN+=500;
						if(CAR_SPEED_MAX<=2000)
						{
							CAR_SPEED_MAX = 2000;
						}
						if(CAR_SPEED_MIN>=-2000)
						{
							CAR_SPEED_MIN = -2000;
						}
						break;
					
					case PSB_R2:      	
						TempPS2_Direction -= 50;
						if(TempPS2_Direction<=200)
						{
							TempPS2_Direction = 200;
						}
						break;
					
					case PSB_L1:      	
						CAR_SPEED_MAX+=500;
						CAR_SPEED_MIN-=500;
						if(CAR_SPEED_MAX>=7000)
						{
							CAR_SPEED_MAX = 7000;
						}
						if(CAR_SPEED_MIN<=-7000)
						{
							CAR_SPEED_MIN = -7000;
						}
						break; 
					
					case PSB_R1:      	
						TempPS2_Direction += 50;
						if(TempPS2_Direction>=600)
						{
							TempPS2_Direction = 600;
						}
						break;     
					
					case PSB_TRIAngle:	
						CarState = STOP;  
						break; 
					
					case PSB_CIRCLE:  	
						//PrintChar("PSB_CIRCLE \n");  
						break; 
					
					case PSB_CROSS:   	
						//PrintChar("PSB_CROSS \n");  
						break; 
					
					case PSB_SQUARE:  	
						//PrintChar("PSB_SQUARE \n"); 
						break;
					
					default: 
						CarState = STOP; 
						break; 
						 
				}
				Red = PS2_RedLight();
				X1 = PS2_AnologData(PSS_LX);
				Y1 = PS2_AnologData(PSS_LY);
				X2 = PS2_AnologData(PSS_RX);
				Y2 = PS2_AnologData(PSS_RY);
				//printf("Y1:%d\n",Y1);
				if(Red == 1)
				{
					PS2_Speed = 0;
					PS2_Direction = 0;
					CarStateOut();
				}
				else
				{	
					PS2_Speed = -7.8*(Y1-127);
					PS2_Direction = 3.5*(X2-128);
				}
			}
		}
	}   

