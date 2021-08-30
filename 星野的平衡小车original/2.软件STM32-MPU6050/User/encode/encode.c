#include "encode.h"
#include "stm32f10x.h"
//#include "sys.h"

/*
编码器 ：Tim3和Tim4
*/

void encoder_Tim3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 1-1;					//这里我们把它设为1，即不分频
	TIM_TimeBaseStructure.TIM_Period = 65535;					//每来一个脉冲信号的上升沿（下面有设置）计数值就累加（或累减），65535则为最大计数值，就溢出了
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //这里按理来说应该不起作用，因为计数方向是受TI1和TI2信号的影响的
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
	TIM_ICStructInit(&TIM_ICInitStructure);						
	
	/*配置为编码器模式，计数器在TI1和TI2上升沿处均计数*/
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);		//清除TIM的更新标志位
	TIM_SetCounter(TIM3, 0);					//将脉冲计数值设为零
	TIM_Cmd(TIM3, ENABLE);						//使能TIM3
}

void encoder_Tim4_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* TIM3 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  /* Timer configuration in Encoder mode */
  TIM_DeInit(TIM4);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  					// No prescaling 
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  		//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //向上计数模式 
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge); //TIM_ICPolarity_Rising上升沿捕获
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6; 			//无滤波器
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
   
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	    //使能中断
  
  TIM_Cmd(TIM4, ENABLE);  	   						//使能定时器3
}

// 读取定时器3计数值
uint32_t read_cnt_TIM3(void)
{
	uint32_t encoder_cnt3;
	encoder_cnt3 = TIM3->CNT;		//读取计数器CNT的值，CNT系uint32_t型的变量
	TIM_SetCounter(TIM3, 0);		//每一次读取完计数值后将计数值清零，重新开始累加脉冲，方便下一次计数
	return encoder_cnt3;				//返回的值就是本次读到的计数值
}

// 读取定时器4计数值
uint32_t read_cnt_TIM4(void)
{
	uint32_t encoder_cnt4;
	encoder_cnt4 = TIM4->CNT;		//读取计数器CNT的值，CNT系uint32_t型的变量
	TIM_SetCounter(TIM4, 0);		//每一次读取完计数值后将计数值清零，重新开始累加脉冲，方便下一次计数
	return encoder_cnt4;				//返回的值就是本次读到的计数值
}

