#include "encode.h"
#include "stm32f10x.h"
//#include "sys.h"

/*
������ ��Tim3��Tim4
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

	TIM_TimeBaseStructure.TIM_Prescaler = 1-1;					//�������ǰ�����Ϊ1��������Ƶ
	TIM_TimeBaseStructure.TIM_Period = 65535;					//ÿ��һ�������źŵ������أ����������ã�����ֵ���ۼӣ����ۼ�����65535��Ϊ������ֵ���������
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ﰴ����˵Ӧ�ò������ã���Ϊ������������TI1��TI2�źŵ�Ӱ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);						
	
	/*����Ϊ������ģʽ����������TI1��TI2�����ش�������*/
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);		//���TIM�ĸ��±�־λ
	TIM_SetCounter(TIM3, 0);					//���������ֵ��Ϊ��
	TIM_Cmd(TIM3, ENABLE);						//ʹ��TIM3
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
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  		//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   //���ϼ���ģʽ 
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge); //TIM_ICPolarity_Rising�����ز���
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6; 			//���˲���
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
   
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	    //ʹ���ж�
  
  TIM_Cmd(TIM4, ENABLE);  	   						//ʹ�ܶ�ʱ��3
}

// ��ȡ��ʱ��3����ֵ
uint32_t read_cnt_TIM3(void)
{
	uint32_t encoder_cnt3;
	encoder_cnt3 = TIM3->CNT;		//��ȡ������CNT��ֵ��CNTϵuint32_t�͵ı���
	TIM_SetCounter(TIM3, 0);		//ÿһ�ζ�ȡ�����ֵ�󽫼���ֵ���㣬���¿�ʼ�ۼ����壬������һ�μ���
	return encoder_cnt3;				//���ص�ֵ���Ǳ��ζ����ļ���ֵ
}

// ��ȡ��ʱ��4����ֵ
uint32_t read_cnt_TIM4(void)
{
	uint32_t encoder_cnt4;
	encoder_cnt4 = TIM4->CNT;		//��ȡ������CNT��ֵ��CNTϵuint32_t�͵ı���
	TIM_SetCounter(TIM4, 0);		//ÿһ�ζ�ȡ�����ֵ�󽫼���ֵ���㣬���¿�ʼ�ۼ����壬������һ�μ���
	return encoder_cnt4;				//���ص�ֵ���Ǳ��ζ����ļ���ֵ
}

