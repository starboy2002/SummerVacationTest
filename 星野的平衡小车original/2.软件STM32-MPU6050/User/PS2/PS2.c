#include "ps2.h"
//#include "systick.h"
#include "sys.h"
#include "delay.h"


#define DO_H         GPIOA->BSRR = GPIO_Pin_12
#define DO_L         GPIOA->BRR  = GPIO_Pin_12
#define CLC_H        GPIOA->BSRR = GPIO_Pin_11
#define CLC_L        GPIOA->BRR  = GPIO_Pin_11
#define CS_H         GPIOA->BSRR = GPIO_Pin_1
#define CS_L         GPIOA->BRR  = GPIO_Pin_1


#define DI      GPIOA->IDR  & GPIO_Pin_8	 


u16 Handkey;
u8 Comd[2]={0x01,0x42};	//��ʼ�����������
u8 scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//{0x01,0x42,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};	// ���Ͷ�ȡ

u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
	};	//����ֵ�밴����


void PS2_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  	/*����GPIOA������ʱ��*/
	//GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_1 | GPIO_Pin_11;	 //DO  CS CLK
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      /*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//DI
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// ���ֱ���������
void PS2_Cmd(u8 CMD) 
{
  volatile u16 ref=0x01; 
  Data[1]=0; 
  for(ref=0x01;ref<0x0100;ref<<=1) 
  {
    if(ref&CMD) 
    {
     DO_H; // ���һλ����λ
    }
    else DO_L; 
    CLC_H; // ʱ������ 
    delay_us(10); 
    CLC_L; 
    delay_us(10); 
    CLC_H; 
    if(DI) 
    {Data[1]=ref|Data[1];}
  }
 delay_us(16);
}

// ��ȡ�ֱ����� 
void PS2_ReadData(void) 
{
  volatile u8 byte=0; 
  volatile u16 ref=0x01; 
  CS_L; 
  PS2_Cmd(Comd[0]); // ��ʼ����
  PS2_Cmd(Comd[1]); // ��������
  for(byte=2;byte<9;byte++) // ��ʼ�������� 
  {
    for(ref=0x01;ref<0x100;ref<<=1) 
    { 
      CLC_H; 
      delay_us(10); 
      CLC_L; 
      delay_us(10); 
      CLC_H; 
      if(DI) 
      {Data[byte]= ref|Data[byte];}
    }
    delay_us(16);
  }
  CS_H;
}   

// �Զ������� PS2 �����ݽ��д���,ֻ����������
//����Ϊ0��δ����Ϊ1
u8 PS2_DataKey() 
{
  u8 index; 
  PS2_ClearData(); 
  PS2_ReadData(); 
  Handkey=(Data[4]<<8)|Data[3]; // ���� 16������ ����Ϊ 0 �� δ����Ϊ 1 
  for(index=0;index<16;index++) 
  {
    if((Handkey&(1<<(MASK[index]-1)))==0) 
    return index+1;
  }
  return 0; // û���κΰ�������
}

void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01); 
	PS2_Cmd(0x42); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0x00); 
	PS2_Cmd(0x00); 
	CS_H;
	delay_us(16);
}

//��������
void PS2_EnterConfing(void)
{
	CS_L;
	delay_us(16); 
	PS2_Cmd(0x01); 
	PS2_Cmd(0x43); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0x01); 
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	CS_H;
	delay_us(16);
}

//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
CS_L; 
	PS2_Cmd(0x01); 
	PS2_Cmd(0x44); 
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00	������÷���ģʽ
	PS2_Cmd(0xEE); //Ox03 �������ã�������ͨ��������MODE������ģʽ��
					//0xEE ������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	CS_H;
	delay_us(16);
}

//������
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16); 
	PS2_Cmd(0x01); 
	PS2_Cmd(0x4D); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0x00); 
	PS2_Cmd(0X01); 
	CS_H;
	delay_us(16);
}

//��ɲ���������
void PS2_ExitConfing(void)
{
	CS_L;
	delay_us(16); 
	PS2_Cmd(0x01); 
	PS2_Cmd(0x43); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0x00); 
	PS2_Cmd(0x5A); 
	PS2_Cmd(0x5A); 
	PS2_Cmd(0x5A); 
	PS2_Cmd(0x5A); 
	PS2_Cmd(0x5A); 
	CS_H;
	delay_us(16);
}

//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	PS2_ShortPoll(); 
	PS2_ShortPoll(); 
	PS2_ShortPoll();
	PS2_EnterConfing();
	PS2_TurnOnAnalogMode(); //�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	//PS2_VibrationMode();	//������ģʽPS2_ExitConfing();	//��ɲ���������
	PS2_ExitConfing();	//��ɲ���������
}

int PS2_RedLight(void) 
{
  CS_L; 
  PS2_Cmd(Comd[0]); // ��ʼ����
  PS2_Cmd(Comd[1]); // ��������
  CS_H; 
  if( Data[1]== 0X73) return 0;
  else return 1;
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
