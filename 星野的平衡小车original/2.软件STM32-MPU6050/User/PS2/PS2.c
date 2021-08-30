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
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据
u8 scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//{0x01,0x42,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};	// 类型读取

u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
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
	};	//按键值与按键明


void PS2_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  	/*开启GPIOA的外设时钟*/
	//GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_1 | GPIO_Pin_11;	 //DO  CS CLK
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      /*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//DI
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 向手柄发送命令
void PS2_Cmd(u8 CMD) 
{
  volatile u16 ref=0x01; 
  Data[1]=0; 
  for(ref=0x01;ref<0x0100;ref<<=1) 
  {
    if(ref&CMD) 
    {
     DO_H; // 输出一位控制位
    }
    else DO_L; 
    CLC_H; // 时钟拉高 
    delay_us(10); 
    CLC_L; 
    delay_us(10); 
    CLC_H; 
    if(DI) 
    {Data[1]=ref|Data[1];}
  }
 delay_us(16);
}

// 读取手柄数据 
void PS2_ReadData(void) 
{
  volatile u8 byte=0; 
  volatile u16 ref=0x01; 
  CS_L; 
  PS2_Cmd(Comd[0]); // 开始命令
  PS2_Cmd(Comd[1]); // 请求数据
  for(byte=2;byte<9;byte++) // 开始接受数据 
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

// 对读出来的 PS2 的数据进行处理,只处理按键部分
//按下为0，未按下为1
u8 PS2_DataKey() 
{
  u8 index; 
  PS2_ClearData(); 
  PS2_ReadData(); 
  Handkey=(Data[4]<<8)|Data[3]; // 这是 16个按键 按下为 0 ， 未按下为 1 
  for(index=0;index<16;index++) 
  {
    if((Handkey&(1<<(MASK[index]-1)))==0) 
    return index+1;
  }
  return 0; // 没有任何按键按下
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

//进入配置
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

//发送模式设置
void PS2_TurnOnAnalogMode(void)
{
CS_L; 
	PS2_Cmd(0x01); 
	PS2_Cmd(0x44); 
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00	软件设置发送模式
	PS2_Cmd(0xEE); //Ox03 锁存设置，即不可通过按键“MODE”设置模式。
					//0xEE 不锁存软件设置，可通过按键“MODE”设置模式。
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	PS2_Cmd(0X00); 
	CS_H;
	delay_us(16);
}

//振动设置
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

//完成并保存配置
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

//手柄配置初始化
void PS2_SetInit(void)
{
	PS2_ShortPoll(); 
	PS2_ShortPoll(); 
	PS2_ShortPoll();
	PS2_EnterConfing();
	PS2_TurnOnAnalogMode(); //“红绿灯”配置模式，并选择是否保存
	//PS2_VibrationMode();	//开启震动模式PS2_ExitConfing();	//完成并保存配置
	PS2_ExitConfing();	//完成并保存配置
}

int PS2_RedLight(void) 
{
  CS_L; 
  PS2_Cmd(Comd[0]); // 开始命令
  PS2_Cmd(Comd[1]); // 请求数据
  CS_H; 
  if( Data[1]== 0X73) return 0;
  else return 1;
}

//得到一个摇杆的模拟量	 范围0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
