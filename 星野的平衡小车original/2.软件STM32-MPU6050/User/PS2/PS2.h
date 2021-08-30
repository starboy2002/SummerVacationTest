
#ifndef __PS2_H__
#define __PS2_H__

#include "stm32f10x.h"
//#define DI    GPIOA->IDR  & GPIO_Pin_8           

//#define DO_H PAout(12)=1        //命令位高
//#define DO_L PAout(12)=0        //命令位低

//#define CS_H PAout(15)=1       //CS拉高
//#define CS_L PAout(15)=0       //CS拉低

//#define CLC_H PAout(11)=1      //时钟拉高
//#define CLC_L PAout(11)=0      //时钟拉低
//GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_TRIAngle    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16



//These are stick values
#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8



extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_DataKey(void);		  //键值读取
u8 PS2_AnologData(u8 button); //得到一个摇杆的模拟量
void PS2_ClearData(void);	  //清除数据缓冲区
void PS2_SetInit(void);
int PS2_RedLight(void);

#endif
