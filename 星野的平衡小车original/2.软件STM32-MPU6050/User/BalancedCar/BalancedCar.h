#ifndef _BALANCEDCAR_H
#define _BALANCEDCAR_H

#define CAR_ZERO_ANGLE 			(0)		 //初始小车车体结构导致小车存在垂直方向不为零角度的情况，固需要消除此误差，增加直立误差角度值。

#define MOTOR_OUT_MAX           1000	   //占空比正最大值
#define MOTOR_OUT_MIN         (-1000)   //占空比负最大值

#define CAR_POSITION_MAX	 6000       //8000
#define CAR_POSITION_MIN	(-6000)     //-8000

#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0

/*小车运行状态枚举*/
enum{
  STOP = 0,
  RUN,
  BACK,
  LEFT,
  RIGHT,
};

extern int BST_u8SpeedControlCount;

void MotorOutput(void);
void AngleControl(void);
void GetMotorPulse(void);
void SpeedControl(void);

extern float PS2_Speed;
extern float PS2_Direction;
extern int CarState; //  1前2后3左4右0停止

extern void CarStateOut(void); 
#endif /*BalancedCar.h*/
