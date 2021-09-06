#ifndef _BALANCEDCAR_H
#define _BALANCEDCAR_H

#define CAR_ZERO_ANGLE 			(0)		 //��ʼС������ṹ����С�����ڴ�ֱ����Ϊ��Ƕȵ����������Ҫ������������ֱ�����Ƕ�ֵ��

#define MOTOR_OUT_MAX           1000	   //ռ�ձ������ֵ
#define MOTOR_OUT_MIN         (-1000)   //ռ�ձȸ����ֵ

#define CAR_POSITION_MAX	 4000       //8000
#define CAR_POSITION_MIN	(-4000)     //-8000

#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0

/*С������״̬ö��*/
enum{
  STOP = 0,
  RUN,
  BACK,
  LEFT,
  RIGHT,
};

extern int SpeedControlCount;

void MotorOutput(void);
void AngleControl(void);
void GetMotorPulse(void);
void SpeedControl(void);

extern float PS2_Speed;
extern float PS2_Direction;
extern float TempPS2_Direction;

extern int CarState; //  1ǰ2��3��4��0ֹͣ

extern int CAR_SPEED_MAX;
extern int CAR_SPEED_MIN;

extern int SpeedLevelFlag_Forward;
extern int SpeedLevelFlag_Turn;

extern void CarStateOut(void); 
#endif /*BalancedCar.h*/
