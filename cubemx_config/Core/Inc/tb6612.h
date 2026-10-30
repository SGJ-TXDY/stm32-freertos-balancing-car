#ifndef __TB6612_H__ 
#define __TB6612_H__

#include "stdint.h"

typedef enum
{
	FORWARD,
	BACKWARD,
	STOP,
}MOTO_DIR;
void startpwm();
void setpwm(int pwm_left, int pwm_right);
void Motor_Left(MOTO_DIR d);   // 左电机
void Motor_Right(MOTO_DIR d); // 右电机
void Motor_Stop(void);           // 停止
void Car_Go(void);			//开车
void Car_Go_High(void);
void Car_Go_Mid(void);
void Car_Go_Slow(void);
void Car_Trun(void);
void Car_Stop(void);
void Car_Back(void);

#endif

