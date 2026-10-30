#ifndef _PT_PID_H 
#define _PT_PID_H

#include "pt_public.h"

//速度环输出
extern int sudu_out; 

//直立环输出
extern int zhili_out;       

//目标移动速度
extern int Movement; 

//目标转向角度
extern int turnment;   

//机械零点
extern float Car_zero;//0.3

//直立环
extern float zhili_Kp;  //极性+  65*0.6 39                25.0
extern float zhili_Kd;       //极性-   -0.32*0.6  -0.192    -0.15

//速度环
extern float sudu_Kp;     //极性+   43  60              5.0
extern float sudu_Ki;    //极性+ kp/200  0.3              0.025

// 函数声明
void PID_Reset(void);
extern int zhili(float Angle,float Gyro);
extern int sudu(int encoder_left,int encoder_right);


#endif