#ifndef __ENCODE_H__ 
#define __ENCODE_H__

#include "stdint.h"
typedef enum
{
    MOTOR_LEFT  = 2,   // 左电机 -> TIM2
    MOTOR_RIGHT = 3    // 右电机 -> TIM3

} Motor_TypeDef;
int Read_Encoder(Motor_TypeDef motor);

#endif

