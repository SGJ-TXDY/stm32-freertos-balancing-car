#include "encode.h"
#include "tim.h"
#include "stm32f1xx_hal.h"

// 读取编码器值（HAL库版本）
// TIMX: 2 表示 TIM2, 3 表示 TIM3
// 返回：带符号的编码器计数值
int Read_Encoder(Motor_TypeDef motor)
{
    int Encoder_TIM = 0;

    switch (motor)
    {
        case MOTOR_LEFT:
            Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim2);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            break;

        case MOTOR_RIGHT:
            Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim3);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            break;

        default:
            Encoder_TIM = 0;
            break;
    }

    return Encoder_TIM;
}
