
#include "tb6612.h"

#include "stm32f1xx_hal.h"
#include "tim.h"
#include "pt_public.h"
/*
    输入值val =  0-3600
    val/3600 *100 是占空比

*/
int myabs(int a)
{
    int temp;
    if (a < 0)  temp = -a;
    else temp = a;
    return temp;
}

void startpwm()
{
    //开始输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Motor_Left_Forward(void)
{
    // AIN1=1, AIN2=0
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
}

void Motor_Left_Backward(void)
{
    // AIN1=0, AIN2=1
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
}

void Motor_Right_Forward(void)
{
    // BIN1=1, BIN2=0
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
}

void Motor_Right_Backward(void)
{
    // BIN1=0, BIN2=1
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin | BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
}

void Motor_EmergencyStop(void)
{
    /* Do not call setpwm(): stopping must not change the direction pins first. */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    Motor_Stop();
}

void Motor_Left(MOTO_DIR d)
{
    if (d == FORWARD)     //左电机前进
    {
        HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
        //        AIN1(0);
        //        AIN2(1);
    }
    if (d == BACKWARD)  //左电机后退
    {
        HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
        //        AIN1(1);
        //        AIN2(0);
    }
    if (d == STOP) //停转
    {
        HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
        //        AIN1(1);
        //        AIN2(1);
    }
}
void Motor_Right(MOTO_DIR d)
{

    if (d == FORWARD)     //右电机前进
    {
        HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
        //        BIN1(0);
        //        BIN2(1);
    }
    if (d == BACKWARD)  //右电机后退
    {
        HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);

        //        BIN1(1);
        //        BIN2(0);
    }
    if (d == STOP) //停转
    {
        HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
        //        BIN1(1);
        //        BIN2(1);
    }
}

void setpwm(int pwm_left, int pwm_right)
{
    if (pwm_left > 3600) pwm_left = 3600;
    if (pwm_left < -3600) pwm_left = -3600;
    if (pwm_right > 3600) pwm_right = 3600;
    if (pwm_right < -3600) pwm_right = -3600;
    if (pwm_left < 0)
    {
        //AIN1(1);AIN2(0);}
        Motor_Left_Forward();
    }
    else
    {
        Motor_Left_Backward();
        //            AIN1(0);
        //            AIN2(1);
    }
    // 通道3
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, myabs(pwm_left));

    if (pwm_right < 0)
    {
        //            BIN1(1);
        //            BIN2(0);
        Motor_Right_Forward();
    }
    else {
        //            BIN1(0);
        //            BIN2(1);
        Motor_Right_Backward();
    }

    // 通道4 占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, myabs(pwm_right));
}

void Car_Go(void)
{
    Motor_Left(FORWARD);
    Motor_Right(FORWARD);
    gGlobalData.TargetMode = PT_WORKMODE_FORWARD;
}
void Car_Back(void)
{
    Motor_Left(BACKWARD);
    Motor_Right(BACKWARD);
    gGlobalData.TargetMode = PT_WORKMODE_BACKWARD;
}
void Car_Stop(void)
{
    Motor_Left(STOP);
    Motor_Right(STOP);
    gGlobalData.TargetMode = PT_WORKMODE_STOP;
}
void Car_Trun(void)
{
    Motor_Left(FORWARD);
    Motor_Right(BACKWARD);
    gGlobalData.TargetMode = PT_WORKMODE_TURNBACK;
}

void Car_Go_Slow(void)
{
    setpwm(1000, 1000);
    Car_Go();
    gGlobalData.TargetSpeed = PT_WORKSPEED_LOW;
}
void Car_Go_Mid(void)
{
    setpwm(2000, 2000);
    Car_Go();
    gGlobalData.TargetSpeed = PT_WORKSPEED_MEDIA;
}
void Car_Go_High(void)
{
    setpwm(3000, 3000);
    Car_Go();
    gGlobalData.TargetSpeed = PT_WORKSPEED_HIGH;
}
