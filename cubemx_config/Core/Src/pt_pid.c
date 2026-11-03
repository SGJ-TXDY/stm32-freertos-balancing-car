#include "pt_pid.h"

//速度环输出
int sudu_out = 0;

//直立环输出
int zhili_out = 0;

//目标移动速度
int Movement = 0;

//目标转向角度
int turnment = 0;

//机械零点
float Car_zero = -1.8f; //0.3

//直立环
float zhili_Kp = 30.0f; //极性+  65*0.6 39                25.0
float zhili_Kd = -0.12f;     //极性-   -0.32*0.6  -0.192    -0.15

//速度环
float sudu_Kp = 8.0f;   //极性+   43  60              5.0
float sudu_Ki = 0.040f;  //极性+ kp/200  0.3              0.025

//直立环      Angle：俯仰角      Gyro：角速度(x/y)
int zhili(float Angle, float Gyro)
{
    float err;
    int pwm_zhili;
    err =   Angle - Car_zero;  //期望值-实际值，这里期望小车平衡，因此期望值就是机械中值
    pwm_zhili = zhili_Kp * err + Gyro * zhili_Kd;//计算平衡控制的电机PWM

    return pwm_zhili;
}

/* Float state: an int filter truncates Encoder_Least * 0.2, so errors of
 * a few counts never enter the integrator and the car drifts. */
static float Encoder;
static float Encoder_Integral;

void PID_Reset(void)
{
    Encoder = 0.0f;
    Encoder_Integral = 0.0f;
}

// 速度环
int sudu(int encoder_left, int encoder_right)
{
    float pwm;
    int Encoder_Least;

    Encoder_Least = (encoder_left + encoder_right) - Movement; //获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此次为零）
    Encoder = Encoder * 0.8f + (float)Encoder_Least * 0.2f;    //一阶低通滤波器
    Encoder_Integral += Encoder;                                //积分出位移 积分时间：5ms

    if (Encoder_Integral > 3500.0f)
    {
        Encoder_Integral = 3500.0f;  //积分限幅
    }
    if (Encoder_Integral < -3500.0f)
    {
        Encoder_Integral = -3500.0f; //积分限幅
    }

    pwm = sudu_Kp * Encoder + sudu_Ki * Encoder_Integral;     //速度PI控制器
    if ((gGlobalData.PitchAngle >= 80) || (gGlobalData.PitchAngle <= -80)) //小车跌倒后清零
    {
        Encoder_Integral = 0.0f;
    }

    return (int)pwm;
}