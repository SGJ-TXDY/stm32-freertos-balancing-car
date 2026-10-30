#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "main.h"
#include <stdint.h>
#include "pt_public.h"


/* 
//目标模式
typedef enum WorkMode {
    PT_WORKMODE_FORWARD,                 //向前 F 0 
    PT_WORKMODE_BACKWARD,                //向后 B 1
    PT_WORKMODE_STOP,                    //停止 T 2 
    PT_WORKMODE_TURNBACK,                //掉头 R 3
}PtWorkMode_en; 

//目标速度
typedef enum WorkSpeed {
    PT_WORKSPEED_LOW,                   //低速  0 
    PT_WORKSPEED_MEDIA,                 //中速  1
    PT_WORKSPEED_HIGH,                  //高速  2
}WorkSpeed_en; 

// 实时信息结构 
typedef struct Globaldata {
    double BatVolt;                     //电池电压
    double PitchAngle;                  //俯仰角
    double CurSpeed;                    //当前速度
    
    PtWorkMode_en TargetMode;           //目标模式
    WorkSpeed_en TargetSpeed;           //目标速度
}PtGlobalData_t;

PtGlobalData_t gGlobalData;             //全局数据
*/




///* 小车显示数据结构 */
//typedef struct
//{
//    float battery_voltage;   /* 电压，单位 V */
//    float angle_deg;         /* 倾角，单位 deg */
//    int16_t speed_rpm;       /* 速度，单位 RPM */
//} CarDisplayData_t;

/* 接口函数 */
void Display_Init(void);
void Display_ShowStartupPage(void);
void Display_ShowMainPage(PtGlobalData_t *data);

#endif
