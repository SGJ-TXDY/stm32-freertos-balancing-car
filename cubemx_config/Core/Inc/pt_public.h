#ifndef __PT_PUBLIC_H__
#define __PT_PUBLIC_H__

//目标模式
typedef enum WorkMode {
    PT_WORKMODE_FORWARD,                //向前
    PT_WORKMODE_BACKWARD,               //向后
    PT_WORKMODE_STOP,                  //停止
    PT_WORKMODE_TURNBACK,               //掉头
}PtWorkMode_en; 

//目标速度
typedef enum WorkSpeed {
    PT_WORKSPEED_LOW,                   //低速
    PT_WORKSPEED_MEDIA,                 //中速
    PT_WORKSPEED_HIGH,                  //高速
}WorkSpeed_en; 

// MPU6050数据结构体配置
typedef struct {
    // 原始数据
    short accel_x_raw;
    short accel_y_raw;
    short accel_z_raw;
	
    short gyro_x_raw;
    short gyro_y_raw;
    short gyro_z_raw;
    
    // 处理后的物理量
    float accel_x;   // m/s2
    float accel_y;   // m/s2
    float accel_z;   // m/s2
	
    float gyro_x;    // °/s
    float gyro_y;    // °/s
    float gyro_z;    // °/s
} MPU6050_Data;

/* 实时信息结构 */
typedef struct Globaldata {
    double BatVolt;                     //电池电压
    double PitchAngle;                  //俯仰角
    double CurSpeed;                    //当前速度
    
    PtWorkMode_en TargetMode;           //目标模式
    WorkSpeed_en TargetSpeed;           //目标速度
	MPU6050_Data data;
}PtGlobalData_t;

extern PtGlobalData_t gGlobalData;             //全局数据

#endif
