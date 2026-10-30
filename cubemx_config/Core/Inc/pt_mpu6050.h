#ifndef _PT_MPU6050_H 
#define _PT_MPU6050_H 

#include "i2c.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "pt_public.h"
#include <math.h>

// MPU6050从机地址配置
#define MPU6050_ADDR				(0x68 << 1)			
#define MPU6050_ADDR_READ		(((0x68 << 1) | (0x01))) 				

// MPU6050寄存器地址配置
#define MPU6050_SMPLRT_DIV        0x19						// 采样频率分频器
#define MPU6050_CONFIG           	0x1A						// 配置寄存器
#define MPU6050_GYRO_CONFIG       0x1B						// 陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG      0x1C						// 加速度计配置寄存器
#define MPU6050_ACCEL_XOUT_H      0x3B						// 加速度值,X轴高8位寄存器
#define MPU6050_ACCEL_XOUT_L      0x3C						// 加速度值,X轴低8位寄存器
#define MPU6050_ACCEL_YOUT_H      0x3D						// 加速度值,Y轴高8位寄存器
#define MPU6050_ACCEL_YOUT_L      0x3E						// 加速度值,Y轴低8位寄存器
#define MPU6050_ACCEL_ZOUT_H      0x3F						// 加速度值,Z轴高8位寄存器
#define MPU6050_ACCEL_ZOUT_L      0x40						// 加速度值,Z轴低8位寄存器
#define MPU6050_TEMP_OUT_H        0x41						// 温度值高8位寄s存器
#define MPU6050_TEMP_OUT_L        0x42						// 温度值低8位寄存器
#define MPU6050_GYRO_XOUT_H       0x43						// 陀螺仪值,X轴高8位寄存器
#define MPU6050_GYRO_XOUT_L       0x44						// 陀螺仪值,X轴低8位寄存器
#define MPU6050_GYRO_YOUT_H       0x45						// 陀螺仪值,Y轴高8位寄存器
#define MPU6050_GYRO_YOUT_L       0x46						// 陀螺仪值，Y轴低8位寄存器
#define MPU6050_GYRO_ZOUT_H       0x47						// 陀螺仪值,Z轴高8位寄存器
#define MPU6050_GYRO_ZOUT_L       0x48						// 陀螺仪值,Z轴低8位寄存器
#define MPU6050_PWR_MGMT_1        0x6B						// 电源管理寄存器1
#define MPU6050_PWR_MGMT_2        0x6C						// 电源管理寄存器2 
#define MPU6050_WHO_AM_I         	0x75						// 器件ID寄存器

// MPU6050灵敏度配置
#define MPU6050_ACCEL_SCALE_2G    16384.0f  // ±2g时的灵敏度
#define MPU6050_ACCEL_SCALE_4G    8192.0f   // ±4g时的灵敏度
#define MPU6050_ACCEL_SCALE_8G    4096.0f   // ±8g时的灵敏度
#define MPU6050_ACCEL_SCALE_16G   2048.0f   // ±16g时的灵敏度
#define MPU6050_GYRO_SCALE_250    131.0f    // ±250°/s时的灵敏度
#define MPU6050_GYRO_SCALE_500    65.5f     // ±500°/s时的灵敏度
#define MPU6050_GYRO_SCALE_1000   32.8f     // ±1000°/s时的灵敏度
#define MPU6050_GYRO_SCALE_2000   16.4f     // ±2000°/s时的灵敏度

// 选择灵敏度范围
#define MPU6050_ACCEL_FS_SEL      MPU6050_ACCEL_SCALE_2G
#define MPU6050_GYRO_FS_SEL       MPU6050_GYRO_SCALE_250

// 重力加速度
#define GRAVITY                 9.80665f

//  PI
#define M_PI 3.14159265358979323846



// 函数声明
extern int MPU6050_Init(void);
extern void MPU6050_Read_ALL(MPU6050_Data *data);
extern int mpu6050_write(unsigned char slaveaddr, unsigned char regaddr, int datalen, unsigned char *pdata);
extern int mpu6050_read(unsigned char slaveaddr, unsigned char regaddr, int datalen, unsigned char *pdata);
extern int DMP_Pitch(void);


#endif