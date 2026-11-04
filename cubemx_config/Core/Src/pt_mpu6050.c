#include "pt_mpu6050.h"

extern I2C_HandleTypeDef hi2c2;

/*******************************************************************
    功能：MPU6050连续读取多个字节
    参数：
                reg:  起始寄存器地址
                buf:  存放数据缓冲区
              len:  读取数据长度
  返回值：
                成功返回0
                失败返回负值
*******************************************************************/
static unsigned char MPU6050_Read_Bytes(unsigned char reg, unsigned char* buf, unsigned char len)
{
    if (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, &reg, 1, 1000) != HAL_OK)
    {
        return -1;
    }

    if (HAL_I2C_Master_Receive(&hi2c2, MPU6050_ADDR_READ, buf, len, 1000) != HAL_OK)
    {
        return -2;
    }

    return 0;
}
/*******************************************************************
    功能：MPU6050初始化
    参数：
            缺省
  返回值：
                成功返回0
                失败返回-1
*******************************************************************/
int MPU6050_Init(void)
{
    if (mpu_init(NULL) ||
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) ||
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) ||
        mpu_set_sample_rate(200) ||
        dmp_load_motion_driver_firmware() ||
        dmp_set_orientation(136) || /* Identity sensor-to-body orientation. */
        dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL |
                           DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO) ||
        dmp_set_fifo_rate(200) || mpu_set_dmp_state(1)) {
        return -1;
    }
    return 0;
}

/*******************************************************************
    功能：读取MPU6050数据
    参数：
            data:  mpu6050数据结构体指针
  返回值：
                成功返回0
                失败返回-1
*******************************************************************/
void MPU6050_Read_ALL(MPU6050_Data* data)
{
    unsigned char buffer[14];

    if (MPU6050_Read_Bytes(MPU6050_ACCEL_XOUT_H, buffer, 14) == 0)
    {
        // 加速度计数据
        data->accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
        data->accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
        data->accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);

        // 陀螺仪数据
        data->gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
        data->gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
        data->gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

        // 转换为物理量
        // 加速度 (g -> m/s2)
        data->accel_x = data->accel_x_raw / MPU6050_ACCEL_FS_SEL * GRAVITY;
        data->accel_y = data->accel_y_raw / MPU6050_ACCEL_FS_SEL * GRAVITY;
        data->accel_z = data->accel_z_raw / MPU6050_ACCEL_FS_SEL * GRAVITY;

        // 陀螺仪 (°/s)
        data->gyro_x = data->gyro_x_raw / MPU6050_GYRO_FS_SEL;
        data->gyro_y = data->gyro_y_raw / MPU6050_GYRO_FS_SEL;
        data->gyro_z = data->gyro_z_raw / MPU6050_GYRO_FS_SEL;
    }
}

// mpu6050 写。调用方在调度器启动后须已持有 I2C 互斥锁。
int mpu6050_write(unsigned char slaveaddr, unsigned char regaddr, int datalen, unsigned char* pdata)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, slaveaddr << 1, regaddr, 1,
                                                 pdata, datalen, 2);
    if (I2C2_Check(status, "I2C ERR mpu_wr\r\n") == 1) {
        status = HAL_I2C_Mem_Write(&hi2c2, slaveaddr << 1, regaddr, 1,
                                   pdata, datalen, 2);
        I2C2_Check(status, "I2C ERR mpu_wr\r\n");
    }
    return status == HAL_OK ? 0 : -1;
}
// mpu6050 读。调用方在调度器启动后须已持有 I2C 互斥锁。
int mpu6050_read(unsigned char slaveaddr, unsigned char regaddr, int datalen, unsigned char* pdata)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, slaveaddr << 1, regaddr, 1,
                                                pdata, datalen, 2);
    if (I2C2_Check(status, "I2C ERR mpu_rd\r\n") == 1) {
        status = HAL_I2C_Mem_Read(&hi2c2, slaveaddr << 1, regaddr, 1,
                                  pdata, datalen, 2);
        I2C2_Check(status, "I2C ERR mpu_rd\r\n");
    }
    return status == HAL_OK ? 0 : -1;
}

/*
 * DMP FIFO is fed at 200 Hz and this task runs every 5 ms. One read per
 * cycle leaves older packets queued when the two clocks drift, and the
 * control loop then balances on stale pitch until the FIFO overflows.
 * Drain while the caller holds the I2C mutex and publish only the newest
 * packet only after reaching the end of the queue. A bounded drain must
 * never relabel an old packet as fresh: if more packets remain, discard
 * this result and let ControlTask disarm while later cycles catch up.
 * Any error after a partial drain also invalidates the entire result.
 */
#define DMP_FIFO_DRAIN_MAX 4

// Only a fully drained, valid FIFO read may replace the published attitude.
int DMP_Pitch(void)
{
    unsigned long timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    float q0, q1, q2, q3, sin_pitch, pitch;
    unsigned int reads;

    for (reads = 0; reads < DMP_FIFO_DRAIN_MAX; reads++) {
        if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0) {
            return reads == 0 ? DMP_PITCH_NO_SAMPLE : DMP_PITCH_UNSAFE;
        }
        if (!(sensors & INV_WXYZ_QUAT)) {
            return DMP_PITCH_UNSAFE;
        }
        if (more != 0) {
            continue;
        }
        /* Q30 fixed point must be scaled before products to avoid integer overflow. */
        q0 = quat[0] / 1073741824.0f;
        q1 = quat[1] / 1073741824.0f;
        q2 = quat[2] / 1073741824.0f;
        q3 = quat[3] / 1073741824.0f;
        sin_pitch = 2.0f * (q0 * q2 - q1 * q3);
        if (sin_pitch > 1.0f) sin_pitch = 1.0f;
        if (sin_pitch < -1.0f) sin_pitch = -1.0f;
        pitch = asinf(sin_pitch) * (180.0f / M_PI);
        if (!isfinite(pitch)) {
            return DMP_PITCH_UNSAFE;
        }
        gGlobalData.PitchAngle = pitch;
        gGlobalData.data.gyro_y = gyro[1] / MPU6050_GYRO_SCALE_2000;
        return DMP_PITCH_OK;
    }
    return DMP_PITCH_UNSAFE;
}
