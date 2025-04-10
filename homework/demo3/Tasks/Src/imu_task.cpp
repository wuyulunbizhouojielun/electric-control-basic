/**
*******************************************************************************
 * @file      :imu_task.cpp
* @brief     : IMU数据处理任务
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      2024-09-08      Jinletian       1. Create this file.
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "imu_task.hpp"
#include "imu.hpp"
#include "mahony.hpp"
#include "spi.h"

// 这些分类private大多是本文件自己用的变量和函数，external可以和外界通信

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::imu::BMI088* bmi088_ptr = nullptr;
hello_world::ahrs::Mahony* mahony_ptr = nullptr;   
float acc_data[3], gyro_data[3], temp;    // 读取加速度(acclerate)、陀螺仪(gyroscope)和温度(temperature)数据
float quat[4];
/* External variables --------------------------------------------------------*/

float euler_angles[3];
/* Private function prototypes -----------------------------------------------*/

static void Quat2Euler(float* quat, float* euler);

/**
 * @brief       IMU 初始化(初始化加速度计、陀螺仪和SPI的引脚)
 * @retval       None
 * @note        使用 IMU 前必须调用此函数进行初始化，并且注意该函数会阻塞直到 IMU 初始化完成
 */
void ImuInit()
{
    // 指定端口配置
    hello_world::imu::BMI088HWConfig default_params = {
        .hspi = &hspi2,
        .acc_cs_port = GPIOC,
        .acc_cs_pin = GPIO_PIN_0,
        .gyro_cs_port = GPIOC,
        .gyro_cs_pin = GPIO_PIN_3,
    };
    float rot_mat_flatten[9] = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    /**< Mahony 滤波器 kp 参数 */
    float mahony_kp = 0.5f;  ///< Mahony 滤波器 kp 参数
    float mahony_ki = 0.0f;  ///< Mahony 滤波器 ki 参数

    /**< Mahony 滤波器初始四元数，[w, x, y, z] */
    // 融合加速度计、陀螺仪和磁力计计算姿态
    float mahony_init_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float sample_freq = 1000.0f;
    bmi088_ptr = new hello_world::imu::BMI088(
        default_params, rot_mat_flatten);
    bmi088_ptr->imuInit();   // 可选是否进行自检测
    mahony_ptr = new hello_world::ahrs::Mahony(
        mahony_init_quat, sample_freq, mahony_kp, mahony_ki);
}

/**
 * @brief       IMU 更新
 * @retval       None
 * @note
 */
void ImuUpdate()
{
    // 在MainTask中调用 数据可以在本文件中处理
    bmi088_ptr->getData(acc_data, gyro_data, &temp);  // 内部已封装spi收发函数
    mahony_ptr->update(gyro_data, acc_data);     // 输入三轴惯性数据，解算出四元数
    mahony_ptr->getQuat(quat);
    Quat2Euler(quat, euler_angles);
}

/**
 * @brief       四元数转欧拉角
 * @param        q: 四元数，[w, x, y, z]
 * @param        euler: 欧拉角，[yaw, pitch, roll]，单位：rad
 * @retval       None
 * @note        涉及四元数转欧拉角的部分请调用该函数
 */
static void Quat2Euler(float* quat, float* euler)
{
    arm_atan2_f32(quat[0] * quat[1] + quat[2] * quat[3],
                  quat[0] * quat[0] + quat[3] * quat[3] - 0.5f,
                  euler + 2);
    float sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
    if (sinp > 1.0f)
        sinp = 1.0;
    else if (sinp < -1.0f)
        sinp = -1.0f;
    euler[1] = asinf(sinp);
    arm_atan2_f32(quat[0] * quat[3] + quat[1] * quat[2],
                  quat[0] * quat[0] + quat[1] * quat[1] - 0.5f,
                  euler + 0);
}