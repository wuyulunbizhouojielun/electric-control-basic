/**
 *******************************************************************************
 * @file      :GM6020.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1。<note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team，Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PROJECT_PATH_FILE_HPP_
#define PROJECT_PATH_FILE_HPP_
/* Includes ------------------------------------------------------------------*/
#include"stdint.h"
/* Exported macro ------------------------------------------------------------*/
# define VOL_2 0
# define VOL_3 0
# define VOL_4 0
# define VOL_5 0
# define VOL_6 0
# define VOL_7 0
# define VOL_8 0
# define kRPM2kRadS 0.10471975511965977  // rpm转换为rad/s
# define kRaw2Angle 0.021974672676002304 // 8192->360
# define kangle2krad 0.017453292519943295 // 角度转换为弧度
# define Max_PI 3.14159265358979323846
/* Exported types ------------------------------------------------------------*/

typedef struct Pid{
    float kp;
    float ki;
    float kd;
}Pid;

// 发送内容为电压，接收包括角度、转速、电流、温度
class GM6020
{
private:
    bool pidopen = true;  // 是否对电机进行pid控制
    bool spi_on = true;  // 是否为跟随板载imu的角度
    bool mode = false; // pid模式，false为转速，true为角度位置
    uint16_t rx_id_;  // 电机ID 0x0205
    uint16_t tx_id_; // 0x1FF
    float angle_;  // 弧度角
    float vel_;   // 转速，单位为rpm
    float current_;
    float temp_;
    int16_t ctrl_vol; // pid解算后的电压
    int16_t test_vol; // 测试用固定值
    float angle_target;  // 设定好目标角度(单位°) 这两个没关系，最后返回的控制电压是int16_t即可
    float speed_target; // 设定好目标速度(单位rad/s) 正弦变化
    Pid SpeedPid, AnglePid;  // 速度和角度的pid

public:
    GM6020(uint16_t rx_id, uint16_t tx_id):rx_id_(rx_id),tx_id_(tx_id),ctrl_vol(0),test_vol(10000),angle_target(90),speed_target(15)
    {
        // 首先p应该调到响应速度基本跟得上，然后动ki减少超调，最后考虑kd
        setSpeedPid(800, 0.2, 0.3);   // 速度pid 当sin频率为0.16Hz时该参数较好，但落后于幅值(800,0.2,0.3)
        setAnglePid(40, 10, 5);  // 角度(位置)pid
    };
    ~GM6020(){};
    void setSpeedPid(float kp, float ki, float kd);
    void setAnglePid(float kp, float ki, float kd);
    float getAngle();
    float getVel();
    float getCurrent(){return this->current_ ;}
    float getTemp(){return this->temp_ ;}
    float getSpeedTarget(){return this->speed_target;}
    float getAngleTarget(){return this->angle_target;}
    int16_t getCtrlVol(){return this->ctrl_vol;}
    bool encode(uint8_t* data, uint8_t len);
    bool decode(uint8_t* data, uint8_t len, uint16_t id); // 一定要注意这个id的长度，否则可能截断
    int16_t AnglePidControl();
    int16_t SpeedPidControl();
};


#endif /* PROJECT_PATH_FILE_HPP_ */
