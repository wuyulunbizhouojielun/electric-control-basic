/**
 *******************************************************************************
 * @file      :GM6020.cpp
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
/* Includes ------------------------------------------------------------------*/
#include"GM6020.hpp"
#include"math.h"
#include<iostream>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern uint32_t tick;
extern int angle_flap_flag; // 控制角度位置翻转跟踪
/* Private function prototypes -----------------------------------------------*/

float GM6020::getAngle()
{
    return this->angle_;
}

float GM6020::getVel()
{
    return this->vel_;
}

void GM6020::setSpeedPid(float kp, float ki, float kd)
{
    this->SpeedPid.kd = kd;
    this->SpeedPid.ki = ki;
    this->SpeedPid.kp = kp;
}

void GM6020::setAnglePid(float kp, float ki, float kd)
{
    this->AnglePid.kd = kd;
    this->AnglePid.ki = ki;
    this->AnglePid.kp = kp;
}

bool GM6020::decode(uint8_t* data, uint8_t len, uint16_t id)
{
    if(id==this->rx_id_)
    {
        // 电机直接发过来的角度是0-8191，需要转换为0-360度，进一步再转为-180-180度
        int16_t angle = (data[0]<<8 | data[1]);
        float angle_temp = angle*kRaw2Angle*kangle2krad; // 0-2PI
        // -PI到PI
        if(angle_temp > Max_PI)
        {
            this->angle_ = angle_temp - 2*Max_PI;
        }
        else
        {
            this->angle_ = angle_temp;
        }
        int16_t vel = (data[2]<<8 | data[3]);
        this->vel_ = vel*kRPM2kRadS;  // 转速单位转换为rad/s
        this->current_ = (float)(data[4]<<8 | data[5]);
        this->temp_ = (float)(data[6]);
        return true;
    }
    return false;
}

// 根据报文内存地址顺序指定哪个电机
bool GM6020::encode(uint8_t* data, uint8_t len)
{
    // 编码前先更新pid
    if(mode)
    {
        this->ctrl_vol = AnglePidControl();
    }
    else
    {
        this->ctrl_vol = SpeedPidControl();
    }
    
    bool status = false;
    uint16_t VOL_1;
    if(pidopen)
    {
        VOL_1 = this->ctrl_vol;
    }
    else
    {
        VOL_1 = this->test_vol;
    }
    // 发送各个ID电机的电压，目前就一个，其他默认为0
    // 1-4
    if(tx_id_==0x1FF)
    {
       data[0] = (VOL_1>>8)&0xFF;
       data[1] = (VOL_1)&0xFF;
       data[2] = (VOL_2>>8)&0xFF;
       data[3] = (VOL_2)&0xFF;
       data[4] = (VOL_3>>8)&0xFF;
       data[5] = (VOL_3)&0xFF;
       data[6] = (VOL_4>>8)&0xFF;
       data[7] = (VOL_4)&0xFF;
       status = true;
    }
    // 5-7
    else if(tx_id_==0x2FF)
    {
       data[0] = (VOL_5>>8)&0xFF;
       data[1] = (VOL_5)&0xFF;
       data[2] = (VOL_6>>8)&0xFF;
       data[3] = (VOL_6)&0xFF;
       data[4] = (VOL_7>>8)&0xFF;
       data[5] = (VOL_7)&0xFF;
       data[6] = 0;
       data[7] = 0;     
       status = true;
    }
    else
    {
       std::cout<<"pack data failed!"<<std::endl;
    }
    return status;
}

// 有点奇怪，接收都是把角度换成弧度，但是这里控制的时候看例子又换回角度
int16_t GM6020::AnglePidControl()
{
    if(tick%1000==0)
    {
        this->angle_target *= -1;   // 尽量减少外部变量extern过来切换，否则延时很严重
    }
    float angle_current = this->getAngle()/kangle2krad;  // 调整回角度
    // 角度处理，可能是防止零漂
    if(this->angle_target - angle_current > 180)
    {
        angle_current += 360;
    }
    else if(this->angle_target - angle_current < -180)
    {
        angle_current -= 360;
    }
    float error = this->angle_target - angle_current;
    
    static float error_Int = 0;
    static float error_pre = 0;
    // 积分限幅
    if(error_Int > 200)
    {
        error_Int = 200;
    }
    else if(error_Int < -200)
    {
        error_Int = -200;
    }
    else
    {
        error_Int += error;
    }

    int16_t vol = (int16_t)(AnglePid.kp*error + AnglePid.ki*error_Int + AnglePid.kd*(error - error_pre));
    error_pre = error;
    // 电压限幅
    if(vol > 20000)
    {
        vol = 20000;
    }
    else if(vol < -20000)
    {
        vol = -20000;
    }

    return vol;
}

int16_t GM6020::SpeedPidControl()
{
    // 空载最大转速为320rpm，转换后大致33rad/s
    float vel_current = this->getVel();
    this->speed_target = 15*sin(tick*0.001);  // 正弦变化
    float error = this->speed_target - vel_current;
    
    static float error_Int = 0;
    static float error_pre = 0;
    // 积分限幅
    if(error_Int > 200)
    {
        error_Int = 200;
    }
    else if(error_Int < -200)
    {
        error_Int = -200;
    }
    else
    {
        error_Int += error;
    }

    int16_t vol = (int16_t)(SpeedPid.kp*error + SpeedPid.ki*error_Int + SpeedPid.kd*(error - error_pre));
    error_pre = error;
    // 电压限幅
    if(vol > 20000)
    {
        vol = 20000;
    }
    else if(vol < -20000)
    {
        vol = -20000;
    }

    return vol;
}