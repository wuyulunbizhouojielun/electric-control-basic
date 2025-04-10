/**
 *******************************************************************************
 * @file      :main_task.cpp
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
#include"main_task.hpp"
#include"stm32h7xx.h"   // 这三个文件是必须的，因为我们要使用STM32的库函数
#include"stm32h723xx.h"
#include"stm32h7xx_hal.h"
#include"math.h"
#include"tim.h"
#include"stdint.h"
#include"gpio.h"
#include"fdcan.hpp"
#include"GM6020.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;
uint32_t pwm = 0;
uint8_t tx_buffer[8] = {0};
extern GM6020 gm6020;
int angle_flap_flag = -1;

// 调试用的一堆变量查看，因为好像直接Ozone内无法显示对象内的变量，局部变量也看不到
float angle;
float vel;
float temperature;
float current;
float speed_target;
float angle_target;
int16_t control_vol;

void MainInit(void)
{
    // Initialize the main task
    tick = 0;
    FdcanInit();   // FDCAN初始化 from fdcan.cpp 包含了filterinit、start和notification
    HAL_TIM_Base_Start_IT(&htim2); 
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // 启动PWM
}

void MainTask(void)
{
    tick++;
    bool status = gm6020.encode(tx_buffer, 8);  // 编码
    angle = gm6020.getAngle();
    vel = gm6020.getVel();
    temperature = gm6020.getTemp();
    current = gm6020.getCurrent();
    speed_target = gm6020.getSpeedTarget();
    angle_target = gm6020.getAngleTarget();
    control_vol = gm6020.getCtrlVol();
    
    if(status)
    {
       SendCanMsg(&hfdcan1, 0x1FF, tx_buffer);  // 发送can信息，根据电机ID切换标识符，10.25测试发送正常
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // PWM 50Hz
  if (htim == &htim2)
  {
      angle_flap_flag = -angle_flap_flag;
      // GM6020输入PWM高电平1000-2000μs
      __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1000+pwm);  // 调整比较值，即占空比
      // HAL_Delay(5);  //该定时器也有优先级，会影响外部中断优先级
      pwm++;
      if(pwm >= 1000)
      {
          pwm = 0;
      }
  }
  
  // 1kHz
  // can通信直接在回调里写
  if(htim == &htim6)
  {
      MainTask();
  }

}

