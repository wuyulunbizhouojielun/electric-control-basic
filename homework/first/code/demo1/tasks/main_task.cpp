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
#include"stm32f103xb.h"
#include"stm32f1xx.h"
#include"stm32f1xx_hal.h"
#include"main_task.hpp"
#include"math.h"
#include"tim.h"
#include"stdint.h"
#include"gpio.h"
#include"usart.h"
#include"can.h"
#include"HW_can.hpp"
#include"string.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;
uint32_t uart_decode_success = 0;
uint32_t can_decode_success = 0;
float value = 0;
uint8_t uart_tx_byte[9] = {0};
uint8_t uart_rx_byte[9] = {0};
uint8_t can_tx_byte[8] = {0};
GPIO_PinState pin_state = GPIO_PIN_RESET;
UartCommData uart_tx_data, uart_rx_data;
CANCommData can_tx_data, can_rx_data1;

// 调试用
uint8_t tx_byte = 2;
uint8_t rx_byte;

void RobotInit()
{
    tick = 0;
}

bool UartPack(uint8_t* uart_data)
{
    if(uart_data==nullptr)
    {
        return false;
    }
    uart_data[0] = 0xAA;
    uart_data[1] = 0xBB;
    uart_data[2] = 0xCC;
    uart_data[3] = ((tick >> 24)&(0x000000ff));
    uart_data[4] = ((tick >> 16)&(0x000000ff));
    uart_data[5] = ((tick >> 8)&(0x000000ff));
    uart_data[6] = ((tick >> 0)&(0x000000ff));
    uart_data[7] = ((int16_t)(value*30000)>>8);
    uart_data[8] = ((int16_t)(value*30000)>>0);

    return true;
}

bool CanPack(uint8_t* can_data)
{
    if(can_data == nullptr)
    {
        return false;
    }
    // struct 初始化
    can_tx_data.tick = tick;
    can_tx_data.value1 = value;
    can_tx_data.value2 = 3;   // 暂时不知道用来干什么

    if(value<0)
    {
        can_tx_data.flag1 = 1;
        can_tx_data.flag2 = 1;
        can_tx_data.flag3 = 1;
        can_tx_data.flag4 = 1;
    }
    else
    {
        can_tx_data.flag1 = 0;
        can_tx_data.flag2 = 0;
        can_tx_data.flag3 = 0;
        can_tx_data.flag4 = 0;
    }

    can_data[0] = tick >> 24;
    can_data[1] = tick >> 16;
    can_data[2] = tick >> 8;
    can_data[3] = tick >> 0;
    can_data[4] = ((int16_t)(value*30000))>>8;
    can_data[5] = ((int16_t)(value*30000))>>0;
    can_data[6] = can_tx_data.value2;
    can_data[7] = (can_tx_data.flag1) << 6 | (can_tx_data.flag2) << 4 | (can_tx_data.flag3) << 2 | (can_tx_data.flag4);

    return true;
    
}

void MainInit(void)
{
    // 根据实际接线情况调整端口
    tick = 0;
    CanFilter_Init();
    HAL_CAN_Start(&hcan);  // 打开can
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 使能CAN的FIFO消息挂起中断请求 接受完数据后再次使能
    HAL_UART_Receive_IT(&huart2, uart_rx_byte, 9); // 主循环前开启接收中断 接收固定大小的数据到缓冲区
    // HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // 哪个口接收写哪个
    HAL_TIM_Base_Start_IT(&htim3); // 启动定时器的中断模式 频率见时钟配置
    
}

void MainTask(void)
{
    tick++;
    value = sin(0.001*tick);  // 乘法和除法速率相差巨大
    uart_tx_data.tick = tick;  // 1kHz更新
    uart_tx_data.value = value;
    // GPIO引脚跳变 亮0.5s灭0.5s
    if(tick%500==0)
    {
        pin_state = (pin_state == GPIO_PIN_SET)?GPIO_PIN_RESET:GPIO_PIN_SET; // 切换电平
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, pin_state);
    }
    // 串口发送频率10Hz
    if(tick%100==1)
    {
        UartPack(uart_tx_byte);
        HAL_UART_Transmit_IT(&huart1, uart_tx_byte, 9); // 配套Receive_IT?总之用下面的不开中断发多字节就寄
        // HAL_UART_Transmit(&huart1, uart_tx_byte, 9, 1); // 串口发送
        // HAL_UART_Transmit(&huart1, &tx_byte, 1, 0xffff);  // 哪个口发写哪个
    }
    // // can
    if(tick%100==1)
    {
        // encode data
        CanPack(can_tx_byte);
        //

        CAN_Send_Msg(&hcan, can_tx_byte, 0x100, 8);  // 向can总线发送数据
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 处理TIM3中断事件
    if(htim == &htim3)
    {
        MainTask();
    }
}

bool UartUnpack(uint8_t *uart_rx_byte)
{
    uint32_t temp1 = (uart_rx_byte[3] << 24);
    uint32_t temp2 = (uart_rx_byte[4] << 16);
    uint32_t temp3 = (uart_rx_byte[5] << 8);
    uint32_t temp4 = (uart_rx_byte[6] << 0);
    int16_t temp5 = (uart_rx_byte[8] << 8);
    int16_t temp6 = (uart_rx_byte[7] << 0);
    uart_rx_data.tick = temp1 | temp2 | temp3 | temp4;
    uart_rx_data.value = (temp5 | temp6)*0.0000333;

    return true;
}

// HAL_UART_Receive_IT函数启动串口接收并且接收到指定数量的数据后，HAL库会自动调用 HAL_UART_RxCpltCallback函数，用户自定义
// UART1发送至UART2
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart2)
    {
        // 串口接收处理
        UartUnpack(uart_rx_byte);
        uart_decode_success++;
    }
    HAL_UART_Receive_IT(&huart2, uart_rx_byte, 9);  // 启动新的接收
    // HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

// can通信的回调函数在HW_Cpp里