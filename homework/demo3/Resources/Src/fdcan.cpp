/**
*******************************************************************************
 * @file      :fdcan.cpp
* @brief     : FDCAN配置
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "fdcan.hpp"
#include"stdint.h"
#include"GM6020.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];

uint32_t can_rec_times = 0;
uint32_t can_success_times=0;
GM6020 gm6020(0x205, 0x1FF);   // 默认电机ID 1

/**
 * @brief       配置FDCAN过滤器，接收所有ID的数据帧
 * @param        hfdcan:
 * @param        rx_fifox: 接收FIFO编号
 * @arg         FDCAN_FILTER_TO_RXFIFO0
 * @arg         FDCAN_FILTER_TO_RXFIFO1
 * @retval       是否配置成功
 * @note        None
 */
bool FdcanFilterInit(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifox)
{
  FDCAN_FilterTypeDef filter_config{
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterConfig = rx_fifox,
      .FilterID1 = 0x000,
      .FilterID2 = 0x000,
      .RxBufferIndex = 0,
      .IsCalibrationMsg = 0,
  };

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
    return false;
  }

  return true;
}

/**
 * @brief       发送FDCAN数据帧
 * @param        hfdcan:
 * @param        std_id: CAN标准ID
 * @param        tx_data: 发送数据
 * @retval       是否发送成功
 * @note        None
 */
bool SendCanMsg(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t tx_data[8])
{
  FDCAN_TxHeaderTypeDef tx_header{
      .Identifier = std_id,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0,
  };

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data) != HAL_OK) {
    return false;
  }

  return true;
}

/**
 * @brief       FDCAN初始化
 * @retval       None
 * @note        None
 */

// 相当于打开了两个can，实际上都需要使用吗？
void FdcanInit()
{
  FdcanFilterInit(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0);
  // FdcanFilterInit(&hfdcan2, FDCAN_FILTER_TO_RXFIFO1);
  HAL_FDCAN_ConfigInterruptLines(
        &hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
  /* HAL_FDCAN_ConfigInterruptLines(
        &hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1); */
  HAL_FDCAN_Start(&hfdcan1);
  // HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  // HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}



/**
 *description  
 * @retval      None
 */

// FDCAN有两个可以配置的FIFO，FIFO0和FIFO1，这里我们只使用FIFO0
// 10.25亲测可用，注意H7的板子也要接上24V的电源，调试烧录代码建议关闭电源，防止烧接口
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_rec_times++;
    // uint32_t header_temp = RxHeader.Identifier;  
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) // 获得接收到的数据头和数据
    {

        bool status = gm6020.decode(RxData, 8, RxHeader.Identifier);  // 解包 确实是Identifier代表电机ID
        if(status)
        {
            can_success_times++;
        }
    }
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 再次使能FIFO0接收中断
}