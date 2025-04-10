/**
*******************************************************************************
 * @file      :fdcan.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HOMEWORK_COMPONENTS_FDCAN_HPP_
#define HOMEWORK_COMPONENTS_FDCAN_HPP_

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

bool FdcanFilterInit(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifox);
bool SendCanMsg(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t tx_data[8]);
void FdcanInit();
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif /* HOMEWORK_COMPONENTS_FDCAN_HPP_ */