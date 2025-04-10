/**
 *******************************************************************************
 * @file      :main_task.hpp
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
#ifndef MAIN_TASK_HPP_
#define MAIN_TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include<stdbool.h>
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct UartCommData
{
    uint32_t tick;
    float value;
}UartCommData;

typedef struct CANCommData
{
    uint32_t tick;
    float value1;
    uint8_t value2;
    bool flag1;
    bool flag2;
    bool flag3;
    bool flag4;
}CANCommData;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

void MainInit(void);
void MainTask(void);
bool UartPack(uint8_t* uart_data);
bool CanPack(uint8_t* can_data);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_TASK_HPP_ */
