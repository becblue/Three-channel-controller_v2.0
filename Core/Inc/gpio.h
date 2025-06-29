/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
// 通道使能状态枚举
typedef enum {
    CHANNEL_1 = 0,
    CHANNEL_2,
    CHANNEL_3
} Channel_TypeDef;

// 继电器状态枚举
typedef enum {
    RELAY_OFF = 0,
    RELAY_ON
} Relay_State_TypeDef;

// 继电器控制枚举
typedef enum {
    RELAY_1_1 = 0,
    RELAY_1_2,
    RELAY_2_1,
    RELAY_2_2,
    RELAY_3_1,
    RELAY_3_2
} Relay_TypeDef;

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
// GPIO状态读取函数
uint8_t GPIO_ReadChannelEnable(Channel_TypeDef channel);
uint8_t GPIO_ReadRelayStatus(Relay_TypeDef relay);
uint8_t GPIO_ReadSwitchStatus(Channel_TypeDef channel);
uint8_t GPIO_ReadDCStatus(void);
uint8_t GPIO_ReadFanSensor(void);

// GPIO状态设置函数
void GPIO_SetRelayOn(Relay_TypeDef relay);
void GPIO_SetRelayOff(Relay_TypeDef relay);
void GPIO_SetAlarm(uint8_t state);
void GPIO_SetBeep(uint8_t state);
void GPIO_SetRS485DE(uint8_t state);

// 引脚状态检测函数
uint8_t GPIO_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expectedState, uint8_t checkTimes, uint32_t interval);
uint8_t GPIO_CheckRelayStatusStable(Relay_TypeDef relay, uint8_t expectedState, uint8_t checkTimes, uint32_t interval);
uint8_t GPIO_CheckSwitchStatusStable(Channel_TypeDef channel, uint8_t expectedState, uint8_t checkTimes, uint32_t interval);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

