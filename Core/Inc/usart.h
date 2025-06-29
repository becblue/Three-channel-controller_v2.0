/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
// ���Լ�����
typedef enum {
    DEBUG_LEVEL_ERROR = 0,  // ������Ϣ
    DEBUG_LEVEL_WARN,       // ������Ϣ
    DEBUG_LEVEL_INFO,       // һ����Ϣ
    DEBUG_LEVEL_DEBUG       // ������Ϣ
} Debug_Level_TypeDef;

// ���Ի�������С
#define DEBUG_BUFFER_SIZE   256
#define DEBUG_QUEUE_SIZE    10

// ������Ϣ�ṹ��
typedef struct {
    Debug_Level_TypeDef level;
    char message[DEBUG_BUFFER_SIZE];
    uint32_t timestamp;
} Debug_Message_TypeDef;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
// �������ڷ��ͺ���
HAL_StatusTypeDef USART_SendString(UART_HandleTypeDef *huart, const char *str);
HAL_StatusTypeDef USART_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);

// ������Ϣ�������
void Debug_Init(void);
void Debug_Printf(Debug_Level_TypeDef level, const char *format, ...);
void Debug_SendMessage(const char *message);
void Debug_SendHex(uint8_t *data, uint16_t length);

// ϵͳ״̬��غ���
void Debug_PrintSystemInfo(void);
void Debug_PrintGPIOStatus(void);
void Debug_PrintTemperatureInfo(float temp1, float temp2, float temp3);
void Debug_PrintRelayStatus(void);
void Debug_PrintErrorInfo(const char *error_type, const char *error_msg);

// ���Ը�������
const char* Debug_GetLevelString(Debug_Level_TypeDef level);
uint32_t Debug_GetTimestamp(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

