/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */

// 调试相关全局变量
static uint8_t debug_initialized = 0;
static char debug_buffer[DEBUG_BUFFER_SIZE];

// 基础串口发送函数
HAL_StatusTypeDef USART_SendString(UART_HandleTypeDef *huart, const char *str)
{
    return HAL_UART_Transmit(huart, (uint8_t*)str, strlen(str), 1000);
}

HAL_StatusTypeDef USART_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit(huart, data, length, 1000);
}

// 调试信息输出函数
void Debug_Init(void)
{
    debug_initialized = 1;
    
    // 发送初始化信息
    const char *init_msg = "\r\n=== 三通道切换箱控制系统调试启动 ===\r\n";
    USART_SendString(&huart3, init_msg);
    
    // 打印系统信息
    Debug_PrintSystemInfo();
}

void Debug_Printf(Debug_Level_TypeDef level, const char *format, ...)
{
    if (!debug_initialized) return;
    
    va_list args;
    va_start(args, format);
    
    // 添加时间戳和级别标识
    uint32_t timestamp = Debug_GetTimestamp();
    int prefix_len = snprintf(debug_buffer, DEBUG_BUFFER_SIZE, 
                             "[%08lu][%s] ", timestamp, Debug_GetLevelString(level));
    
    // 格式化用户消息
    vsnprintf(debug_buffer + prefix_len, DEBUG_BUFFER_SIZE - prefix_len, format, args);
    
    va_end(args);
    
    // 添加换行符
    strcat(debug_buffer, "\r\n");
    
    // 发送调试信息
    USART_SendString(&huart3, debug_buffer);
}

void Debug_SendMessage(const char *message)
{
    if (!debug_initialized) return;
    
    snprintf(debug_buffer, DEBUG_BUFFER_SIZE, "%s\r\n", message);
    USART_SendString(&huart3, debug_buffer);
}

void Debug_SendHex(uint8_t *data, uint16_t length)
{
    if (!debug_initialized) return;
    
    USART_SendString(&huart3, "HEX: ");
    
    for (uint16_t i = 0; i < length; i++) {
        snprintf(debug_buffer, DEBUG_BUFFER_SIZE, "%02X ", data[i]);
        USART_SendString(&huart3, debug_buffer);
    }
    
    USART_SendString(&huart3, "\r\n");
}

// 系统状态监控函数
void Debug_PrintSystemInfo(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "系统信息:");
    Debug_Printf(DEBUG_LEVEL_INFO, "  MCU: STM32F103RCT6");
    Debug_Printf(DEBUG_LEVEL_INFO, "  系统时钟: %lu MHz", HAL_RCC_GetSysClockFreq() / 1000000);
    Debug_Printf(DEBUG_LEVEL_INFO, "  固件版本: v2.0");
    Debug_Printf(DEBUG_LEVEL_INFO, "  编译时间: %s %s", __DATE__, __TIME__);
}

void Debug_PrintGPIOStatus(void)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "GPIO状态:");
    
    // 通道使能状态
    Debug_Printf(DEBUG_LEVEL_DEBUG, "  通道使能: K1_EN=%d, K2_EN=%d, K3_EN=%d",
                GPIO_ReadChannelEnable(CHANNEL_1),
                GPIO_ReadChannelEnable(CHANNEL_2),
                GPIO_ReadChannelEnable(CHANNEL_3));
    
    // 继电器状态
    Debug_Printf(DEBUG_LEVEL_DEBUG, "  继电器状态: K1_1=%d, K1_2=%d, K2_1=%d, K2_2=%d, K3_1=%d, K3_2=%d",
                GPIO_ReadRelayStatus(RELAY_1_1),
                GPIO_ReadRelayStatus(RELAY_1_2),
                GPIO_ReadRelayStatus(RELAY_2_1),
                GPIO_ReadRelayStatus(RELAY_2_2),
                GPIO_ReadRelayStatus(RELAY_3_1),
                GPIO_ReadRelayStatus(RELAY_3_2));
    
    // 开关状态
    Debug_Printf(DEBUG_LEVEL_DEBUG, "  开关状态: SW1=%d, SW2=%d, SW3=%d",
                GPIO_ReadSwitchStatus(CHANNEL_1),
                GPIO_ReadSwitchStatus(CHANNEL_2),
                GPIO_ReadSwitchStatus(CHANNEL_3));
    
    // 电源和风扇状态
    Debug_Printf(DEBUG_LEVEL_DEBUG, "  电源状态: DC_CTRL=%d", GPIO_ReadDCStatus());
    Debug_Printf(DEBUG_LEVEL_DEBUG, "  风扇传感器: FAN_SEN=%d", GPIO_ReadFanSensor());
}

void Debug_PrintTemperatureInfo(float temp1, float temp2, float temp3)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "温度信息: NTC1=%.1f°C, NTC2=%.1f°C, NTC3=%.1f°C", 
                temp1, temp2, temp3);
}

void Debug_PrintRelayStatus(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器状态检查:");
    
    for (int i = 0; i < 6; i++) {
        uint8_t status = GPIO_ReadRelayStatus((Relay_TypeDef)i);
        const char* relay_names[] = {"K1_1", "K1_2", "K2_1", "K2_2", "K3_1", "K3_2"};
        Debug_Printf(DEBUG_LEVEL_INFO, "  %s: %s", relay_names[i], status ? "吸合" : "断开");
    }
}

void Debug_PrintErrorInfo(const char *error_type, const char *error_msg)
{
    Debug_Printf(DEBUG_LEVEL_ERROR, "错误信息 [%s]: %s", error_type, error_msg);
}

// 调试辅助函数
const char* Debug_GetLevelString(Debug_Level_TypeDef level)
{
    switch (level) {
        case DEBUG_LEVEL_ERROR: return "ERROR";
        case DEBUG_LEVEL_WARN:  return "WARN ";
        case DEBUG_LEVEL_INFO:  return "INFO ";
        case DEBUG_LEVEL_DEBUG: return "DEBUG";
        default: return "UNKN ";
    }
}

uint32_t Debug_GetTimestamp(void)
{
    return HAL_GetTick();
}

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART3_PARTIAL();

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
