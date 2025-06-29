/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "tim.h"

// GPIO×´Ì¬¶ÁÈ¡º¯Êý
uint8_t GPIO_ReadChannelEnable(Channel_TypeDef channel)
{
    uint8_t state = 0;
    switch(channel)
    {
        case CHANNEL_1:
            state = HAL_GPIO_ReadPin(K1_EN_GPIO_Port, K1_EN_Pin);
            break;
        case CHANNEL_2:
            state = HAL_GPIO_ReadPin(K2_EN_GPIO_Port, K2_EN_Pin);
            break;
        case CHANNEL_3:
            state = HAL_GPIO_ReadPin(K3_EN_GPIO_Port, K3_EN_Pin);
            break;
        default:
            break;
    }
    return state;
}

uint8_t GPIO_ReadRelayStatus(Relay_TypeDef relay)
{
    uint8_t state = 0;
    switch(relay)
    {
        case RELAY_1_1:
            state = HAL_GPIO_ReadPin(K1_1_STA_GPIO_Port, K1_1_STA_Pin);
            break;
        case RELAY_1_2:
            state = HAL_GPIO_ReadPin(K1_2_STA_GPIO_Port, K1_2_STA_Pin);
            break;
        case RELAY_2_1:
            state = HAL_GPIO_ReadPin(K2_1_STA_GPIO_Port, K2_1_STA_Pin);
            break;
        case RELAY_2_2:
            state = HAL_GPIO_ReadPin(K2_2_STA_GPIO_Port, K2_2_STA_Pin);
            break;
        case RELAY_3_1:
            state = HAL_GPIO_ReadPin(K3_1_STA_GPIO_Port, K3_1_STA_Pin);
            break;
        case RELAY_3_2:
            state = HAL_GPIO_ReadPin(K3_2_STA_GPIO_Port, K3_2_STA_Pin);
            break;
        default:
            break;
    }
    return state;
}

uint8_t GPIO_ReadSwitchStatus(Channel_TypeDef channel)
{
    uint8_t state = 0;
    switch(channel)
    {
        case CHANNEL_1:
            state = HAL_GPIO_ReadPin(SW1_STA_GPIO_Port, SW1_STA_Pin);
            break;
        case CHANNEL_2:
            state = HAL_GPIO_ReadPin(SW2_STA_GPIO_Port, SW2_STA_Pin);
            break;
        case CHANNEL_3:
            state = HAL_GPIO_ReadPin(SW3_STA_GPIO_Port, SW3_STA_Pin);
            break;
        default:
            break;
    }
    return state;
}

uint8_t GPIO_ReadDCStatus(void)
{
    return HAL_GPIO_ReadPin(DC_CTRL_GPIO_Port, DC_CTRL_Pin);
}

uint8_t GPIO_ReadFanSensor(void)
{
    return HAL_GPIO_ReadPin(FAN_SEN_GPIO_Port, FAN_SEN_Pin);
}

// GPIO×´Ì¬ÉèÖÃº¯Êý
void GPIO_SetRelayOn(Relay_TypeDef relay)
{
    switch(relay)
    {
        case RELAY_1_1:
            HAL_GPIO_WritePin(K1_1_ON_GPIO_Port, K1_1_ON_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_1_2:
            HAL_GPIO_WritePin(K1_2_ON_GPIO_Port, K1_2_ON_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_2_1:
            HAL_GPIO_WritePin(K2_1_ON_GPIO_Port, K2_1_ON_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_2_2:
            HAL_GPIO_WritePin(K2_2_ON_GPIO_Port, K2_2_ON_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_3_1:
            HAL_GPIO_WritePin(K3_1_ON_GPIO_Port, K3_1_ON_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_3_2:
            HAL_GPIO_WritePin(K3_2_ON_GPIO_Port, K3_2_ON_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

void GPIO_SetRelayOff(Relay_TypeDef relay)
{
    switch(relay)
    {
        case RELAY_1_1:
            HAL_GPIO_WritePin(K1_1_OFF_GPIO_Port, K1_1_OFF_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_1_2:
            HAL_GPIO_WritePin(K1_2_OFF_GPIO_Port, K1_2_OFF_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_2_1:
            HAL_GPIO_WritePin(K2_1_OFF_GPIO_Port, K2_1_OFF_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_2_2:
            HAL_GPIO_WritePin(K2_2_OFF_GPIO_Port, K2_2_OFF_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_3_1:
            HAL_GPIO_WritePin(K3_1_OFF_GPIO_Port, K3_1_OFF_Pin, GPIO_PIN_RESET);
            break;
        case RELAY_3_2:
            HAL_GPIO_WritePin(K3_2_OFF_GPIO_Port, K3_2_OFF_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

void GPIO_SetAlarm(uint8_t state)
{
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void GPIO_SetBeep(uint8_t state)
{
    HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void GPIO_SetRS485DE(uint8_t state)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Òý½Å×´Ì¬¼ì²âº¯Êý
uint8_t GPIO_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expectedState, uint8_t checkTimes, uint32_t interval)
{
    uint8_t i;
    for(i = 0; i < checkTimes; i++)
    {
        if(GPIO_ReadChannelEnable(channel) != expectedState)
        {
            return 0;
        }
        HAL_Delay(interval);
    }
    return 1;
}

uint8_t GPIO_CheckRelayStatusStable(Relay_TypeDef relay, uint8_t expectedState, uint8_t checkTimes, uint32_t interval)
{
    uint8_t i;
    for(i = 0; i < checkTimes; i++)
    {
        if(GPIO_ReadRelayStatus(relay) != expectedState)
        {
            return 0;
        }
        HAL_Delay(interval);
    }
    return 1;
}

uint8_t GPIO_CheckSwitchStatusStable(Channel_TypeDef channel, uint8_t expectedState, uint8_t checkTimes, uint32_t interval)
{
    uint8_t i;
    for(i = 0; i < checkTimes; i++)
    {
        if(GPIO_ReadSwitchStatus(channel) != expectedState)
        {
            return 0;
        }
        HAL_Delay(interval);
    }
    return 1;
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, K1_1_ON_Pin|K1_1_OFF_Pin|K2_1_ON_Pin|K2_1_OFF_Pin
                          |K3_1_OFF_Pin|K3_1_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, K1_2_OFF_Pin|K2_2_ON_Pin|K2_2_OFF_Pin|K3_2_OFF_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, K1_2_ON_Pin|ALARM_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(K3_2_ON_GPIO_Port, K3_2_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_1_ON_Pin K1_1_OFF_Pin K2_1_ON_Pin K2_1_OFF_Pin
                           K3_1_OFF_Pin K3_1_ON_Pin */
  GPIO_InitStruct.Pin = K1_1_ON_Pin|K1_1_OFF_Pin|K2_1_ON_Pin|K2_1_OFF_Pin
                          |K3_1_OFF_Pin|K3_1_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_2_OFF_Pin K2_2_ON_Pin K2_2_OFF_Pin K3_2_OFF_Pin */
  GPIO_InitStruct.Pin = K1_2_OFF_Pin|K2_2_ON_Pin|K2_2_OFF_Pin|K3_2_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_1_STA_Pin K2_1_STA_Pin SW3_STA_Pin SW2_STA_Pin */
  GPIO_InitStruct.Pin = K1_1_STA_Pin|K2_1_STA_Pin|SW3_STA_Pin|SW2_STA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : K3_1_STA_Pin K1_2_STA_Pin K2_2_STA_Pin K3_2_STA_Pin */
  GPIO_InitStruct.Pin = K3_1_STA_Pin|K1_2_STA_Pin|K2_2_STA_Pin|K3_2_STA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : K1_2_ON_Pin ALARM_Pin */
  GPIO_InitStruct.Pin = K1_2_ON_Pin|ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_STA_Pin */
  GPIO_InitStruct.Pin = SW1_STA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SW1_STA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : K3_EN_Pin */
  GPIO_InitStruct.Pin = K3_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(K3_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_SEN_Pin */
  GPIO_InitStruct.Pin = FAN_SEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FAN_SEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : K3_2_ON_Pin */
  GPIO_InitStruct.Pin = K3_2_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(K3_2_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_CTRL_Pin */
  GPIO_InitStruct.Pin = DC_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DC_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : K2_EN_Pin K1_EN_Pin */
  GPIO_InitStruct.Pin = K2_EN_Pin|K1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
