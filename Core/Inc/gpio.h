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
// ͨ��ʹ��״̬ö��
typedef enum {
    CHANNEL_1 = 0,
    CHANNEL_2,
    CHANNEL_3
} Channel_TypeDef;

// �̵���״̬ö��
typedef enum {
    RELAY_OFF = 0,
    RELAY_ON
} Relay_State_TypeDef;

// �̵�������ö��
typedef enum {
    RELAY_1_1 = 0,
    RELAY_1_2,
    RELAY_2_1,
    RELAY_2_2,
    RELAY_3_1,
    RELAY_3_2
} Relay_TypeDef;

// �ж�����ö��
typedef enum {
    INTERRUPT_TYPE_CHANNEL_ENABLE = 0,
    INTERRUPT_TYPE_DC_CTRL,
    INTERRUPT_TYPE_KEY
} Interrupt_Type_TypeDef;

// �̵�����������ö��
typedef enum {
    RELAY_ACTION_ON = 0,
    RELAY_ACTION_OFF
} Relay_Action_TypeDef;

// �ж�״̬�ṹ��
typedef struct {
    uint8_t channel_enable_flags;  // λ0-2�ֱ��ӦK1_EN��K2_EN��K3_EN
    uint8_t dc_ctrl_flag;
    uint8_t key_flags;             // λ0-2�ֱ��ӦKEY1��KEY2��KEY3
    uint32_t last_interrupt_time[8]; // ��¼�����жϵ���󴥷�ʱ��
} Interrupt_Status_TypeDef;

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
// GPIO״̬��ȡ����
uint8_t GPIO_ReadChannelEnable(Channel_TypeDef channel);
uint8_t GPIO_ReadRelayStatus(Relay_TypeDef relay);
uint8_t GPIO_ReadSwitchStatus(Channel_TypeDef channel);
uint8_t GPIO_ReadDCStatus(void);
uint8_t GPIO_ReadFanSensor(void);

// GPIO״̬���ú���
void GPIO_SetRelayOn(Relay_TypeDef relay);
void GPIO_SetRelayOff(Relay_TypeDef relay);
void GPIO_SetAlarm(uint8_t state);
void GPIO_SetBeep(uint8_t state);
void GPIO_SetRS485DE(uint8_t state);

// ����״̬��⺯��
uint8_t GPIO_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expectedState, uint8_t checkTimes, uint32_t interval);
uint8_t GPIO_CheckRelayStatusStable(Relay_TypeDef relay, uint8_t expectedState, uint8_t checkTimes, uint32_t interval);
uint8_t GPIO_CheckSwitchStatusStable(Channel_TypeDef channel, uint8_t expectedState, uint8_t checkTimes, uint32_t interval);

// �̵������ƺ���
HAL_StatusTypeDef GPIO_RelayPulseControl(Channel_TypeDef channel, Relay_Action_TypeDef action);
HAL_StatusTypeDef GPIO_RelayChannelControl(Channel_TypeDef channel, Relay_Action_TypeDef action);
void GPIO_RelayInitialize(void);
uint8_t GPIO_RelayStatusCheck(Channel_TypeDef channel);

// �жϴ�����
void GPIO_InterruptInit(void);
void GPIO_InterruptHandler(uint16_t GPIO_Pin);
uint8_t GPIO_GetInterruptStatus(Interrupt_Type_TypeDef type);
void GPIO_ClearInterruptStatus(Interrupt_Type_TypeDef type);

// GPIO״̬������
void GPIO_StateMonitorInit(void);
void GPIO_StateMonitorUpdate(void);
uint8_t GPIO_StateAnomalyCheck(void);
void GPIO_StatePrint(void);

// ���Ժ���֤����
void GPIO_RelayTest(Channel_TypeDef channel);
void GPIO_InterruptTest(void);
void GPIO_StateTest(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

