/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    relay_control.h
  * @brief   This file contains all the function prototypes for
  *          the relay_control.c file
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

#ifndef __RELAY_CONTROL_H
#define __RELAY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

// �̵�������״̬ö��
typedef enum {
    RELAY_CTRL_STATE_IDLE = 0,          // ����״̬
    RELAY_CTRL_STATE_CHECKING,          // ���״̬
    RELAY_CTRL_STATE_EXECUTING,         // ִ����
    RELAY_CTRL_STATE_VERIFYING,         // ��֤��
    RELAY_CTRL_STATE_COMPLETED,         // ���
    RELAY_CTRL_STATE_ERROR              // ����
} Relay_Control_State_TypeDef;

// �̵������ƶ�������
typedef enum {
    RELAY_CTRL_ACTION_OPEN = 0,         // ����ͨ��
    RELAY_CTRL_ACTION_CLOSE             // �ر�ͨ��
} Relay_Control_Action_TypeDef;

// �쳣��־ö��
typedef enum {
    ALARM_FLAG_NONE = 0,                // ���쳣
    ALARM_FLAG_A,                       // ʹ�ܳ�ͻ�쳣
    ALARM_FLAG_B,                       // K1_1_STA�����쳣
    ALARM_FLAG_C,                       // K2_1_STA�����쳣
    ALARM_FLAG_D,                       // K3_1_STA�����쳣
    ALARM_FLAG_E,                       // K1_2_STA�����쳣
    ALARM_FLAG_F,                       // K2_2_STA�����쳣
    ALARM_FLAG_G,                       // K3_2_STA�����쳣
    ALARM_FLAG_H,                       // SW1_STA�����쳣
    ALARM_FLAG_I,                       // SW2_STA�����쳣
    ALARM_FLAG_J,                       // SW3_STA�����쳣
    ALARM_FLAG_K,                       // NTC_1�¶��쳣
    ALARM_FLAG_L,                       // NTC_2�¶��쳣
    ALARM_FLAG_M,                       // NTC_3�¶��쳣
    ALARM_FLAG_N                        // �Լ��쳣
} Alarm_Flag_TypeDef;

// �̵�����������ṹ��
typedef struct {
    Channel_TypeDef channel;                    // ����ͨ��
    Relay_Control_Action_TypeDef action;       // ���ƶ���
    Relay_Control_State_TypeDef state;         // ��ǰ״̬
    uint32_t start_time;                       // ��ʼʱ��
    uint8_t check_count;                       // ������
    Alarm_Flag_TypeDef alarm_flag;             // �쳣��־
    uint8_t completed;                         // ��ɱ�־
} Relay_Control_Task_TypeDef;

// ͨ��״̬�ṹ��
typedef struct {
    uint8_t enable_state;                      // ʹ��״̬
    uint8_t relay1_state;                      // �̵���1״̬
    uint8_t relay2_state;                      // �̵���2״̬
    uint8_t switch_state;                      // ����״̬
    uint8_t is_active;                         // ͨ���Ƿ񼤻�
} Channel_Status_TypeDef;

// ϵͳ״̬�ṹ��
typedef struct {
    Channel_Status_TypeDef channels[3];        // ����ͨ��״̬
    uint32_t active_alarm_flags;               // ������쳣��־λͼ
    uint8_t system_ready;                      // ϵͳ������־
    uint32_t last_update_time;                 // ������ʱ��
} System_Status_TypeDef;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

// �̵������Ƴ�ʼ��������
HAL_StatusTypeDef Relay_Control_Init(void);
HAL_StatusTypeDef Relay_Control_Reset(void);

// �̵���������Ҫ�ӿ�
HAL_StatusTypeDef Relay_Control_ProcessChannelEnable(Channel_TypeDef channel, uint8_t enable_state);
HAL_StatusTypeDef Relay_Control_ExecuteChannelOpen(Channel_TypeDef channel);
HAL_StatusTypeDef Relay_Control_ExecuteChannelClose(Channel_TypeDef channel);

// ״̬������֤
uint8_t Relay_Control_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expected_state);
uint8_t Relay_Control_CheckOtherChannelsIdle(Channel_TypeDef active_channel);
uint8_t Relay_Control_CheckOtherChannelsStatus(Channel_TypeDef active_channel);
uint8_t Relay_Control_VerifyChannelStatus(Channel_TypeDef channel, Relay_Control_Action_TypeDef action);

// �쳣����
void Relay_Control_SetAlarmFlag(Alarm_Flag_TypeDef flag);
void Relay_Control_ClearAlarmFlag(Alarm_Flag_TypeDef flag);
uint8_t Relay_Control_IsAlarmActive(Alarm_Flag_TypeDef flag);
uint32_t Relay_Control_GetActiveAlarms(void);

// ϵͳ״̬����
void Relay_Control_UpdateSystemStatus(void);
System_Status_TypeDef* Relay_Control_GetSystemStatus(void);
void Relay_Control_PrintSystemStatus(void);

// �������
HAL_StatusTypeDef Relay_Control_CreateTask(Channel_TypeDef channel, Relay_Control_Action_TypeDef action);
void Relay_Control_ProcessTasks(void);
uint8_t Relay_Control_IsTaskActive(Channel_TypeDef channel);

// �Լ칦��
HAL_StatusTypeDef Relay_Control_SelfCheck(void);
uint8_t Relay_Control_CheckInitialState(void);

// ���Թ���
void Relay_Control_Test(Channel_TypeDef channel);
void Relay_Control_TestAll(void);

// ���Թ���
void Relay_Control_PrintChannelStatus(Channel_TypeDef channel);
void Relay_Control_PrintAlarmStatus(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_CONTROL_H */ 


