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

// 继电器控制状态枚举
typedef enum {
    RELAY_CTRL_STATE_IDLE = 0,          // 空闲状态
    RELAY_CTRL_STATE_CHECKING,          // 检查状态
    RELAY_CTRL_STATE_EXECUTING,         // 执行中
    RELAY_CTRL_STATE_VERIFYING,         // 验证中
    RELAY_CTRL_STATE_COMPLETED,         // 完成
    RELAY_CTRL_STATE_ERROR              // 错误
} Relay_Control_State_TypeDef;

// 继电器控制动作类型
typedef enum {
    RELAY_CTRL_ACTION_OPEN = 0,         // 开启通道
    RELAY_CTRL_ACTION_CLOSE             // 关闭通道
} Relay_Control_Action_TypeDef;

// 异常标志枚举
typedef enum {
    ALARM_FLAG_NONE = 0,                // 无异常
    ALARM_FLAG_A,                       // 使能冲突异常
    ALARM_FLAG_B,                       // K1_1_STA工作异常
    ALARM_FLAG_C,                       // K2_1_STA工作异常
    ALARM_FLAG_D,                       // K3_1_STA工作异常
    ALARM_FLAG_E,                       // K1_2_STA工作异常
    ALARM_FLAG_F,                       // K2_2_STA工作异常
    ALARM_FLAG_G,                       // K3_2_STA工作异常
    ALARM_FLAG_H,                       // SW1_STA工作异常
    ALARM_FLAG_I,                       // SW2_STA工作异常
    ALARM_FLAG_J,                       // SW3_STA工作异常
    ALARM_FLAG_K,                       // NTC_1温度异常
    ALARM_FLAG_L,                       // NTC_2温度异常
    ALARM_FLAG_M,                       // NTC_3温度异常
    ALARM_FLAG_N                        // 自检异常
} Alarm_Flag_TypeDef;

// 继电器控制任务结构体
typedef struct {
    Channel_TypeDef channel;                    // 控制通道
    Relay_Control_Action_TypeDef action;       // 控制动作
    Relay_Control_State_TypeDef state;         // 当前状态
    uint32_t start_time;                       // 开始时间
    uint8_t check_count;                       // 检查次数
    Alarm_Flag_TypeDef alarm_flag;             // 异常标志
    uint8_t completed;                         // 完成标志
} Relay_Control_Task_TypeDef;

// 通道状态结构体
typedef struct {
    uint8_t enable_state;                      // 使能状态
    uint8_t relay1_state;                      // 继电器1状态
    uint8_t relay2_state;                      // 继电器2状态
    uint8_t switch_state;                      // 开关状态
    uint8_t is_active;                         // 通道是否激活
} Channel_Status_TypeDef;

// 系统状态结构体
typedef struct {
    Channel_Status_TypeDef channels[3];        // 三个通道状态
    uint32_t active_alarm_flags;               // 激活的异常标志位图
    uint8_t system_ready;                      // 系统就绪标志
    uint32_t last_update_time;                 // 最后更新时间
} System_Status_TypeDef;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

// 继电器控制初始化和配置
HAL_StatusTypeDef Relay_Control_Init(void);
HAL_StatusTypeDef Relay_Control_Reset(void);

// 继电器控制主要接口
HAL_StatusTypeDef Relay_Control_ProcessChannelEnable(Channel_TypeDef channel, uint8_t enable_state);
HAL_StatusTypeDef Relay_Control_ExecuteChannelOpen(Channel_TypeDef channel);
HAL_StatusTypeDef Relay_Control_ExecuteChannelClose(Channel_TypeDef channel);

// 状态检查和验证
uint8_t Relay_Control_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expected_state);
uint8_t Relay_Control_CheckOtherChannelsIdle(Channel_TypeDef active_channel);
uint8_t Relay_Control_CheckOtherChannelsStatus(Channel_TypeDef active_channel);
uint8_t Relay_Control_VerifyChannelStatus(Channel_TypeDef channel, Relay_Control_Action_TypeDef action);

// 异常处理
void Relay_Control_SetAlarmFlag(Alarm_Flag_TypeDef flag);
void Relay_Control_ClearAlarmFlag(Alarm_Flag_TypeDef flag);
uint8_t Relay_Control_IsAlarmActive(Alarm_Flag_TypeDef flag);
uint32_t Relay_Control_GetActiveAlarms(void);

// 系统状态管理
void Relay_Control_UpdateSystemStatus(void);
System_Status_TypeDef* Relay_Control_GetSystemStatus(void);
void Relay_Control_PrintSystemStatus(void);

// 任务管理
HAL_StatusTypeDef Relay_Control_CreateTask(Channel_TypeDef channel, Relay_Control_Action_TypeDef action);
void Relay_Control_ProcessTasks(void);
uint8_t Relay_Control_IsTaskActive(Channel_TypeDef channel);

// 自检功能
HAL_StatusTypeDef Relay_Control_SelfCheck(void);
uint8_t Relay_Control_CheckInitialState(void);

// 测试功能
void Relay_Control_Test(Channel_TypeDef channel);
void Relay_Control_TestAll(void);

// 调试功能
void Relay_Control_PrintChannelStatus(Channel_TypeDef channel);
void Relay_Control_PrintAlarmStatus(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RELAY_CONTROL_H */ 


