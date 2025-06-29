/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    relay_control.c
  * @brief   This file provides code for relay control functionality
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
#include "relay_control.h"
#include "usart.h"
#include "iwdg.h"
#include <string.h>

/* USER CODE BEGIN 0 */

// 全局变量定义
static System_Status_TypeDef system_status = {0};
static Relay_Control_Task_TypeDef active_tasks[3] = {
    {CHANNEL_1, RELAY_CTRL_ACTION_OPEN, RELAY_CTRL_STATE_IDLE, 0, 0, ALARM_FLAG_NONE, 0},
    {CHANNEL_2, RELAY_CTRL_ACTION_OPEN, RELAY_CTRL_STATE_IDLE, 0, 0, ALARM_FLAG_NONE, 0},
    {CHANNEL_3, RELAY_CTRL_ACTION_OPEN, RELAY_CTRL_STATE_IDLE, 0, 0, ALARM_FLAG_NONE, 0}
};
static uint32_t alarm_flags_bitmap = 0;

// 常量定义
#define STABLE_CHECK_INTERVAL_MS    50      // 稳定性检查间隔50ms
#define STABLE_CHECK_TIMES          3       // 连续检查3次
#define PULSE_DURATION_MS           500     // 脉冲持续时间500ms
#define VERIFY_DELAY_MS             500     // 验证延时500ms

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
 * @brief  继电器控制模块初始化
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_Init(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器控制模块初始化开始");
    
    // 清零系统状态
    memset(&system_status, 0, sizeof(system_status));
    memset(active_tasks, 0, sizeof(active_tasks));
    alarm_flags_bitmap = 0;
    
    // 初始化GPIO继电器
    GPIO_RelayInitialize();
    
    // 更新系统状态
    Relay_Control_UpdateSystemStatus();
    
    // 设置系统就绪标志
    system_status.system_ready = 1;
    
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器控制模块初始化完成");
    return HAL_OK;
}

/**
 * @brief  继电器控制模块复位
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_Reset(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器控制模块复位");
    
    // 停止所有任务
    memset(active_tasks, 0, sizeof(active_tasks));
    
    // 清除所有异常标志
    alarm_flags_bitmap = 0;
    
    // 重新初始化
    return Relay_Control_Init();
}

/**
 * @brief  处理通道使能信号（中断触发的主要入口）
 * @param  channel: 通道号
 * @param  enable_state: 使能状态（0=低电平触发开启，1=高电平触发关闭）
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_ProcessChannelEnable(Channel_TypeDef channel, uint8_t enable_state)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "处理通道%d使能信号，状态=%d", channel + 1, enable_state);
    
    // 检查系统是否就绪
    if (!system_status.system_ready) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "系统未就绪，拒绝处理通道使能");
        return HAL_ERROR;
    }
    
    // 检查是否已有任务在处理该通道
    if (Relay_Control_IsTaskActive(channel)) {
        Debug_Printf(DEBUG_LEVEL_WARN, "通道%d已有任务在处理", channel + 1);
        return HAL_BUSY;
    }
    
    // 检查通道使能信号稳定性（50ms间隔3次检测）
    if (!Relay_Control_CheckChannelEnableStable(channel, enable_state)) {
        Debug_Printf(DEBUG_LEVEL_WARN, "通道%d使能信号不稳定", channel + 1);
        return HAL_ERROR;
    }
    
    // 根据使能状态创建对应任务
    Relay_Control_Action_TypeDef action = (enable_state == 0) ? 
                                          RELAY_CTRL_ACTION_OPEN : 
                                          RELAY_CTRL_ACTION_CLOSE;
    
    return Relay_Control_CreateTask(channel, action);
}

/**
 * @brief  执行通道开启操作
 * @param  channel: 通道号
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_ExecuteChannelOpen(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "执行通道%d开启操作", channel + 1);
    
    // 开始判定1：检测其他通道是否为高电平
    if (!Relay_Control_CheckOtherChannelsIdle(channel)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "其他通道使能冲突");
        Relay_Control_SetAlarmFlag(ALARM_FLAG_A);
        return HAL_ERROR;
    }
    
    // 开始判定2：检测其他通道的继电器和开关状态是否为低电平
    if (!Relay_Control_CheckOtherChannelsStatus(channel)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "其他通道状态异常");
        return HAL_ERROR;
    }
    
    // 执行继电器吸合脉冲
    HAL_StatusTypeDef result = GPIO_RelayPulseControl(channel, RELAY_ACTION_ON);
    if (result != HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器脉冲控制失败", channel + 1);
        return result;
    }
    
    // 延时500ms后验证状态
    for(int i = 0; i < 5; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    // 验证通道状态
    if (!Relay_Control_VerifyChannelStatus(channel, RELAY_CTRL_ACTION_OPEN)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d开启验证失败", channel + 1);
        return HAL_ERROR;
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "通道%d开启成功", channel + 1);
    return HAL_OK;
}

/**
 * @brief  执行通道关闭操作
 * @param  channel: 通道号
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_ExecuteChannelClose(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "执行通道%d关闭操作", channel + 1);
    
    // 执行继电器断开脉冲
    HAL_StatusTypeDef result = GPIO_RelayPulseControl(channel, RELAY_ACTION_OFF);
    if (result != HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器脉冲控制失败", channel + 1);
        return result;
    }
    
    // 延时500ms后验证状态
    for(int i = 0; i < 5; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    // 验证通道状态
    if (!Relay_Control_VerifyChannelStatus(channel, RELAY_CTRL_ACTION_CLOSE)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d关闭验证失败", channel + 1);
        return HAL_ERROR;
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "通道%d关闭成功", channel + 1);
    return HAL_OK;
}

/**
 * @brief  检查通道使能信号稳定性
 * @param  channel: 通道号
 * @param  expected_state: 期望状态
 * @retval 1=稳定，0=不稳定
 */
uint8_t Relay_Control_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expected_state)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "检查通道%d使能信号稳定性", channel + 1);
    
    for (int i = 0; i < STABLE_CHECK_TIMES; i++) {
        uint8_t current_state = GPIO_ReadChannelEnable(channel);
        if (current_state != expected_state) {
            Debug_Printf(DEBUG_LEVEL_DEBUG, "通道%d第%d次检查失败：期望=%d，实际=%d", 
                        channel + 1, i + 1, expected_state, current_state);
            return 0;
        }
        
        if (i < STABLE_CHECK_TIMES - 1) {
            HAL_Delay(STABLE_CHECK_INTERVAL_MS);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "通道%d使能信号稳定性检查通过", channel + 1);
    return 1;
}

/**
 * @brief  检查其他通道是否处于空闲状态
 * @param  active_channel: 当前激活的通道
 * @retval 1=其他通道空闲，0=有冲突
 */
uint8_t Relay_Control_CheckOtherChannelsIdle(Channel_TypeDef active_channel)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "检查其他通道空闲状态");
    
    for (int i = 0; i < 3; i++) {
        if (i == active_channel) continue;
        
        uint8_t enable_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        if (enable_state == 0) {  // 低电平表示激活
            Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d处于激活状态，与通道%d冲突", 
                        i + 1, active_channel + 1);
            return 0;
        }
    }
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "其他通道空闲状态检查通过");
    return 1;
}

/**
 * @brief  检查其他通道的状态反馈
 * @param  active_channel: 当前激活的通道
 * @retval 1=状态正常，0=状态异常
 */
uint8_t Relay_Control_CheckOtherChannelsStatus(Channel_TypeDef active_channel)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "检查其他通道状态反馈");
    
    uint8_t error_detected = 0;
    
    for (int i = 0; i < 3; i++) {
        if (i == active_channel) continue;
        
        // 检查继电器状态
        uint8_t relay1_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2));
        uint8_t relay2_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2 + 1));
        uint8_t switch_state = GPIO_ReadSwitchStatus((Channel_TypeDef)i);
        
        // 继电器应该为低电平（断开状态）
        if (relay1_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器1状态异常", i + 1);
            Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_B + i * 2));
            error_detected = 1;
        }
        
        if (relay2_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器2状态异常", i + 1);
            Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_E + i));
            error_detected = 1;
        }
        
        // 开关应该为低电平（断开状态）
        if (switch_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d开关状态异常", i + 1);
            Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_H + i));
            error_detected = 1;
        }
    }
    
    if (!error_detected) {
        Debug_Printf(DEBUG_LEVEL_DEBUG, "其他通道状态检查通过");
    }
    
    return !error_detected;
}

/**
 * @brief  验证通道状态
 * @param  channel: 通道号
 * @param  action: 期望的动作结果
 * @retval 1=验证通过，0=验证失败
 */
uint8_t Relay_Control_VerifyChannelStatus(Channel_TypeDef channel, Relay_Control_Action_TypeDef action)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "验证通道%d状态", channel + 1);
    
    // 读取当前状态
    uint8_t relay1_state = GPIO_ReadRelayStatus((Relay_TypeDef)(channel * 2));
    uint8_t relay2_state = GPIO_ReadRelayStatus((Relay_TypeDef)(channel * 2 + 1));
    uint8_t switch_state = GPIO_ReadSwitchStatus(channel);
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "通道%d状态：K%d_1=%d, K%d_2=%d, SW%d=%d", 
                channel + 1, channel + 1, relay1_state, 
                channel + 1, relay2_state, channel + 1, switch_state);
    
    uint8_t expected_relay_state = (action == RELAY_CTRL_ACTION_OPEN) ? 1 : 0;
    uint8_t verification_passed = 1;
    
    // 验证继电器状态
    if (relay1_state != expected_relay_state) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器1状态验证失败", channel + 1);
        Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_B + channel * 2));
        verification_passed = 0;
    }
    
    if (relay2_state != expected_relay_state) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器2状态验证失败", channel + 1);
        Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_E + channel));
        verification_passed = 0;
    }
    
    // 验证开关状态（如果硬件连接的话）
    if (switch_state != expected_relay_state) {
        Debug_Printf(DEBUG_LEVEL_DEBUG, "通道%d开关状态与继电器状态不一致（硬件未连接属正常）", channel + 1);
        // 不设置为错误，因为SW状态硬件未连接
    }
    
    if (verification_passed) {
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d状态验证通过", channel + 1);
    }
    
    return verification_passed;
}

/**
 * @brief  设置异常标志
 * @param  flag: 异常标志
 */
void Relay_Control_SetAlarmFlag(Alarm_Flag_TypeDef flag)
{
    if (flag != ALARM_FLAG_NONE) {
        alarm_flags_bitmap |= (1 << flag);
        Debug_Printf(DEBUG_LEVEL_ERROR, "设置异常标志: %c", 'A' + flag - 1);
    }
}

/**
 * @brief  清除异常标志
 * @param  flag: 异常标志
 */
void Relay_Control_ClearAlarmFlag(Alarm_Flag_TypeDef flag)
{
    if (flag != ALARM_FLAG_NONE) {
        alarm_flags_bitmap &= ~(1 << flag);
        Debug_Printf(DEBUG_LEVEL_INFO, "清除异常标志: %c", 'A' + flag - 1);
    }
}

/**
 * @brief  检查异常标志是否激活
 * @param  flag: 异常标志
 * @retval 1=激活，0=未激活
 */
uint8_t Relay_Control_IsAlarmActive(Alarm_Flag_TypeDef flag)
{
    return (alarm_flags_bitmap & (1 << flag)) ? 1 : 0;
}

/**
 * @brief  获取所有激活的异常标志
 * @retval 异常标志位图
 */
uint32_t Relay_Control_GetActiveAlarms(void)
{
    return alarm_flags_bitmap;
}

/**
 * @brief  更新系统状态
 */
void Relay_Control_UpdateSystemStatus(void)
{
    // 更新三个通道的状态
    for (int i = 0; i < 3; i++) {
        system_status.channels[i].enable_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        system_status.channels[i].relay1_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2));
        system_status.channels[i].relay2_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2 + 1));
        system_status.channels[i].switch_state = GPIO_ReadSwitchStatus((Channel_TypeDef)i);
        system_status.channels[i].is_active = (system_status.channels[i].relay1_state && 
                                               system_status.channels[i].relay2_state);
    }
    
    // 更新异常标志
    system_status.active_alarm_flags = alarm_flags_bitmap;
    
    // 更新时间戳
    system_status.last_update_time = HAL_GetTick();
}

/**
 * @brief  获取系统状态指针
 * @retval 系统状态指针
 */
System_Status_TypeDef* Relay_Control_GetSystemStatus(void)
{
    Relay_Control_UpdateSystemStatus();
    return &system_status;
}

/**
 * @brief  打印系统状态
 */
void Relay_Control_PrintSystemStatus(void)
{
    Relay_Control_UpdateSystemStatus();
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 继电器控制系统状态 ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "系统就绪: %s", system_status.system_ready ? "是" : "否");
    
    for (int i = 0; i < 3; i++) {
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d: EN=%d, K%d_1=%d, K%d_2=%d, SW%d=%d, 激活=%s", 
                    i + 1, 
                    system_status.channels[i].enable_state,
                    i + 1, system_status.channels[i].relay1_state,
                    i + 1, system_status.channels[i].relay2_state,
                    i + 1, system_status.channels[i].switch_state,
                    system_status.channels[i].is_active ? "是" : "否");
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "异常标志: 0x%08lX", system_status.active_alarm_flags);
}

/**
 * @brief  创建继电器控制任务
 * @param  channel: 通道号
 * @param  action: 控制动作
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_CreateTask(Channel_TypeDef channel, Relay_Control_Action_TypeDef action)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "创建通道%d控制任务，动作=%s", 
                channel + 1, (action == RELAY_CTRL_ACTION_OPEN) ? "开启" : "关闭");
    
    // 检查通道范围
    if (channel >= 3) {
        return HAL_ERROR;
    }
    
    // 初始化任务
    active_tasks[channel].channel = channel;
    active_tasks[channel].action = action;
    active_tasks[channel].state = RELAY_CTRL_STATE_CHECKING;
    active_tasks[channel].start_time = HAL_GetTick();
    active_tasks[channel].check_count = 0;
    active_tasks[channel].alarm_flag = ALARM_FLAG_NONE;
    active_tasks[channel].completed = 0;
    
    return HAL_OK;
}

/**
 * @brief  处理所有激活的任务
 */
void Relay_Control_ProcessTasks(void)
{
    HAL_IWDG_Refresh(&hiwdg);  // 任务处理开始前喂狗
    
    for (int i = 0; i < 3; i++) {
        if (active_tasks[i].completed || active_tasks[i].state == RELAY_CTRL_STATE_IDLE) {
            continue;
        }
        
        HAL_IWDG_Refresh(&hiwdg);  // 每个任务处理前喂狗
        
        switch (active_tasks[i].state) {
            case RELAY_CTRL_STATE_CHECKING:
                // 检查阶段已在创建任务时完成，直接进入执行阶段
                active_tasks[i].state = RELAY_CTRL_STATE_EXECUTING;
                break;
                
            case RELAY_CTRL_STATE_EXECUTING:
                // 执行继电器控制
                HAL_IWDG_Refresh(&hiwdg);  // 执行前喂狗
                if (active_tasks[i].action == RELAY_CTRL_ACTION_OPEN) {
                    if (Relay_Control_ExecuteChannelOpen((Channel_TypeDef)i) == HAL_OK) {
                        active_tasks[i].state = RELAY_CTRL_STATE_COMPLETED;
                    } else {
                        active_tasks[i].state = RELAY_CTRL_STATE_ERROR;
                    }
                } else {
                    if (Relay_Control_ExecuteChannelClose((Channel_TypeDef)i) == HAL_OK) {
                        active_tasks[i].state = RELAY_CTRL_STATE_COMPLETED;
                    } else {
                        active_tasks[i].state = RELAY_CTRL_STATE_ERROR;
                    }
                }
                HAL_IWDG_Refresh(&hiwdg);  // 执行后喂狗
                break;
                
            case RELAY_CTRL_STATE_COMPLETED:
                // 任务完成
                active_tasks[i].completed = 1;
                Debug_Printf(DEBUG_LEVEL_INFO, "通道%d控制任务完成", i + 1);
                break;
                
            case RELAY_CTRL_STATE_ERROR:
                // 任务错误
                active_tasks[i].completed = 1;
                Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d控制任务失败", i + 1);
                break;
                
            default:
                break;
        }
    }
}

/**
 * @brief  检查指定通道是否有激活的任务
 * @param  channel: 通道号
 * @retval 1=有激活任务，0=无激活任务
 */
uint8_t Relay_Control_IsTaskActive(Channel_TypeDef channel)
{
    if (channel >= 3) return 0;
    
    return (!active_tasks[channel].completed && 
            active_tasks[channel].state != RELAY_CTRL_STATE_IDLE) ? 1 : 0;
}

/**
 * @brief  继电器控制系统自检
 * @retval HAL状态
 */
HAL_StatusTypeDef Relay_Control_SelfCheck(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 继电器控制系统自检开始 ===");
    
    uint8_t self_check_passed = 1;
    
    // 检查初始状态
    if (!Relay_Control_CheckInitialState()) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "初始状态检查失败");
        self_check_passed = 0;
    }
    
    // 更新系统状态
    Relay_Control_UpdateSystemStatus();
    
    if (self_check_passed) {
        Debug_Printf(DEBUG_LEVEL_INFO, "继电器控制系统自检通过");
        return HAL_OK;
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "继电器控制系统自检失败");
        Relay_Control_SetAlarmFlag(ALARM_FLAG_N);
        return HAL_ERROR;
    }
}

/**
 * @brief  检查系统初始状态
 * @retval 1=正常，0=异常
 */
uint8_t Relay_Control_CheckInitialState(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "检查系统初始状态");
    
    uint8_t check_passed = 1;
    
    // 检查1：三路输入信号K1_EN、K2_EN、K3_EN均为高电平
    for (int i = 0; i < 3; i++) {
        uint8_t enable_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        if (enable_state != 1) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d使能信号初始状态异常：期望=1，实际=%d", 
                        i + 1, enable_state);
            check_passed = 0;
        }
    }
    
    // 检查2：所有继电器和开关状态均为低电平
    for (int i = 0; i < 6; i++) {
        uint8_t relay_state = GPIO_ReadRelayStatus((Relay_TypeDef)i);
        if (relay_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "继电器%d初始状态异常：期望=0，实际=%d", 
                        i + 1, relay_state);
            check_passed = 0;
        }
    }
    
    for (int i = 0; i < 3; i++) {
        uint8_t switch_state = GPIO_ReadSwitchStatus((Channel_TypeDef)i);
        if (switch_state != 0) {
            Debug_Printf(DEBUG_LEVEL_DEBUG, "开关%d初始状态：期望=0，实际=%d（硬件未连接属正常）", 
                        i + 1, switch_state);
            // 不设置为检查失败，因为SW状态硬件未连接
        }
    }
    
    if (check_passed) {
        Debug_Printf(DEBUG_LEVEL_INFO, "系统初始状态检查通过");
    }
    
    return check_passed;
}

/**
 * @brief  继电器控制测试
 * @param  channel: 测试通道
 */
void Relay_Control_Test(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 通道%d继电器控制测试 ===", channel + 1);
    
    // 测试开启
    Debug_Printf(DEBUG_LEVEL_INFO, "测试通道%d开启", channel + 1);
    if (Relay_Control_ExecuteChannelOpen(channel) == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d开启测试通过", channel + 1);
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d开启测试失败", channel + 1);
    }
    
    HAL_Delay(2000);  // 等待2秒
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试关闭
    Debug_Printf(DEBUG_LEVEL_INFO, "测试通道%d关闭", channel + 1);
    if (Relay_Control_ExecuteChannelClose(channel) == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d关闭测试通过", channel + 1);
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d关闭测试失败", channel + 1);
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 通道%d测试完成 ===", channel + 1);
}

/**
 * @brief  所有通道继电器控制测试
 */
void Relay_Control_TestAll(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 全部通道继电器控制测试开始 ===");
    
    for (int i = 0; i < 3; i++) {
        Relay_Control_Test((Channel_TypeDef)i);
        HAL_Delay(1000);  // 通道间间隔1秒
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 全部通道继电器控制测试完成 ===");
}

/**
 * @brief  打印通道状态
 * @param  channel: 通道号
 */
void Relay_Control_PrintChannelStatus(Channel_TypeDef channel)
{
    if (channel >= 3) return;
    
    Relay_Control_UpdateSystemStatus();
    
    Debug_Printf(DEBUG_LEVEL_INFO, "通道%d状态详情:", channel + 1);
    Debug_Printf(DEBUG_LEVEL_INFO, "  使能状态: %d", system_status.channels[channel].enable_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  继电器1状态: %d", system_status.channels[channel].relay1_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  继电器2状态: %d", system_status.channels[channel].relay2_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  开关状态: %d", system_status.channels[channel].switch_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  通道激活: %s", system_status.channels[channel].is_active ? "是" : "否");
}

/**
 * @brief  打印异常状态
 */
void Relay_Control_PrintAlarmStatus(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "异常状态详情:");
    Debug_Printf(DEBUG_LEVEL_INFO, "  异常标志位图: 0x%08lX", alarm_flags_bitmap);
    
    if (alarm_flags_bitmap == 0) {
        Debug_Printf(DEBUG_LEVEL_INFO, "  当前无异常");
        return;
    }
    
    for (int i = ALARM_FLAG_A; i <= ALARM_FLAG_N; i++) {
        if (Relay_Control_IsAlarmActive((Alarm_Flag_TypeDef)i)) {
            const char* alarm_names[] = {
                "", "使能冲突", "K1_1_STA异常", "K2_1_STA异常", "K3_1_STA异常",
                "K1_2_STA异常", "K2_2_STA异常", "K3_2_STA异常",
                "SW1_STA异常", "SW2_STA异常", "SW3_STA异常",
                "NTC_1温度异常", "NTC_2温度异常", "NTC_3温度异常", "自检异常"
            };
            Debug_Printf(DEBUG_LEVEL_INFO, "  异常%c: %s", 'A' + i - 1, alarm_names[i]);
        }
    }
}

/* USER CODE END 1 */ 


