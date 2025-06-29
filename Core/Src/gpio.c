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
#include "iwdg.h"    // 添加iwdg.h支持看门狗喂狗操作
#include <string.h>  // 添加string.h支持memset函数
#include "usart.h"   // 添加usart.h支持调试函数

// 中断状态全局变量
static Interrupt_Status_TypeDef interrupt_status = {0};
static uint8_t gpio_state_monitor_enabled = 0;

// GPIO状态历史记录
typedef struct {
    uint8_t channel_enable[3];     // K1_EN, K2_EN, K3_EN
    uint8_t relay_status[6];       // K1_1_STA, K1_2_STA, K2_1_STA, K2_2_STA, K3_1_STA, K3_2_STA
    uint8_t switch_status[3];      // SW1_STA, SW2_STA, SW3_STA
    uint8_t dc_ctrl;
    uint8_t fan_sensor;
    uint32_t timestamp;
} GPIO_State_History_TypeDef;

static GPIO_State_History_TypeDef gpio_state_current = {0};
static GPIO_State_History_TypeDef gpio_state_previous = {0};

// GPIO状态读取函数
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

// GPIO状态设置函数
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

// 引脚状态检测函数
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

// 继电器控制函数
HAL_StatusTypeDef GPIO_RelayPulseControl(Channel_TypeDef channel, Relay_Action_TypeDef action)
{
    // 根据通道和动作选择对应的继电器
    Relay_TypeDef relay1, relay2;
    
    switch(channel)
    {
        case CHANNEL_1:
            relay1 = RELAY_1_1;
            relay2 = RELAY_1_2;
            break;
        case CHANNEL_2:
            relay1 = RELAY_2_1;
            relay2 = RELAY_2_2;
            break;
        case CHANNEL_3:
            relay1 = RELAY_3_1;
            relay2 = RELAY_3_2;
            break;
        default:
            return HAL_ERROR;
    }
    
    // 同时控制两个继电器
    if(action == RELAY_ACTION_ON)
    {
        GPIO_SetRelayOn(relay1);
        GPIO_SetRelayOn(relay2);
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d继电器吸合脉冲开始", channel + 1);
    }
    else
    {
        GPIO_SetRelayOff(relay1);
        GPIO_SetRelayOff(relay2);
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d继电器断开脉冲开始", channel + 1);
    }
    
    // 保持500ms低电平脉冲（期间需要喂狗）
    for(int delay_count = 0; delay_count < 5; delay_count++)
    {
        HAL_Delay(100);  // 分段延时100ms
        HAL_IWDG_Refresh(&hiwdg);  // 每100ms喂一次狗
    }
    
    // 恢复高电平 - 根据通道正确恢复对应的引脚
    if(action == RELAY_ACTION_ON)
    {
        switch(channel)
        {
            case CHANNEL_1:
                HAL_GPIO_WritePin(K1_1_ON_GPIO_Port, K1_1_ON_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(K1_2_ON_GPIO_Port, K1_2_ON_Pin, GPIO_PIN_SET);
                break;
            case CHANNEL_2:
                HAL_GPIO_WritePin(K2_1_ON_GPIO_Port, K2_1_ON_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(K2_2_ON_GPIO_Port, K2_2_ON_Pin, GPIO_PIN_SET);
                break;
            case CHANNEL_3:
                HAL_GPIO_WritePin(K3_1_ON_GPIO_Port, K3_1_ON_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(K3_2_ON_GPIO_Port, K3_2_ON_Pin, GPIO_PIN_SET);
                break;
            default:
                break;
        }
    }
    else
    {
        switch(channel)
        {
            case CHANNEL_1:
                HAL_GPIO_WritePin(K1_1_OFF_GPIO_Port, K1_1_OFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(K1_2_OFF_GPIO_Port, K1_2_OFF_Pin, GPIO_PIN_SET);
                break;
            case CHANNEL_2:
                HAL_GPIO_WritePin(K2_1_OFF_GPIO_Port, K2_1_OFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(K2_2_OFF_GPIO_Port, K2_2_OFF_Pin, GPIO_PIN_SET);
                break;
            case CHANNEL_3:
                HAL_GPIO_WritePin(K3_1_OFF_GPIO_Port, K3_1_OFF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(K3_2_OFF_GPIO_Port, K3_2_OFF_Pin, GPIO_PIN_SET);
                break;
            default:
                break;
        }
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "通道%d继电器脉冲控制完成", channel + 1);
    
    return HAL_OK;
}

HAL_StatusTypeDef GPIO_RelayChannelControl(Channel_TypeDef channel, Relay_Action_TypeDef action)
{
    // 执行脉冲控制
    HAL_StatusTypeDef result = GPIO_RelayPulseControl(channel, action);
    
    if(result == HAL_OK)
    {
        // 延时500ms后检查状态（期间喂狗）
        for(int delay_count = 0; delay_count < 5; delay_count++)
        {
            HAL_Delay(100);  // 分段延时100ms
            HAL_IWDG_Refresh(&hiwdg);  // 每100ms喂一次狗
        }
        
        // 检查继电器状态是否符合预期
        uint8_t status_check = GPIO_RelayStatusCheck(channel);
        
        if(action == RELAY_ACTION_ON && status_check)
        {
            Debug_Printf(DEBUG_LEVEL_INFO, "通道%d继电器吸合成功", channel + 1);
        }
        else if(action == RELAY_ACTION_OFF && !status_check)
        {
            Debug_Printf(DEBUG_LEVEL_INFO, "通道%d继电器断开成功", channel + 1);
        }
        else
        {
            Debug_Printf(DEBUG_LEVEL_ERROR, "通道%d继电器状态异常", channel + 1);
            return HAL_ERROR;
        }
    }
    
    return result;
}

void GPIO_RelayInitialize(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器初始化开始");
    
    // 确保所有继电器控制引脚为高电平（非激活状态）
    HAL_GPIO_WritePin(K1_1_ON_GPIO_Port, K1_1_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K1_1_OFF_GPIO_Port, K1_1_OFF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K1_2_ON_GPIO_Port, K1_2_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K1_2_OFF_GPIO_Port, K1_2_OFF_Pin, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(K2_1_ON_GPIO_Port, K2_1_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K2_1_OFF_GPIO_Port, K2_1_OFF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K2_2_ON_GPIO_Port, K2_2_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K2_2_OFF_GPIO_Port, K2_2_OFF_Pin, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(K3_1_ON_GPIO_Port, K3_1_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K3_1_OFF_GPIO_Port, K3_1_OFF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K3_2_ON_GPIO_Port, K3_2_ON_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(K3_2_OFF_GPIO_Port, K3_2_OFF_Pin, GPIO_PIN_SET);
    
    // 读取当前继电器状态
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器初始状态:");
    for(int i = 0; i < 3; i++)
    {
        uint8_t status = GPIO_RelayStatusCheck((Channel_TypeDef)i);
        Debug_Printf(DEBUG_LEVEL_INFO, "  通道%d: %s", i + 1, status ? "吸合" : "断开");
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "继电器初始化完成");
}

uint8_t GPIO_RelayStatusCheck(Channel_TypeDef channel)
{
    uint8_t relay1_status, relay2_status, switch_status;
    
    switch(channel)
    {
        case CHANNEL_1:
            relay1_status = GPIO_ReadRelayStatus(RELAY_1_1);
            relay2_status = GPIO_ReadRelayStatus(RELAY_1_2);
            switch_status = GPIO_ReadSwitchStatus(CHANNEL_1);
            break;
        case CHANNEL_2:
            relay1_status = GPIO_ReadRelayStatus(RELAY_2_1);
            relay2_status = GPIO_ReadRelayStatus(RELAY_2_2);
            switch_status = GPIO_ReadSwitchStatus(CHANNEL_2);
            break;
        case CHANNEL_3:
            relay1_status = GPIO_ReadRelayStatus(RELAY_3_1);
            relay2_status = GPIO_ReadRelayStatus(RELAY_3_2);
            switch_status = GPIO_ReadSwitchStatus(CHANNEL_3);
            break;
        default:
            return 0;
    }
    
    // 输出详细状态信息用于调试
    Debug_Printf(DEBUG_LEVEL_DEBUG, "通道%d状态: K%d_1=%d, K%d_2=%d, SW%d=%d", 
                channel + 1, channel + 1, relay1_status, channel + 1, relay2_status, channel + 1, switch_status);
    
    // 只要双继电器状态一致就认为正常（不强制要求与开关状态一致）
    return (relay1_status && relay2_status);
}

// 中断处理函数
void GPIO_InterruptInit(void)
{
    // 清零中断状态
    memset(&interrupt_status, 0, sizeof(interrupt_status));
    
    Debug_Printf(DEBUG_LEVEL_INFO, "GPIO中断系统初始化完成");
}

void GPIO_InterruptHandler(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick();
    
    // 处理通道使能中断
    if(GPIO_Pin == K1_EN_Pin)
    {
        // 防抖处理：检查距离上次中断是否超过50ms
        if(current_time - interrupt_status.last_interrupt_time[0] > 50)
        {
            interrupt_status.channel_enable_flags |= 0x01;  // 设置K1_EN标志
            interrupt_status.last_interrupt_time[0] = current_time;
            
            uint8_t pin_state = GPIO_ReadChannelEnable(CHANNEL_1);
            Debug_Printf(DEBUG_LEVEL_INFO, "K1_EN中断触发, 当前状态: %d", pin_state);
            
            // 检查信号稳定性
            if(GPIO_CheckChannelEnableStable(CHANNEL_1, pin_state, 3, 50))
            {
                if(pin_state == 0)  // 下降沿
                {
                    Debug_Printf(DEBUG_LEVEL_INFO, "K1_EN下降沿检测，准备开启通道1");
                    // 这里后续会添加通道开启逻辑
                }
                else  // 上升沿
                {
                    Debug_Printf(DEBUG_LEVEL_INFO, "K1_EN上升沿检测，准备关闭通道1");
                    // 这里后续会添加通道关闭逻辑
                }
            }
        }
    }
    else if(GPIO_Pin == K2_EN_Pin)
    {
        if(current_time - interrupt_status.last_interrupt_time[1] > 50)
        {
            interrupt_status.channel_enable_flags |= 0x02;  // 设置K2_EN标志
            interrupt_status.last_interrupt_time[1] = current_time;
            
            uint8_t pin_state = GPIO_ReadChannelEnable(CHANNEL_2);
            Debug_Printf(DEBUG_LEVEL_INFO, "K2_EN中断触发, 当前状态: %d", pin_state);
            
            if(GPIO_CheckChannelEnableStable(CHANNEL_2, pin_state, 3, 50))
            {
                if(pin_state == 0)
                {
                    Debug_Printf(DEBUG_LEVEL_INFO, "K2_EN下降沿检测，准备开启通道2");
                }
                else
                {
                    Debug_Printf(DEBUG_LEVEL_INFO, "K2_EN上升沿检测，准备关闭通道2");
                }
            }
        }
    }
    else if(GPIO_Pin == K3_EN_Pin)
    {
        if(current_time - interrupt_status.last_interrupt_time[2] > 50)
        {
            interrupt_status.channel_enable_flags |= 0x04;  // 设置K3_EN标志
            interrupt_status.last_interrupt_time[2] = current_time;
            
            uint8_t pin_state = GPIO_ReadChannelEnable(CHANNEL_3);
            Debug_Printf(DEBUG_LEVEL_INFO, "K3_EN中断触发, 当前状态: %d", pin_state);
            
            if(GPIO_CheckChannelEnableStable(CHANNEL_3, pin_state, 3, 50))
            {
                if(pin_state == 0)
                {
                    Debug_Printf(DEBUG_LEVEL_INFO, "K3_EN下降沿检测，准备开启通道3");
                }
                else
                {
                    Debug_Printf(DEBUG_LEVEL_INFO, "K3_EN上升沿检测，准备关闭通道3");
                }
            }
        }
    }
    // 处理DC_CTRL中断
    else if(GPIO_Pin == DC_CTRL_Pin)
    {
        if(current_time - interrupt_status.last_interrupt_time[3] > 50)
        {
            interrupt_status.dc_ctrl_flag = 1;
            interrupt_status.last_interrupt_time[3] = current_time;
            
            uint8_t pin_state = GPIO_ReadDCStatus();
            Debug_Printf(DEBUG_LEVEL_WARN, "DC_CTRL中断触发, 电源状态: %d", pin_state);
        }
    }
    // 处理按键中断
    else if(GPIO_Pin == KEY1_Pin)
    {
        if(current_time - interrupt_status.last_interrupt_time[4] > 200)  // 按键防抖200ms
        {
            interrupt_status.key_flags |= 0x01;
            interrupt_status.last_interrupt_time[4] = current_time;
            Debug_Printf(DEBUG_LEVEL_DEBUG, "KEY1按键中断触发");
        }
    }
    else if(GPIO_Pin == KEY2_Pin)
    {
        if(current_time - interrupt_status.last_interrupt_time[5] > 200)
        {
            interrupt_status.key_flags |= 0x02;
            interrupt_status.last_interrupt_time[5] = current_time;
            Debug_Printf(DEBUG_LEVEL_DEBUG, "KEY2按键中断触发");
        }
    }
}

uint8_t GPIO_GetInterruptStatus(Interrupt_Type_TypeDef type)
{
    switch(type)
    {
        case INTERRUPT_TYPE_CHANNEL_ENABLE:
            return interrupt_status.channel_enable_flags;
        case INTERRUPT_TYPE_DC_CTRL:
            return interrupt_status.dc_ctrl_flag;
        case INTERRUPT_TYPE_KEY:
            return interrupt_status.key_flags;
        default:
            return 0;
    }
}

void GPIO_ClearInterruptStatus(Interrupt_Type_TypeDef type)
{
    switch(type)
    {
        case INTERRUPT_TYPE_CHANNEL_ENABLE:
            interrupt_status.channel_enable_flags = 0;
            break;
        case INTERRUPT_TYPE_DC_CTRL:
            interrupt_status.dc_ctrl_flag = 0;
            break;
        case INTERRUPT_TYPE_KEY:
            interrupt_status.key_flags = 0;
            break;
        default:
            break;
    }
}

// GPIO状态管理函数
void GPIO_StateMonitorInit(void)
{
    gpio_state_monitor_enabled = 1;
    
    // 初始化状态记录
    memset(&gpio_state_current, 0, sizeof(gpio_state_current));
    memset(&gpio_state_previous, 0, sizeof(gpio_state_previous));
    
    Debug_Printf(DEBUG_LEVEL_INFO, "GPIO状态监控系统初始化完成");
}

void GPIO_StateMonitorUpdate(void)
{
    if(!gpio_state_monitor_enabled) return;
    
    // 保存上一次状态
    gpio_state_previous = gpio_state_current;
    
    // 更新当前状态
    gpio_state_current.channel_enable[0] = GPIO_ReadChannelEnable(CHANNEL_1);
    gpio_state_current.channel_enable[1] = GPIO_ReadChannelEnable(CHANNEL_2);
    gpio_state_current.channel_enable[2] = GPIO_ReadChannelEnable(CHANNEL_3);
    
    gpio_state_current.relay_status[0] = GPIO_ReadRelayStatus(RELAY_1_1);
    gpio_state_current.relay_status[1] = GPIO_ReadRelayStatus(RELAY_1_2);
    gpio_state_current.relay_status[2] = GPIO_ReadRelayStatus(RELAY_2_1);
    gpio_state_current.relay_status[3] = GPIO_ReadRelayStatus(RELAY_2_2);
    gpio_state_current.relay_status[4] = GPIO_ReadRelayStatus(RELAY_3_1);
    gpio_state_current.relay_status[5] = GPIO_ReadRelayStatus(RELAY_3_2);
    
    gpio_state_current.switch_status[0] = GPIO_ReadSwitchStatus(CHANNEL_1);
    gpio_state_current.switch_status[1] = GPIO_ReadSwitchStatus(CHANNEL_2);
    gpio_state_current.switch_status[2] = GPIO_ReadSwitchStatus(CHANNEL_3);
    
    gpio_state_current.dc_ctrl = GPIO_ReadDCStatus();
    gpio_state_current.fan_sensor = GPIO_ReadFanSensor();
    gpio_state_current.timestamp = HAL_GetTick();
}

uint8_t GPIO_StateAnomalyCheck(void)
{
    uint8_t anomaly_detected = 0;
    
    // 检查通道使能冲突（不能有多个通道同时为低电平）
    uint8_t low_count = 0;
    for(int i = 0; i < 3; i++)
    {
        if(gpio_state_current.channel_enable[i] == 0)
        {
            low_count++;
        }
    }
    
    if(low_count > 1)
    {
        Debug_Printf(DEBUG_LEVEL_ERROR, "检测到通道使能冲突：%d个通道同时激活", low_count);
        anomaly_detected = 1;
    }
    
    // 检查继电器状态异常
    for(int i = 0; i < 3; i++)
    {
        uint8_t relay1 = gpio_state_current.relay_status[i*2];
        uint8_t relay2 = gpio_state_current.relay_status[i*2+1];
        uint8_t switch_state = gpio_state_current.switch_status[i];
        
        // 如果继电器状态不一致
        if(relay1 != relay2)
        {
            Debug_Printf(DEBUG_LEVEL_WARN, "通道%d继电器状态不一致：K%d_1=%d, K%d_2=%d", 
                        i+1, i+1, relay1, i+1, relay2);
            anomaly_detected = 1;
        }
        
        // 继电器状态与开关状态不匹配是正常现象，仅在DEBUG级别输出
        if((relay1 && relay2) != switch_state)
        {
            Debug_Printf(DEBUG_LEVEL_DEBUG, "通道%d继电器与开关状态：继电器=%d, 开关=%d", 
                        i+1, (relay1 && relay2), switch_state);
        }
    }
    
    return anomaly_detected;
}

void GPIO_StatePrint(void)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "=== GPIO状态详情 ===");
    Debug_Printf(DEBUG_LEVEL_DEBUG, "通道使能: K1=%d, K2=%d, K3=%d",
                gpio_state_current.channel_enable[0],
                gpio_state_current.channel_enable[1],
                gpio_state_current.channel_enable[2]);
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "继电器状态: K1_1=%d, K1_2=%d, K2_1=%d, K2_2=%d, K3_1=%d, K3_2=%d",
                gpio_state_current.relay_status[0],
                gpio_state_current.relay_status[1],
                gpio_state_current.relay_status[2],
                gpio_state_current.relay_status[3],
                gpio_state_current.relay_status[4],
                gpio_state_current.relay_status[5]);
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "开关状态: SW1=%d, SW2=%d, SW3=%d",
                gpio_state_current.switch_status[0],
                gpio_state_current.switch_status[1],
                gpio_state_current.switch_status[2]);
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "其他状态: DC_CTRL=%d, FAN_SEN=%d",
                gpio_state_current.dc_ctrl,
                gpio_state_current.fan_sensor);
}

// 测试和验证函数
void GPIO_RelayTest(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 通道%d继电器测试开始 ===", channel + 1);
    
    // 测试继电器吸合
    Debug_Printf(DEBUG_LEVEL_INFO, "测试继电器吸合...");
    GPIO_RelayChannelControl(channel, RELAY_ACTION_ON);
    
    HAL_Delay(2000);  // 等待2秒
    
    // 测试继电器断开
    Debug_Printf(DEBUG_LEVEL_INFO, "测试继电器断开...");
    GPIO_RelayChannelControl(channel, RELAY_ACTION_OFF);
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 通道%d继电器测试完成 ===", channel + 1);
}

void GPIO_InterruptTest(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 中断功能测试 ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "请触发外部信号以测试中断功能");
    Debug_Printf(DEBUG_LEVEL_INFO, "监控中断状态中...");
    
    uint32_t test_start_time = HAL_GetTick();
    uint32_t last_print_time = test_start_time;
    
    // 测试30秒
    while(HAL_GetTick() - test_start_time < 30000)
    {
        // 每5秒输出一次中断状态
        if(HAL_GetTick() - last_print_time >= 5000)
        {
            uint8_t channel_flags = GPIO_GetInterruptStatus(INTERRUPT_TYPE_CHANNEL_ENABLE);
            uint8_t dc_flag = GPIO_GetInterruptStatus(INTERRUPT_TYPE_DC_CTRL);
            uint8_t key_flags = GPIO_GetInterruptStatus(INTERRUPT_TYPE_KEY);
            
            Debug_Printf(DEBUG_LEVEL_INFO, "中断状态: 通道=0x%02X, DC=0x%02X, 按键=0x%02X", 
                        channel_flags, dc_flag, key_flags);
            
            last_print_time = HAL_GetTick();
        }
        
        HAL_Delay(100);
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 中断功能测试完成 ===");
}

void GPIO_StateTest(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== GPIO状态测试开始 ===");
    
    // 启动状态监控
    GPIO_StateMonitorInit();
    
    for(int i = 0; i < 10; i++)
    {
        GPIO_StateMonitorUpdate();
        GPIO_StatePrint();
        
        uint8_t anomaly = GPIO_StateAnomalyCheck();
        if(anomaly)
        {
            Debug_Printf(DEBUG_LEVEL_WARN, "检测到状态异常");
        }
        
        HAL_Delay(1000);
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== GPIO状态测试完成 ===");
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
