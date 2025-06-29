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

// ȫ�ֱ�������
static System_Status_TypeDef system_status = {0};
static Relay_Control_Task_TypeDef active_tasks[3] = {
    {CHANNEL_1, RELAY_CTRL_ACTION_OPEN, RELAY_CTRL_STATE_IDLE, 0, 0, ALARM_FLAG_NONE, 0},
    {CHANNEL_2, RELAY_CTRL_ACTION_OPEN, RELAY_CTRL_STATE_IDLE, 0, 0, ALARM_FLAG_NONE, 0},
    {CHANNEL_3, RELAY_CTRL_ACTION_OPEN, RELAY_CTRL_STATE_IDLE, 0, 0, ALARM_FLAG_NONE, 0}
};
static uint32_t alarm_flags_bitmap = 0;

// ��������
#define STABLE_CHECK_INTERVAL_MS    50      // �ȶ��Լ����50ms
#define STABLE_CHECK_TIMES          3       // �������3��
#define PULSE_DURATION_MS           500     // �������ʱ��500ms
#define VERIFY_DELAY_MS             500     // ��֤��ʱ500ms

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
 * @brief  �̵�������ģ���ʼ��
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_Init(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "�̵�������ģ���ʼ����ʼ");
    
    // ����ϵͳ״̬
    memset(&system_status, 0, sizeof(system_status));
    memset(active_tasks, 0, sizeof(active_tasks));
    alarm_flags_bitmap = 0;
    
    // ��ʼ��GPIO�̵���
    GPIO_RelayInitialize();
    
    // ����ϵͳ״̬
    Relay_Control_UpdateSystemStatus();
    
    // ����ϵͳ������־
    system_status.system_ready = 1;
    
    Debug_Printf(DEBUG_LEVEL_INFO, "�̵�������ģ���ʼ�����");
    return HAL_OK;
}

/**
 * @brief  �̵�������ģ�鸴λ
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_Reset(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "�̵�������ģ�鸴λ");
    
    // ֹͣ��������
    memset(active_tasks, 0, sizeof(active_tasks));
    
    // ��������쳣��־
    alarm_flags_bitmap = 0;
    
    // ���³�ʼ��
    return Relay_Control_Init();
}

/**
 * @brief  ����ͨ��ʹ���źţ��жϴ�������Ҫ��ڣ�
 * @param  channel: ͨ����
 * @param  enable_state: ʹ��״̬��0=�͵�ƽ����������1=�ߵ�ƽ�����رգ�
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_ProcessChannelEnable(Channel_TypeDef channel, uint8_t enable_state)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "����ͨ��%dʹ���źţ�״̬=%d", channel + 1, enable_state);
    
    // ���ϵͳ�Ƿ����
    if (!system_status.system_ready) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ϵͳδ�������ܾ�����ͨ��ʹ��");
        return HAL_ERROR;
    }
    
    // ����Ƿ����������ڴ����ͨ��
    if (Relay_Control_IsTaskActive(channel)) {
        Debug_Printf(DEBUG_LEVEL_WARN, "ͨ��%d���������ڴ���", channel + 1);
        return HAL_BUSY;
    }
    
    // ���ͨ��ʹ���ź��ȶ��ԣ�50ms���3�μ�⣩
    if (!Relay_Control_CheckChannelEnableStable(channel, enable_state)) {
        Debug_Printf(DEBUG_LEVEL_WARN, "ͨ��%dʹ���źŲ��ȶ�", channel + 1);
        return HAL_ERROR;
    }
    
    // ����ʹ��״̬������Ӧ����
    Relay_Control_Action_TypeDef action = (enable_state == 0) ? 
                                          RELAY_CTRL_ACTION_OPEN : 
                                          RELAY_CTRL_ACTION_CLOSE;
    
    return Relay_Control_CreateTask(channel, action);
}

/**
 * @brief  ִ��ͨ����������
 * @param  channel: ͨ����
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_ExecuteChannelOpen(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "ִ��ͨ��%d��������", channel + 1);
    
    // ��ʼ�ж�1���������ͨ���Ƿ�Ϊ�ߵ�ƽ
    if (!Relay_Control_CheckOtherChannelsIdle(channel)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "����ͨ��ʹ�ܳ�ͻ");
        Relay_Control_SetAlarmFlag(ALARM_FLAG_A);
        return HAL_ERROR;
    }
    
    // ��ʼ�ж�2���������ͨ���ļ̵����Ϳ���״̬�Ƿ�Ϊ�͵�ƽ
    if (!Relay_Control_CheckOtherChannelsStatus(channel)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "����ͨ��״̬�쳣");
        return HAL_ERROR;
    }
    
    // ִ�м̵�����������
    HAL_StatusTypeDef result = GPIO_RelayPulseControl(channel, RELAY_ACTION_ON);
    if (result != HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�̵����������ʧ��", channel + 1);
        return result;
    }
    
    // ��ʱ500ms����֤״̬
    for(int i = 0; i < 5; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    // ��֤ͨ��״̬
    if (!Relay_Control_VerifyChannelStatus(channel, RELAY_CTRL_ACTION_OPEN)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d������֤ʧ��", channel + 1);
        return HAL_ERROR;
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d�����ɹ�", channel + 1);
    return HAL_OK;
}

/**
 * @brief  ִ��ͨ���رղ���
 * @param  channel: ͨ����
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_ExecuteChannelClose(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "ִ��ͨ��%d�رղ���", channel + 1);
    
    // ִ�м̵����Ͽ�����
    HAL_StatusTypeDef result = GPIO_RelayPulseControl(channel, RELAY_ACTION_OFF);
    if (result != HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�̵����������ʧ��", channel + 1);
        return result;
    }
    
    // ��ʱ500ms����֤״̬
    for(int i = 0; i < 5; i++) {
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    // ��֤ͨ��״̬
    if (!Relay_Control_VerifyChannelStatus(channel, RELAY_CTRL_ACTION_CLOSE)) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�ر���֤ʧ��", channel + 1);
        return HAL_ERROR;
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d�رճɹ�", channel + 1);
    return HAL_OK;
}

/**
 * @brief  ���ͨ��ʹ���ź��ȶ���
 * @param  channel: ͨ����
 * @param  expected_state: ����״̬
 * @retval 1=�ȶ���0=���ȶ�
 */
uint8_t Relay_Control_CheckChannelEnableStable(Channel_TypeDef channel, uint8_t expected_state)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "���ͨ��%dʹ���ź��ȶ���", channel + 1);
    
    for (int i = 0; i < STABLE_CHECK_TIMES; i++) {
        uint8_t current_state = GPIO_ReadChannelEnable(channel);
        if (current_state != expected_state) {
            Debug_Printf(DEBUG_LEVEL_DEBUG, "ͨ��%d��%d�μ��ʧ�ܣ�����=%d��ʵ��=%d", 
                        channel + 1, i + 1, expected_state, current_state);
            return 0;
        }
        
        if (i < STABLE_CHECK_TIMES - 1) {
            HAL_Delay(STABLE_CHECK_INTERVAL_MS);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "ͨ��%dʹ���ź��ȶ��Լ��ͨ��", channel + 1);
    return 1;
}

/**
 * @brief  �������ͨ���Ƿ��ڿ���״̬
 * @param  active_channel: ��ǰ�����ͨ��
 * @retval 1=����ͨ�����У�0=�г�ͻ
 */
uint8_t Relay_Control_CheckOtherChannelsIdle(Channel_TypeDef active_channel)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "�������ͨ������״̬");
    
    for (int i = 0; i < 3; i++) {
        if (i == active_channel) continue;
        
        uint8_t enable_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        if (enable_state == 0) {  // �͵�ƽ��ʾ����
            Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d���ڼ���״̬����ͨ��%d��ͻ", 
                        i + 1, active_channel + 1);
            return 0;
        }
    }
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "����ͨ������״̬���ͨ��");
    return 1;
}

/**
 * @brief  �������ͨ����״̬����
 * @param  active_channel: ��ǰ�����ͨ��
 * @retval 1=״̬������0=״̬�쳣
 */
uint8_t Relay_Control_CheckOtherChannelsStatus(Channel_TypeDef active_channel)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "�������ͨ��״̬����");
    
    uint8_t error_detected = 0;
    
    for (int i = 0; i < 3; i++) {
        if (i == active_channel) continue;
        
        // ���̵���״̬
        uint8_t relay1_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2));
        uint8_t relay2_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2 + 1));
        uint8_t switch_state = GPIO_ReadSwitchStatus((Channel_TypeDef)i);
        
        // �̵���Ӧ��Ϊ�͵�ƽ���Ͽ�״̬��
        if (relay1_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�̵���1״̬�쳣", i + 1);
            Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_B + i * 2));
            error_detected = 1;
        }
        
        if (relay2_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�̵���2״̬�쳣", i + 1);
            Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_E + i));
            error_detected = 1;
        }
        
        // ����Ӧ��Ϊ�͵�ƽ���Ͽ�״̬��
        if (switch_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d����״̬�쳣", i + 1);
            Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_H + i));
            error_detected = 1;
        }
    }
    
    if (!error_detected) {
        Debug_Printf(DEBUG_LEVEL_DEBUG, "����ͨ��״̬���ͨ��");
    }
    
    return !error_detected;
}

/**
 * @brief  ��֤ͨ��״̬
 * @param  channel: ͨ����
 * @param  action: �����Ķ������
 * @retval 1=��֤ͨ����0=��֤ʧ��
 */
uint8_t Relay_Control_VerifyChannelStatus(Channel_TypeDef channel, Relay_Control_Action_TypeDef action)
{
    Debug_Printf(DEBUG_LEVEL_DEBUG, "��֤ͨ��%d״̬", channel + 1);
    
    // ��ȡ��ǰ״̬
    uint8_t relay1_state = GPIO_ReadRelayStatus((Relay_TypeDef)(channel * 2));
    uint8_t relay2_state = GPIO_ReadRelayStatus((Relay_TypeDef)(channel * 2 + 1));
    uint8_t switch_state = GPIO_ReadSwitchStatus(channel);
    
    Debug_Printf(DEBUG_LEVEL_DEBUG, "ͨ��%d״̬��K%d_1=%d, K%d_2=%d, SW%d=%d", 
                channel + 1, channel + 1, relay1_state, 
                channel + 1, relay2_state, channel + 1, switch_state);
    
    uint8_t expected_relay_state = (action == RELAY_CTRL_ACTION_OPEN) ? 1 : 0;
    uint8_t verification_passed = 1;
    
    // ��֤�̵���״̬
    if (relay1_state != expected_relay_state) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�̵���1״̬��֤ʧ��", channel + 1);
        Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_B + channel * 2));
        verification_passed = 0;
    }
    
    if (relay2_state != expected_relay_state) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�̵���2״̬��֤ʧ��", channel + 1);
        Relay_Control_SetAlarmFlag((Alarm_Flag_TypeDef)(ALARM_FLAG_E + channel));
        verification_passed = 0;
    }
    
    // ��֤����״̬�����Ӳ�����ӵĻ���
    if (switch_state != expected_relay_state) {
        Debug_Printf(DEBUG_LEVEL_DEBUG, "ͨ��%d����״̬��̵���״̬��һ�£�Ӳ��δ������������", channel + 1);
        // ������Ϊ������ΪSW״̬Ӳ��δ����
    }
    
    if (verification_passed) {
        Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d״̬��֤ͨ��", channel + 1);
    }
    
    return verification_passed;
}

/**
 * @brief  �����쳣��־
 * @param  flag: �쳣��־
 */
void Relay_Control_SetAlarmFlag(Alarm_Flag_TypeDef flag)
{
    if (flag != ALARM_FLAG_NONE) {
        alarm_flags_bitmap |= (1 << flag);
        Debug_Printf(DEBUG_LEVEL_ERROR, "�����쳣��־: %c", 'A' + flag - 1);
    }
}

/**
 * @brief  ����쳣��־
 * @param  flag: �쳣��־
 */
void Relay_Control_ClearAlarmFlag(Alarm_Flag_TypeDef flag)
{
    if (flag != ALARM_FLAG_NONE) {
        alarm_flags_bitmap &= ~(1 << flag);
        Debug_Printf(DEBUG_LEVEL_INFO, "����쳣��־: %c", 'A' + flag - 1);
    }
}

/**
 * @brief  ����쳣��־�Ƿ񼤻�
 * @param  flag: �쳣��־
 * @retval 1=���0=δ����
 */
uint8_t Relay_Control_IsAlarmActive(Alarm_Flag_TypeDef flag)
{
    return (alarm_flags_bitmap & (1 << flag)) ? 1 : 0;
}

/**
 * @brief  ��ȡ���м�����쳣��־
 * @retval �쳣��־λͼ
 */
uint32_t Relay_Control_GetActiveAlarms(void)
{
    return alarm_flags_bitmap;
}

/**
 * @brief  ����ϵͳ״̬
 */
void Relay_Control_UpdateSystemStatus(void)
{
    // ��������ͨ����״̬
    for (int i = 0; i < 3; i++) {
        system_status.channels[i].enable_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        system_status.channels[i].relay1_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2));
        system_status.channels[i].relay2_state = GPIO_ReadRelayStatus((Relay_TypeDef)(i * 2 + 1));
        system_status.channels[i].switch_state = GPIO_ReadSwitchStatus((Channel_TypeDef)i);
        system_status.channels[i].is_active = (system_status.channels[i].relay1_state && 
                                               system_status.channels[i].relay2_state);
    }
    
    // �����쳣��־
    system_status.active_alarm_flags = alarm_flags_bitmap;
    
    // ����ʱ���
    system_status.last_update_time = HAL_GetTick();
}

/**
 * @brief  ��ȡϵͳ״ָ̬��
 * @retval ϵͳ״ָ̬��
 */
System_Status_TypeDef* Relay_Control_GetSystemStatus(void)
{
    Relay_Control_UpdateSystemStatus();
    return &system_status;
}

/**
 * @brief  ��ӡϵͳ״̬
 */
void Relay_Control_PrintSystemStatus(void)
{
    Relay_Control_UpdateSystemStatus();
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== �̵�������ϵͳ״̬ ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "ϵͳ����: %s", system_status.system_ready ? "��" : "��");
    
    for (int i = 0; i < 3; i++) {
        Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d: EN=%d, K%d_1=%d, K%d_2=%d, SW%d=%d, ����=%s", 
                    i + 1, 
                    system_status.channels[i].enable_state,
                    i + 1, system_status.channels[i].relay1_state,
                    i + 1, system_status.channels[i].relay2_state,
                    i + 1, system_status.channels[i].switch_state,
                    system_status.channels[i].is_active ? "��" : "��");
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "�쳣��־: 0x%08lX", system_status.active_alarm_flags);
}

/**
 * @brief  �����̵�����������
 * @param  channel: ͨ����
 * @param  action: ���ƶ���
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_CreateTask(Channel_TypeDef channel, Relay_Control_Action_TypeDef action)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "����ͨ��%d�������񣬶���=%s", 
                channel + 1, (action == RELAY_CTRL_ACTION_OPEN) ? "����" : "�ر�");
    
    // ���ͨ����Χ
    if (channel >= 3) {
        return HAL_ERROR;
    }
    
    // ��ʼ������
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
 * @brief  �������м��������
 */
void Relay_Control_ProcessTasks(void)
{
    HAL_IWDG_Refresh(&hiwdg);  // ������ʼǰι��
    
    for (int i = 0; i < 3; i++) {
        if (active_tasks[i].completed || active_tasks[i].state == RELAY_CTRL_STATE_IDLE) {
            continue;
        }
        
        HAL_IWDG_Refresh(&hiwdg);  // ÿ��������ǰι��
        
        switch (active_tasks[i].state) {
            case RELAY_CTRL_STATE_CHECKING:
                // ���׶����ڴ�������ʱ��ɣ�ֱ�ӽ���ִ�н׶�
                active_tasks[i].state = RELAY_CTRL_STATE_EXECUTING;
                break;
                
            case RELAY_CTRL_STATE_EXECUTING:
                // ִ�м̵�������
                HAL_IWDG_Refresh(&hiwdg);  // ִ��ǰι��
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
                HAL_IWDG_Refresh(&hiwdg);  // ִ�к�ι��
                break;
                
            case RELAY_CTRL_STATE_COMPLETED:
                // �������
                active_tasks[i].completed = 1;
                Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d�����������", i + 1);
                break;
                
            case RELAY_CTRL_STATE_ERROR:
                // �������
                active_tasks[i].completed = 1;
                Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d��������ʧ��", i + 1);
                break;
                
            default:
                break;
        }
    }
}

/**
 * @brief  ���ָ��ͨ���Ƿ��м��������
 * @param  channel: ͨ����
 * @retval 1=�м�������0=�޼�������
 */
uint8_t Relay_Control_IsTaskActive(Channel_TypeDef channel)
{
    if (channel >= 3) return 0;
    
    return (!active_tasks[channel].completed && 
            active_tasks[channel].state != RELAY_CTRL_STATE_IDLE) ? 1 : 0;
}

/**
 * @brief  �̵�������ϵͳ�Լ�
 * @retval HAL״̬
 */
HAL_StatusTypeDef Relay_Control_SelfCheck(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== �̵�������ϵͳ�Լ쿪ʼ ===");
    
    uint8_t self_check_passed = 1;
    
    // ����ʼ״̬
    if (!Relay_Control_CheckInitialState()) {
        Debug_Printf(DEBUG_LEVEL_ERROR, "��ʼ״̬���ʧ��");
        self_check_passed = 0;
    }
    
    // ����ϵͳ״̬
    Relay_Control_UpdateSystemStatus();
    
    if (self_check_passed) {
        Debug_Printf(DEBUG_LEVEL_INFO, "�̵�������ϵͳ�Լ�ͨ��");
        return HAL_OK;
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "�̵�������ϵͳ�Լ�ʧ��");
        Relay_Control_SetAlarmFlag(ALARM_FLAG_N);
        return HAL_ERROR;
    }
}

/**
 * @brief  ���ϵͳ��ʼ״̬
 * @retval 1=������0=�쳣
 */
uint8_t Relay_Control_CheckInitialState(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "���ϵͳ��ʼ״̬");
    
    uint8_t check_passed = 1;
    
    // ���1����·�����ź�K1_EN��K2_EN��K3_EN��Ϊ�ߵ�ƽ
    for (int i = 0; i < 3; i++) {
        uint8_t enable_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        if (enable_state != 1) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%dʹ���źų�ʼ״̬�쳣������=1��ʵ��=%d", 
                        i + 1, enable_state);
            check_passed = 0;
        }
    }
    
    // ���2�����м̵����Ϳ���״̬��Ϊ�͵�ƽ
    for (int i = 0; i < 6; i++) {
        uint8_t relay_state = GPIO_ReadRelayStatus((Relay_TypeDef)i);
        if (relay_state != 0) {
            Debug_Printf(DEBUG_LEVEL_ERROR, "�̵���%d��ʼ״̬�쳣������=0��ʵ��=%d", 
                        i + 1, relay_state);
            check_passed = 0;
        }
    }
    
    for (int i = 0; i < 3; i++) {
        uint8_t switch_state = GPIO_ReadSwitchStatus((Channel_TypeDef)i);
        if (switch_state != 0) {
            Debug_Printf(DEBUG_LEVEL_DEBUG, "����%d��ʼ״̬������=0��ʵ��=%d��Ӳ��δ������������", 
                        i + 1, switch_state);
            // ������Ϊ���ʧ�ܣ���ΪSW״̬Ӳ��δ����
        }
    }
    
    if (check_passed) {
        Debug_Printf(DEBUG_LEVEL_INFO, "ϵͳ��ʼ״̬���ͨ��");
    }
    
    return check_passed;
}

/**
 * @brief  �̵������Ʋ���
 * @param  channel: ����ͨ��
 */
void Relay_Control_Test(Channel_TypeDef channel)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== ͨ��%d�̵������Ʋ��� ===", channel + 1);
    
    // ���Կ���
    Debug_Printf(DEBUG_LEVEL_INFO, "����ͨ��%d����", channel + 1);
    if (Relay_Control_ExecuteChannelOpen(channel) == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d��������ͨ��", channel + 1);
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d��������ʧ��", channel + 1);
    }
    
    HAL_Delay(2000);  // �ȴ�2��
    HAL_IWDG_Refresh(&hiwdg);
    
    // ���Թر�
    Debug_Printf(DEBUG_LEVEL_INFO, "����ͨ��%d�ر�", channel + 1);
    if (Relay_Control_ExecuteChannelClose(channel) == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d�رղ���ͨ��", channel + 1);
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "ͨ��%d�رղ���ʧ��", channel + 1);
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== ͨ��%d������� ===", channel + 1);
}

/**
 * @brief  ����ͨ���̵������Ʋ���
 */
void Relay_Control_TestAll(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "=== ȫ��ͨ���̵������Ʋ��Կ�ʼ ===");
    
    for (int i = 0; i < 3; i++) {
        Relay_Control_Test((Channel_TypeDef)i);
        HAL_Delay(1000);  // ͨ������1��
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    Debug_Printf(DEBUG_LEVEL_INFO, "=== ȫ��ͨ���̵������Ʋ������ ===");
}

/**
 * @brief  ��ӡͨ��״̬
 * @param  channel: ͨ����
 */
void Relay_Control_PrintChannelStatus(Channel_TypeDef channel)
{
    if (channel >= 3) return;
    
    Relay_Control_UpdateSystemStatus();
    
    Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d״̬����:", channel + 1);
    Debug_Printf(DEBUG_LEVEL_INFO, "  ʹ��״̬: %d", system_status.channels[channel].enable_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  �̵���1״̬: %d", system_status.channels[channel].relay1_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  �̵���2״̬: %d", system_status.channels[channel].relay2_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  ����״̬: %d", system_status.channels[channel].switch_state);
    Debug_Printf(DEBUG_LEVEL_INFO, "  ͨ������: %s", system_status.channels[channel].is_active ? "��" : "��");
}

/**
 * @brief  ��ӡ�쳣״̬
 */
void Relay_Control_PrintAlarmStatus(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "�쳣״̬����:");
    Debug_Printf(DEBUG_LEVEL_INFO, "  �쳣��־λͼ: 0x%08lX", alarm_flags_bitmap);
    
    if (alarm_flags_bitmap == 0) {
        Debug_Printf(DEBUG_LEVEL_INFO, "  ��ǰ���쳣");
        return;
    }
    
    for (int i = ALARM_FLAG_A; i <= ALARM_FLAG_N; i++) {
        if (Relay_Control_IsAlarmActive((Alarm_Flag_TypeDef)i)) {
            const char* alarm_names[] = {
                "", "ʹ�ܳ�ͻ", "K1_1_STA�쳣", "K2_1_STA�쳣", "K3_1_STA�쳣",
                "K1_2_STA�쳣", "K2_2_STA�쳣", "K3_2_STA�쳣",
                "SW1_STA�쳣", "SW2_STA�쳣", "SW3_STA�쳣",
                "NTC_1�¶��쳣", "NTC_2�¶��쳣", "NTC_3�¶��쳣", "�Լ��쳣"
            };
            Debug_Printf(DEBUG_LEVEL_INFO, "  �쳣%c: %s", 'A' + i - 1, alarm_names[i]);
        }
    }
}

/* USER CODE END 1 */ 


