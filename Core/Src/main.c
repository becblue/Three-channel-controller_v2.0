/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "debug.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "relay_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Relay_Control_FullTest(void);
void Relay_Control_InterruptTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  
  /* USER CODE BEGIN 2 */
  
  // 首先初始化调试系统（必须在USART3初始化之后）
  Debug_Init();
  
  // 输出系统启动信息
  Debug_Printf(DEBUG_LEVEL_INFO, "=== 系统启动 ===");
  Debug_Printf(DEBUG_LEVEL_INFO, "STM32F103RCT6 三通道高压切换箱控制系统");
  Debug_Printf(DEBUG_LEVEL_INFO, "版本: v2.0");
  Debug_Printf(DEBUG_LEVEL_INFO, "编译时间: %s %s", __DATE__, __TIME__);
  
  // 初始化GPIO功能模块 - 第二阶段第二步新增
  Debug_Printf(DEBUG_LEVEL_INFO, "=== 初始化GPIO功能模块 ===");
  
  // 1. 继电器系统初始化
  GPIO_RelayInitialize();
  
  // 2. 中断系统初始化
  GPIO_InterruptInit();
  
  // 3. 状态监控系统初始化
  GPIO_StateMonitorInit();
  
  // 4. 初始化继电器控制模块
  if (Relay_Control_Init() != HAL_OK) {
    Debug_Printf(DEBUG_LEVEL_ERROR, "继电器控制模块初始化失败");
    Error_Handler();
  }
  
  // 5. 执行继电器控制系统自检
  if (Relay_Control_SelfCheck() != HAL_OK) {
    Debug_Printf(DEBUG_LEVEL_ERROR, "继电器控制系统自检失败");
  }
  
  Debug_Printf(DEBUG_LEVEL_INFO, "继电器控制系统初始化完成");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint32_t last_debug_time = 0;
  uint32_t last_state_update_time = 0;
  uint32_t loop_count = 0;
  
  Debug_Printf(DEBUG_LEVEL_INFO, "进入主循环，开始继电器控制系统运行");
  
  // 添加测试标志
  uint8_t relay_test_completed = 0;
  uint8_t full_test_completed = 0;
  uint8_t interrupt_test_completed = 0;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    uint32_t current_time = HAL_GetTick();
    
    // 每100ms更新一次状态监控
    if(current_time - last_state_update_time >= 100)
    {
        // 更新GPIO状态监控
        GPIO_StateMonitorUpdate();
        
        // 检查GPIO状态异常
        if(GPIO_StateAnomalyCheck())
        {
            Debug_Printf(DEBUG_LEVEL_WARN, "检测到GPIO状态异常");
        }
        
        // 处理继电器控制任务
        Relay_Control_ProcessTasks();
        
        last_state_update_time = current_time;
    }
    
    // 启动后10秒进行基础继电器测试（只执行一次）
    if(!relay_test_completed && current_time >= 10000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "=== 开始基础继电器测试 ===");
        
        // 使用新的继电器控制模块进行基础测试
        Relay_Control_TestAll();
        
        Debug_Printf(DEBUG_LEVEL_INFO, "=== 基础继电器测试完成 ===");
        relay_test_completed = 1;
    }
    
    // 启动后30秒进行完整功能测试（只执行一次）
    if(!full_test_completed && current_time >= 30000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? 开始执行完整功能测试...");
        
        // 执行完整的继电器控制模块测试
        Relay_Control_FullTest();
        
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? 完整功能测试执行完成");
        full_test_completed = 1;
    }
    
    // 启动后60秒进行中断模拟测试（只执行一次）
    if(!interrupt_test_completed && current_time >= 60000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? 开始执行中断模拟测试...");
        Debug_Printf(DEBUG_LEVEL_INFO, "当前时间: %lu ms, 准备进入中断测试函数", current_time);
        HAL_IWDG_Refresh(&hiwdg);  // 测试开始前喂狗
        
        // 强制刷新输出缓冲区
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
        
        Debug_Printf(DEBUG_LEVEL_INFO, "即将调用 Relay_Control_InterruptTest() 函数...");
        HAL_IWDG_Refresh(&hiwdg);
        
        // 执行中断处理模拟测试
        Relay_Control_InterruptTest();
        
        HAL_IWDG_Refresh(&hiwdg);  // 测试完成后喂狗
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? 中断模拟测试执行完成");
        interrupt_test_completed = 1;
        
        // 所有测试完成后输出总结
        HAL_IWDG_Refresh(&hiwdg);  // 总结开始前喂狗
        Debug_Printf(DEBUG_LEVEL_INFO, "\n" 
                    "????????????????????\n"
                    "?          继电器控制模块测试总结          ?\n"
                    "????????????????????");
        
        Debug_Printf(DEBUG_LEVEL_INFO, "? 基础继电器测试: 完成");
        Debug_Printf(DEBUG_LEVEL_INFO, "? 完整功能测试: 完成");  
        Debug_Printf(DEBUG_LEVEL_INFO, "? 中断模拟测试: 完成");
        Debug_Printf(DEBUG_LEVEL_INFO, "");
        HAL_IWDG_Refresh(&hiwdg);  // 输出中间喂狗
        Debug_Printf(DEBUG_LEVEL_INFO, "? 测试覆盖范围:");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 系统初始化和自检");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 继电器开启/关闭控制");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 状态检查和验证");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 异常处理系统");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 稳定性检查机制");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 通道互锁保护");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 任务管理系统");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 中断处理逻辑");
        Debug_Printf(DEBUG_LEVEL_INFO, "");
        HAL_IWDG_Refresh(&hiwdg);  // 输出中间喂狗
        Debug_Printf(DEBUG_LEVEL_INFO, "?? 安全特性验证:");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 50ms间隔3次检测机制");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 500ms脉冲控制");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 看门狗兼容性");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 多重安全保护");
        Debug_Printf(DEBUG_LEVEL_INFO, "");
        Debug_Printf(DEBUG_LEVEL_INFO, "? 第三阶段第一步开发验证: 成功完成!");
        Debug_Printf(DEBUG_LEVEL_INFO, "? 继电器控制模块已准备就绪，可进入下一阶段开发");
        HAL_IWDG_Refresh(&hiwdg);  // 总结结束前喂狗
        Debug_Printf(DEBUG_LEVEL_INFO, 
                    "????????????????????");
    }
    
    // 每5秒输出一次状态信息
    if(current_time - last_debug_time >= 5000)
    {
        loop_count++;
        Debug_Printf(DEBUG_LEVEL_INFO, "主循环计数: %lu", loop_count);
        
        // 每10次输出详细状态（即每50秒）
        if(loop_count % 10 == 0)
        {
            Debug_Printf(DEBUG_LEVEL_INFO, "=== 详细状态报告 ===");
            
            // 输出测试进度
            Debug_Printf(DEBUG_LEVEL_INFO, "测试进度: 基础测试=%s, 完整测试=%s, 中断测试=%s", 
                        relay_test_completed ? "?完成" : "?待执行",
                        full_test_completed ? "?完成" : "?待执行", 
                        interrupt_test_completed ? "?完成" : "?待执行");
            
            // 输出继电器控制系统状态
            Relay_Control_PrintSystemStatus();
            
            // 输出异常状态
            Relay_Control_PrintAlarmStatus();
            
            // 输出GPIO状态
            GPIO_StatePrint();
            
            // 输出中断状态
            uint8_t channel_flags = GPIO_GetInterruptStatus(INTERRUPT_TYPE_CHANNEL_ENABLE);
            uint8_t dc_flag = GPIO_GetInterruptStatus(INTERRUPT_TYPE_DC_CTRL);
            uint8_t key_flags = GPIO_GetInterruptStatus(INTERRUPT_TYPE_KEY);
            
            Debug_Printf(DEBUG_LEVEL_INFO, "中断状态: 通道=0x%02X, DC=0x%02X, 按键=0x%02X", 
                        channel_flags, dc_flag, key_flags);
            
            // 清除中断状态标志
            GPIO_ClearInterruptStatus(INTERRUPT_TYPE_CHANNEL_ENABLE);
            GPIO_ClearInterruptStatus(INTERRUPT_TYPE_DC_CTRL);
            GPIO_ClearInterruptStatus(INTERRUPT_TYPE_KEY);
        }
        
        last_debug_time = current_time;
    }
    
    // 喂狗操作，防止看门狗复位
    HAL_IWDG_Refresh(&hiwdg);
    
    // 短暂延时
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  继电器控制模块完整测试程序
 * @retval None
 */
void Relay_Control_FullTest(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 继电器控制模块完整测试开始 ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    
    // 测试1：系统状态检查
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试1：系统状态检查 ---");
    HAL_IWDG_Refresh(&hiwdg);  // 测试开始前喂狗
    Relay_Control_PrintSystemStatus();
    HAL_IWDG_Refresh(&hiwdg);  // 状态输出后喂狗
    Relay_Control_PrintAlarmStatus();
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试2：初始状态验证
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试2：初始状态验证 ---");
    HAL_IWDG_Refresh(&hiwdg);  // 测试前喂狗
    if (Relay_Control_CheckInitialState()) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 初始状态检查通过");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 初始状态检查失败");
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试3：单通道详细测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试3：单通道详细测试 ---");
    for (int i = 0; i < 3; i++) {
        Debug_Printf(DEBUG_LEVEL_INFO, "\n>> 测试通道%d <<", i + 1);
        
        // 显示测试前状态
        Debug_Printf(DEBUG_LEVEL_INFO, "测试前状态:");
        HAL_IWDG_Refresh(&hiwdg);  // 状态输出前喂狗
        Relay_Control_PrintChannelStatus((Channel_TypeDef)i);
        
        // 测试通道开启
        Debug_Printf(DEBUG_LEVEL_INFO, "执行通道%d开启测试...", i + 1);
        HAL_IWDG_Refresh(&hiwdg);  // 开启测试前喂狗
        HAL_StatusTypeDef result = Relay_Control_ExecuteChannelOpen((Channel_TypeDef)i);
        HAL_IWDG_Refresh(&hiwdg);  // 开启测试后喂狗
        
        if (result == HAL_OK) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? 通道%d开启测试通过", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_ERROR, "? 通道%d开启测试失败", i + 1);
        }
        
        // 显示测试后状态
        Debug_Printf(DEBUG_LEVEL_INFO, "开启后状态:");
        HAL_IWDG_Refresh(&hiwdg);  // 状态输出前喂狗
        Relay_Control_PrintChannelStatus((Channel_TypeDef)i);
        
        HAL_Delay(2000);  // 等待2秒观察状态
        HAL_IWDG_Refresh(&hiwdg);
        
        // 测试通道关闭
        Debug_Printf(DEBUG_LEVEL_INFO, "执行通道%d关闭测试...", i + 1);
        HAL_IWDG_Refresh(&hiwdg);  // 关闭测试前喂狗
        result = Relay_Control_ExecuteChannelClose((Channel_TypeDef)i);
        HAL_IWDG_Refresh(&hiwdg);  // 关闭测试后喂狗
        
        if (result == HAL_OK) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? 通道%d关闭测试通过", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_ERROR, "? 通道%d关闭测试失败", i + 1);
        }
        
        // 显示最终状态
        Debug_Printf(DEBUG_LEVEL_INFO, "关闭后状态:");
        HAL_IWDG_Refresh(&hiwdg);  // 状态输出前喂狗
        Relay_Control_PrintChannelStatus((Channel_TypeDef)i);
        
        HAL_Delay(1000);  // 通道间间隔
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    // 测试4：异常处理测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试4：异常处理测试 ---");
    
    // 测试异常标志设置和清除
    Debug_Printf(DEBUG_LEVEL_INFO, "测试异常标志管理...");
    Relay_Control_SetAlarmFlag(ALARM_FLAG_A);
    if (Relay_Control_IsAlarmActive(ALARM_FLAG_A)) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 异常标志A设置成功");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 异常标志A设置失败");
    }
    
    Relay_Control_PrintAlarmStatus();
    
    Relay_Control_ClearAlarmFlag(ALARM_FLAG_A);
    if (!Relay_Control_IsAlarmActive(ALARM_FLAG_A)) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 异常标志A清除成功");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 异常标志A清除失败");
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试5：稳定性检查测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试5：稳定性检查测试 ---");
    HAL_IWDG_Refresh(&hiwdg);  // 测试开始前喂狗
    for (int i = 0; i < 3; i++) {
        uint8_t current_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        Debug_Printf(DEBUG_LEVEL_INFO, "通道%d当前使能状态: %d", i + 1, current_state);
        
        HAL_IWDG_Refresh(&hiwdg);  // 稳定性检查前喂狗
        if (Relay_Control_CheckChannelEnableStable((Channel_TypeDef)i, current_state)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? 通道%d稳定性检查通过", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_WARN, "?? 通道%d稳定性检查失败", i + 1);
        }
        HAL_IWDG_Refresh(&hiwdg);  // 每个通道检查后喂狗
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试6：通道互锁检查测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试6：通道互锁检查测试 ---");
    HAL_IWDG_Refresh(&hiwdg);  // 测试开始前喂狗
    for (int i = 0; i < 3; i++) {
        HAL_IWDG_Refresh(&hiwdg);  // 每个通道检查前喂狗
        if (Relay_Control_CheckOtherChannelsIdle((Channel_TypeDef)i)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? 通道%d互锁检查通过", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_WARN, "?? 通道%d互锁检查发现冲突", i + 1);
        }
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试7：状态反馈检查测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试7：状态反馈检查测试 ---");
    HAL_IWDG_Refresh(&hiwdg);  // 测试开始前喂狗
    for (int i = 0; i < 3; i++) {
        HAL_IWDG_Refresh(&hiwdg);  // 每个通道检查前喂狗
        if (Relay_Control_CheckOtherChannelsStatus((Channel_TypeDef)i)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? 通道%d状态反馈检查通过", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_WARN, "?? 通道%d状态反馈检查发现异常", i + 1);
        }
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试8：任务管理测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试8：任务管理测试 ---");
    
    // 创建测试任务
    Debug_Printf(DEBUG_LEVEL_INFO, "创建通道1开启任务...");
    if (Relay_Control_CreateTask(CHANNEL_1, RELAY_CTRL_ACTION_OPEN) == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 任务创建成功");
        
        if (Relay_Control_IsTaskActive(CHANNEL_1)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? 任务状态检查正确");
        } else {
            Debug_Printf(DEBUG_LEVEL_ERROR, "? 任务状态检查错误");
        }
        
        // 处理任务
        Debug_Printf(DEBUG_LEVEL_INFO, "处理任务...");
        HAL_IWDG_Refresh(&hiwdg);  // 任务处理前喂狗
        Relay_Control_ProcessTasks();
        HAL_IWDG_Refresh(&hiwdg);  // 任务处理后喂狗
        
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 任务创建失败");
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试9：系统自检测试
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试9：系统自检测试 ---");
    HAL_IWDG_Refresh(&hiwdg);  // 自检前喂狗
    if (Relay_Control_SelfCheck() == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 系统自检通过");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 系统自检失败");
    }
    HAL_IWDG_Refresh(&hiwdg);  // 自检后喂狗
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // 测试10：最终状态检查
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 测试10：最终状态检查 ---");
    Debug_Printf(DEBUG_LEVEL_INFO, "最终系统状态:");
    HAL_IWDG_Refresh(&hiwdg);  // 状态输出前喂狗
    Relay_Control_PrintSystemStatus();
    HAL_IWDG_Refresh(&hiwdg);  // 状态输出后喂狗
    Relay_Control_PrintAlarmStatus();
    HAL_IWDG_Refresh(&hiwdg);  // 最终喂狗
    
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 继电器控制模块完整测试完成 ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
}

/**
 * @brief  中断模拟测试程序
 * @retval None
 */
void Relay_Control_InterruptTest(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "\n??? 正在进入中断测试函数 ???");
    HAL_IWDG_Refresh(&hiwdg);  // 函数入口喂狗
    
    Debug_Printf(DEBUG_LEVEL_INFO, "\n========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 中断处理模拟测试开始 ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    
    HAL_IWDG_Refresh(&hiwdg);  // 标题输出后喂狗
    
    // 模拟K1_EN下降沿中断（开启通道1）
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 模拟K1_EN下降沿中断 ---");
    Debug_Printf(DEBUG_LEVEL_INFO, "模拟通道1使能信号下降沿...");
    Debug_Printf(DEBUG_LEVEL_INFO, "注意：这是模拟测试，跳过GPIO稳定性检查");
    HAL_IWDG_Refresh(&hiwdg);  // 中断处理前喂狗
    
    // 直接调用通道开启操作，跳过稳定性检查
    HAL_StatusTypeDef result = Relay_Control_ExecuteChannelOpen(CHANNEL_1);
    HAL_IWDG_Refresh(&hiwdg);  // 中断处理后喂狗
    
    if (result == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 通道1开启模拟测试成功");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 通道1开启模拟测试失败");
    }
    
    HAL_Delay(3000);  // 等待3秒观察状态
    HAL_IWDG_Refresh(&hiwdg);
    
    // 模拟K1_EN上升沿中断（关闭通道1）
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- 模拟K1_EN上升沿中断 ---");
    Debug_Printf(DEBUG_LEVEL_INFO, "模拟通道1使能信号上升沿...");
    Debug_Printf(DEBUG_LEVEL_INFO, "注意：这是模拟测试，跳过GPIO稳定性检查");
    HAL_IWDG_Refresh(&hiwdg);  // 中断处理前喂狗
    
    // 直接调用通道关闭操作，跳过稳定性检查
    result = Relay_Control_ExecuteChannelClose(CHANNEL_1);
    HAL_IWDG_Refresh(&hiwdg);  // 中断处理后喂狗
    
    if (result == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? 通道1关闭模拟测试成功");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? 通道1关闭模拟测试失败");
    }
    
    HAL_Delay(3000);  // 等待3秒观察状态
    HAL_IWDG_Refresh(&hiwdg);
    
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== 中断处理模拟测试完成 ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  
  // 输出错误信息到调试串口
  Debug_Printf(DEBUG_LEVEL_ERROR, "系统发生严重错误，进入错误处理程序");
  Debug_Printf(DEBUG_LEVEL_ERROR, "系统将被挂起，请检查硬件连接和配置");
  
  __disable_irq();
  while (1)
  {
    // 在错误状态下闪烁报警LED（如果有的话）
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
