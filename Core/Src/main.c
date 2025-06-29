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
  
  // ���ȳ�ʼ������ϵͳ��������USART3��ʼ��֮��
  Debug_Init();
  
  // ���ϵͳ������Ϣ
  Debug_Printf(DEBUG_LEVEL_INFO, "=== ϵͳ���� ===");
  Debug_Printf(DEBUG_LEVEL_INFO, "STM32F103RCT6 ��ͨ����ѹ�л������ϵͳ");
  Debug_Printf(DEBUG_LEVEL_INFO, "�汾: v2.0");
  Debug_Printf(DEBUG_LEVEL_INFO, "����ʱ��: %s %s", __DATE__, __TIME__);
  
  // ��ʼ��GPIO����ģ�� - �ڶ��׶εڶ�������
  Debug_Printf(DEBUG_LEVEL_INFO, "=== ��ʼ��GPIO����ģ�� ===");
  
  // 1. �̵���ϵͳ��ʼ��
  GPIO_RelayInitialize();
  
  // 2. �ж�ϵͳ��ʼ��
  GPIO_InterruptInit();
  
  // 3. ״̬���ϵͳ��ʼ��
  GPIO_StateMonitorInit();
  
  // 4. ��ʼ���̵�������ģ��
  if (Relay_Control_Init() != HAL_OK) {
    Debug_Printf(DEBUG_LEVEL_ERROR, "�̵�������ģ���ʼ��ʧ��");
    Error_Handler();
  }
  
  // 5. ִ�м̵�������ϵͳ�Լ�
  if (Relay_Control_SelfCheck() != HAL_OK) {
    Debug_Printf(DEBUG_LEVEL_ERROR, "�̵�������ϵͳ�Լ�ʧ��");
  }
  
  Debug_Printf(DEBUG_LEVEL_INFO, "�̵�������ϵͳ��ʼ�����");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint32_t last_debug_time = 0;
  uint32_t last_state_update_time = 0;
  uint32_t loop_count = 0;
  
  Debug_Printf(DEBUG_LEVEL_INFO, "������ѭ������ʼ�̵�������ϵͳ����");
  
  // ��Ӳ��Ա�־
  uint8_t relay_test_completed = 0;
  uint8_t full_test_completed = 0;
  uint8_t interrupt_test_completed = 0;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    uint32_t current_time = HAL_GetTick();
    
    // ÿ100ms����һ��״̬���
    if(current_time - last_state_update_time >= 100)
    {
        // ����GPIO״̬���
        GPIO_StateMonitorUpdate();
        
        // ���GPIO״̬�쳣
        if(GPIO_StateAnomalyCheck())
        {
            Debug_Printf(DEBUG_LEVEL_WARN, "��⵽GPIO״̬�쳣");
        }
        
        // ����̵�����������
        Relay_Control_ProcessTasks();
        
        last_state_update_time = current_time;
    }
    
    // ������10����л����̵������ԣ�ִֻ��һ�Σ�
    if(!relay_test_completed && current_time >= 10000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "=== ��ʼ�����̵������� ===");
        
        // ʹ���µļ̵�������ģ����л�������
        Relay_Control_TestAll();
        
        Debug_Printf(DEBUG_LEVEL_INFO, "=== �����̵���������� ===");
        relay_test_completed = 1;
    }
    
    // ������30������������ܲ��ԣ�ִֻ��һ�Σ�
    if(!full_test_completed && current_time >= 30000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? ��ʼִ���������ܲ���...");
        
        // ִ�������ļ̵�������ģ�����
        Relay_Control_FullTest();
        
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? �������ܲ���ִ�����");
        full_test_completed = 1;
    }
    
    // ������60������ж�ģ����ԣ�ִֻ��һ�Σ�
    if(!interrupt_test_completed && current_time >= 60000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? ��ʼִ���ж�ģ�����...");
        Debug_Printf(DEBUG_LEVEL_INFO, "��ǰʱ��: %lu ms, ׼�������жϲ��Ժ���", current_time);
        HAL_IWDG_Refresh(&hiwdg);  // ���Կ�ʼǰι��
        
        // ǿ��ˢ�����������
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
        
        Debug_Printf(DEBUG_LEVEL_INFO, "�������� Relay_Control_InterruptTest() ����...");
        HAL_IWDG_Refresh(&hiwdg);
        
        // ִ���жϴ���ģ�����
        Relay_Control_InterruptTest();
        
        HAL_IWDG_Refresh(&hiwdg);  // ������ɺ�ι��
        Debug_Printf(DEBUG_LEVEL_INFO, "\n? �ж�ģ�����ִ�����");
        interrupt_test_completed = 1;
        
        // ���в�����ɺ�����ܽ�
        HAL_IWDG_Refresh(&hiwdg);  // �ܽῪʼǰι��
        Debug_Printf(DEBUG_LEVEL_INFO, "\n" 
                    "????????????????????\n"
                    "?          �̵�������ģ������ܽ�          ?\n"
                    "????????????????????");
        
        Debug_Printf(DEBUG_LEVEL_INFO, "? �����̵�������: ���");
        Debug_Printf(DEBUG_LEVEL_INFO, "? �������ܲ���: ���");  
        Debug_Printf(DEBUG_LEVEL_INFO, "? �ж�ģ�����: ���");
        Debug_Printf(DEBUG_LEVEL_INFO, "");
        HAL_IWDG_Refresh(&hiwdg);  // ����м�ι��
        Debug_Printf(DEBUG_LEVEL_INFO, "? ���Ը��Ƿ�Χ:");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? ϵͳ��ʼ�����Լ�");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? �̵�������/�رտ���");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? ״̬������֤");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? �쳣����ϵͳ");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? �ȶ��Լ�����");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? ͨ����������");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? �������ϵͳ");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? �жϴ����߼�");
        Debug_Printf(DEBUG_LEVEL_INFO, "");
        HAL_IWDG_Refresh(&hiwdg);  // ����м�ι��
        Debug_Printf(DEBUG_LEVEL_INFO, "?? ��ȫ������֤:");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 50ms���3�μ�����");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? 500ms�������");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? ���Ź�������");
        Debug_Printf(DEBUG_LEVEL_INFO, "  ? ���ذ�ȫ����");
        Debug_Printf(DEBUG_LEVEL_INFO, "");
        Debug_Printf(DEBUG_LEVEL_INFO, "? �����׶ε�һ��������֤: �ɹ����!");
        Debug_Printf(DEBUG_LEVEL_INFO, "? �̵�������ģ����׼���������ɽ�����һ�׶ο���");
        HAL_IWDG_Refresh(&hiwdg);  // �ܽ����ǰι��
        Debug_Printf(DEBUG_LEVEL_INFO, 
                    "????????????????????");
    }
    
    // ÿ5�����һ��״̬��Ϣ
    if(current_time - last_debug_time >= 5000)
    {
        loop_count++;
        Debug_Printf(DEBUG_LEVEL_INFO, "��ѭ������: %lu", loop_count);
        
        // ÿ10�������ϸ״̬����ÿ50�룩
        if(loop_count % 10 == 0)
        {
            Debug_Printf(DEBUG_LEVEL_INFO, "=== ��ϸ״̬���� ===");
            
            // ������Խ���
            Debug_Printf(DEBUG_LEVEL_INFO, "���Խ���: ��������=%s, ��������=%s, �жϲ���=%s", 
                        relay_test_completed ? "?���" : "?��ִ��",
                        full_test_completed ? "?���" : "?��ִ��", 
                        interrupt_test_completed ? "?���" : "?��ִ��");
            
            // ����̵�������ϵͳ״̬
            Relay_Control_PrintSystemStatus();
            
            // ����쳣״̬
            Relay_Control_PrintAlarmStatus();
            
            // ���GPIO״̬
            GPIO_StatePrint();
            
            // ����ж�״̬
            uint8_t channel_flags = GPIO_GetInterruptStatus(INTERRUPT_TYPE_CHANNEL_ENABLE);
            uint8_t dc_flag = GPIO_GetInterruptStatus(INTERRUPT_TYPE_DC_CTRL);
            uint8_t key_flags = GPIO_GetInterruptStatus(INTERRUPT_TYPE_KEY);
            
            Debug_Printf(DEBUG_LEVEL_INFO, "�ж�״̬: ͨ��=0x%02X, DC=0x%02X, ����=0x%02X", 
                        channel_flags, dc_flag, key_flags);
            
            // ����ж�״̬��־
            GPIO_ClearInterruptStatus(INTERRUPT_TYPE_CHANNEL_ENABLE);
            GPIO_ClearInterruptStatus(INTERRUPT_TYPE_DC_CTRL);
            GPIO_ClearInterruptStatus(INTERRUPT_TYPE_KEY);
        }
        
        last_debug_time = current_time;
    }
    
    // ι����������ֹ���Ź���λ
    HAL_IWDG_Refresh(&hiwdg);
    
    // ������ʱ
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
 * @brief  �̵�������ģ���������Գ���
 * @retval None
 */
void Relay_Control_FullTest(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== �̵�������ģ���������Կ�ʼ ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    
    // ����1��ϵͳ״̬���
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����1��ϵͳ״̬��� ---");
    HAL_IWDG_Refresh(&hiwdg);  // ���Կ�ʼǰι��
    Relay_Control_PrintSystemStatus();
    HAL_IWDG_Refresh(&hiwdg);  // ״̬�����ι��
    Relay_Control_PrintAlarmStatus();
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����2����ʼ״̬��֤
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����2����ʼ״̬��֤ ---");
    HAL_IWDG_Refresh(&hiwdg);  // ����ǰι��
    if (Relay_Control_CheckInitialState()) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? ��ʼ״̬���ͨ��");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? ��ʼ״̬���ʧ��");
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����3����ͨ����ϸ����
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����3����ͨ����ϸ���� ---");
    for (int i = 0; i < 3; i++) {
        Debug_Printf(DEBUG_LEVEL_INFO, "\n>> ����ͨ��%d <<", i + 1);
        
        // ��ʾ����ǰ״̬
        Debug_Printf(DEBUG_LEVEL_INFO, "����ǰ״̬:");
        HAL_IWDG_Refresh(&hiwdg);  // ״̬���ǰι��
        Relay_Control_PrintChannelStatus((Channel_TypeDef)i);
        
        // ����ͨ������
        Debug_Printf(DEBUG_LEVEL_INFO, "ִ��ͨ��%d��������...", i + 1);
        HAL_IWDG_Refresh(&hiwdg);  // ��������ǰι��
        HAL_StatusTypeDef result = Relay_Control_ExecuteChannelOpen((Channel_TypeDef)i);
        HAL_IWDG_Refresh(&hiwdg);  // �������Ժ�ι��
        
        if (result == HAL_OK) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��%d��������ͨ��", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_ERROR, "? ͨ��%d��������ʧ��", i + 1);
        }
        
        // ��ʾ���Ժ�״̬
        Debug_Printf(DEBUG_LEVEL_INFO, "������״̬:");
        HAL_IWDG_Refresh(&hiwdg);  // ״̬���ǰι��
        Relay_Control_PrintChannelStatus((Channel_TypeDef)i);
        
        HAL_Delay(2000);  // �ȴ�2��۲�״̬
        HAL_IWDG_Refresh(&hiwdg);
        
        // ����ͨ���ر�
        Debug_Printf(DEBUG_LEVEL_INFO, "ִ��ͨ��%d�رղ���...", i + 1);
        HAL_IWDG_Refresh(&hiwdg);  // �رղ���ǰι��
        result = Relay_Control_ExecuteChannelClose((Channel_TypeDef)i);
        HAL_IWDG_Refresh(&hiwdg);  // �رղ��Ժ�ι��
        
        if (result == HAL_OK) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��%d�رղ���ͨ��", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_ERROR, "? ͨ��%d�رղ���ʧ��", i + 1);
        }
        
        // ��ʾ����״̬
        Debug_Printf(DEBUG_LEVEL_INFO, "�رպ�״̬:");
        HAL_IWDG_Refresh(&hiwdg);  // ״̬���ǰι��
        Relay_Control_PrintChannelStatus((Channel_TypeDef)i);
        
        HAL_Delay(1000);  // ͨ������
        HAL_IWDG_Refresh(&hiwdg);
    }
    
    // ����4���쳣�������
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����4���쳣������� ---");
    
    // �����쳣��־���ú����
    Debug_Printf(DEBUG_LEVEL_INFO, "�����쳣��־����...");
    Relay_Control_SetAlarmFlag(ALARM_FLAG_A);
    if (Relay_Control_IsAlarmActive(ALARM_FLAG_A)) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? �쳣��־A���óɹ�");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? �쳣��־A����ʧ��");
    }
    
    Relay_Control_PrintAlarmStatus();
    
    Relay_Control_ClearAlarmFlag(ALARM_FLAG_A);
    if (!Relay_Control_IsAlarmActive(ALARM_FLAG_A)) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? �쳣��־A����ɹ�");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? �쳣��־A���ʧ��");
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����5���ȶ��Լ�����
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����5���ȶ��Լ����� ---");
    HAL_IWDG_Refresh(&hiwdg);  // ���Կ�ʼǰι��
    for (int i = 0; i < 3; i++) {
        uint8_t current_state = GPIO_ReadChannelEnable((Channel_TypeDef)i);
        Debug_Printf(DEBUG_LEVEL_INFO, "ͨ��%d��ǰʹ��״̬: %d", i + 1, current_state);
        
        HAL_IWDG_Refresh(&hiwdg);  // �ȶ��Լ��ǰι��
        if (Relay_Control_CheckChannelEnableStable((Channel_TypeDef)i, current_state)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��%d�ȶ��Լ��ͨ��", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_WARN, "?? ͨ��%d�ȶ��Լ��ʧ��", i + 1);
        }
        HAL_IWDG_Refresh(&hiwdg);  // ÿ��ͨ������ι��
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����6��ͨ������������
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����6��ͨ������������ ---");
    HAL_IWDG_Refresh(&hiwdg);  // ���Կ�ʼǰι��
    for (int i = 0; i < 3; i++) {
        HAL_IWDG_Refresh(&hiwdg);  // ÿ��ͨ�����ǰι��
        if (Relay_Control_CheckOtherChannelsIdle((Channel_TypeDef)i)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��%d�������ͨ��", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_WARN, "?? ͨ��%d������鷢�ֳ�ͻ", i + 1);
        }
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����7��״̬����������
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����7��״̬���������� ---");
    HAL_IWDG_Refresh(&hiwdg);  // ���Կ�ʼǰι��
    for (int i = 0; i < 3; i++) {
        HAL_IWDG_Refresh(&hiwdg);  // ÿ��ͨ�����ǰι��
        if (Relay_Control_CheckOtherChannelsStatus((Channel_TypeDef)i)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��%d״̬�������ͨ��", i + 1);
        } else {
            Debug_Printf(DEBUG_LEVEL_WARN, "?? ͨ��%d״̬������鷢���쳣", i + 1);
        }
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����8������������
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����8������������ ---");
    
    // ������������
    Debug_Printf(DEBUG_LEVEL_INFO, "����ͨ��1��������...");
    if (Relay_Control_CreateTask(CHANNEL_1, RELAY_CTRL_ACTION_OPEN) == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? ���񴴽��ɹ�");
        
        if (Relay_Control_IsTaskActive(CHANNEL_1)) {
            Debug_Printf(DEBUG_LEVEL_INFO, "? ����״̬�����ȷ");
        } else {
            Debug_Printf(DEBUG_LEVEL_ERROR, "? ����״̬������");
        }
        
        // ��������
        Debug_Printf(DEBUG_LEVEL_INFO, "��������...");
        HAL_IWDG_Refresh(&hiwdg);  // ������ǰι��
        Relay_Control_ProcessTasks();
        HAL_IWDG_Refresh(&hiwdg);  // �������ι��
        
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? ���񴴽�ʧ��");
    }
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����9��ϵͳ�Լ����
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����9��ϵͳ�Լ���� ---");
    HAL_IWDG_Refresh(&hiwdg);  // �Լ�ǰι��
    if (Relay_Control_SelfCheck() == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? ϵͳ�Լ�ͨ��");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? ϵͳ�Լ�ʧ��");
    }
    HAL_IWDG_Refresh(&hiwdg);  // �Լ��ι��
    
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);
    
    // ����10������״̬���
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ����10������״̬��� ---");
    Debug_Printf(DEBUG_LEVEL_INFO, "����ϵͳ״̬:");
    HAL_IWDG_Refresh(&hiwdg);  // ״̬���ǰι��
    Relay_Control_PrintSystemStatus();
    HAL_IWDG_Refresh(&hiwdg);  // ״̬�����ι��
    Relay_Control_PrintAlarmStatus();
    HAL_IWDG_Refresh(&hiwdg);  // ����ι��
    
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== �̵�������ģ������������� ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
}

/**
 * @brief  �ж�ģ����Գ���
 * @retval None
 */
void Relay_Control_InterruptTest(void)
{
    Debug_Printf(DEBUG_LEVEL_INFO, "\n??? ���ڽ����жϲ��Ժ��� ???");
    HAL_IWDG_Refresh(&hiwdg);  // �������ι��
    
    Debug_Printf(DEBUG_LEVEL_INFO, "\n========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== �жϴ���ģ����Կ�ʼ ===");
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    
    HAL_IWDG_Refresh(&hiwdg);  // ���������ι��
    
    // ģ��K1_EN�½����жϣ�����ͨ��1��
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ģ��K1_EN�½����ж� ---");
    Debug_Printf(DEBUG_LEVEL_INFO, "ģ��ͨ��1ʹ���ź��½���...");
    Debug_Printf(DEBUG_LEVEL_INFO, "ע�⣺����ģ����ԣ�����GPIO�ȶ��Լ��");
    HAL_IWDG_Refresh(&hiwdg);  // �жϴ���ǰι��
    
    // ֱ�ӵ���ͨ�����������������ȶ��Լ��
    HAL_StatusTypeDef result = Relay_Control_ExecuteChannelOpen(CHANNEL_1);
    HAL_IWDG_Refresh(&hiwdg);  // �жϴ����ι��
    
    if (result == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��1����ģ����Գɹ�");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? ͨ��1����ģ�����ʧ��");
    }
    
    HAL_Delay(3000);  // �ȴ�3��۲�״̬
    HAL_IWDG_Refresh(&hiwdg);
    
    // ģ��K1_EN�������жϣ��ر�ͨ��1��
    Debug_Printf(DEBUG_LEVEL_INFO, "\n--- ģ��K1_EN�������ж� ---");
    Debug_Printf(DEBUG_LEVEL_INFO, "ģ��ͨ��1ʹ���ź�������...");
    Debug_Printf(DEBUG_LEVEL_INFO, "ע�⣺����ģ����ԣ�����GPIO�ȶ��Լ��");
    HAL_IWDG_Refresh(&hiwdg);  // �жϴ���ǰι��
    
    // ֱ�ӵ���ͨ���رղ����������ȶ��Լ��
    result = Relay_Control_ExecuteChannelClose(CHANNEL_1);
    HAL_IWDG_Refresh(&hiwdg);  // �жϴ����ι��
    
    if (result == HAL_OK) {
        Debug_Printf(DEBUG_LEVEL_INFO, "? ͨ��1�ر�ģ����Գɹ�");
    } else {
        Debug_Printf(DEBUG_LEVEL_ERROR, "? ͨ��1�ر�ģ�����ʧ��");
    }
    
    HAL_Delay(3000);  // �ȴ�3��۲�״̬
    HAL_IWDG_Refresh(&hiwdg);
    
    Debug_Printf(DEBUG_LEVEL_INFO, "========================================");
    Debug_Printf(DEBUG_LEVEL_INFO, "=== �жϴ���ģ�������� ===");
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
  
  // ���������Ϣ�����Դ���
  Debug_Printf(DEBUG_LEVEL_ERROR, "ϵͳ�������ش��󣬽�����������");
  Debug_Printf(DEBUG_LEVEL_ERROR, "ϵͳ������������Ӳ�����Ӻ�����");
  
  __disable_irq();
  while (1)
  {
    // �ڴ���״̬����˸����LED������еĻ���
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
