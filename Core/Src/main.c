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
  
  Debug_Printf(DEBUG_LEVEL_INFO, "GPIO����ģ���ʼ�����");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint32_t last_debug_time = 0;
  uint32_t last_state_update_time = 0;
  uint32_t loop_count = 0;
  
  Debug_Printf(DEBUG_LEVEL_INFO, "������ѭ������ʼGPIO���ܲ���");
  
  // ��Ӽ̵������Ա�־
  uint8_t relay_test_completed = 0;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    uint32_t current_time = HAL_GetTick();
    
    // ÿ100ms����һ��GPIO״̬���
    if(current_time - last_state_update_time >= 100)
    {
        GPIO_StateMonitorUpdate();
        
        // ���״̬�쳣
        if(GPIO_StateAnomalyCheck())
        {
            Debug_Printf(DEBUG_LEVEL_WARN, "��⵽GPIO״̬�쳣");
        }
        
        last_state_update_time = current_time;
    }
    
    // ������10����м̵������ԣ�ִֻ��һ�Σ�
    if(!relay_test_completed && current_time >= 10000)
    {
        Debug_Printf(DEBUG_LEVEL_INFO, "=== ��ʼ�򻯼̵������� ===");
        
        // ���ٲ�������ͨ���ļ̵��������ι��������
        for(int i = 0; i < 3; i++)
        {
            Debug_Printf(DEBUG_LEVEL_INFO, "����ͨ��%d�̵���", i + 1);
            
            // ι����������ֹ���Թ����и�λ
            HAL_IWDG_Refresh(&hiwdg);
            
            // ���Լ̵�������
            GPIO_RelayPulseControl((Channel_TypeDef)i, RELAY_ACTION_ON);
            HAL_Delay(100);  // ������ʱ���״̬
            
            // �ٴ�ι��
            HAL_IWDG_Refresh(&hiwdg);
            
            // ���״̬
            uint8_t status = GPIO_RelayStatusCheck((Channel_TypeDef)i);
            Debug_Printf(DEBUG_LEVEL_INFO, "  ͨ��%d���ϲ��Խ��: %s", i + 1, status ? "�ɹ�" : "ʧ��");
            
            HAL_Delay(500);  // ���500ms
            
            // ι������
            HAL_IWDG_Refresh(&hiwdg);
            
            // ���Լ̵����Ͽ�
            GPIO_RelayPulseControl((Channel_TypeDef)i, RELAY_ACTION_OFF);
            HAL_Delay(100);  // ������ʱ���״̬
            
            // �ٴ�ι��
            HAL_IWDG_Refresh(&hiwdg);
            
            status = GPIO_RelayStatusCheck((Channel_TypeDef)i);
            Debug_Printf(DEBUG_LEVEL_INFO, "  ͨ��%d�Ͽ����Խ��: %s", i + 1, status ? "ʧ��" : "�ɹ�");
            
            HAL_Delay(500);  // ���500ms
            
            // ���ι��
            HAL_IWDG_Refresh(&hiwdg);
        }
        
        Debug_Printf(DEBUG_LEVEL_INFO, "=== �̵���������� ===");
        relay_test_completed = 1;
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
