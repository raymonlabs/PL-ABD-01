/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_ON GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t AMP_SW_Stat = 0;
uint8_t FAN_SW_Stat = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int uartStartMsg(void)
{
    uint8_t start_msg[] = "Hello Paper !! \r\n" ;
    HAL_UART_Transmit(&huart1,start_msg,sizeof(start_msg),10);
    HAL_Delay(100);
}
int gpioInit()
{
    HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_RESET); //AMP OFF
    HAL_GPIO_WritePin(BST_EN_GPIO_Port, BST_EN_Pin, GPIO_PIN_RESET); // BOOSTER OFF
    HAL_GPIO_WritePin(LED_CHG_GPIO_Port, LED_CHG_Pin, GPIO_PIN_RESET); // BOOSTER OFF
}
int ledInit()
{
    //RED
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_OFF);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_OFF);
    HAL_Delay(100);
    //GRN
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_OFF);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_OFF);
    HAL_Delay(100);
    //BLU
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_OFF);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_OFF);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_ON);
    HAL_Delay(100);
    //YEL
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_OFF);
    HAL_Delay(250);
    //MAGENTA
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_OFF);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_ON);
    HAL_Delay(250);
    //CYAN
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_OFF);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_ON);
    HAL_Delay(250);
    //WHT
    HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin, LED_ON);
    HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin, LED_ON);
    HAL_Delay(250);


}
int pwmInit()
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

}
/* USER CODE END 0 */

/*
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    pwmInit();
    uartStartMsg();
    gpioInit();
    ledInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if(HAL_GPIO_ReadPin(AMP_SW_GPIO_Port,AMP_SW_Pin) == 0)
    {
        if (AMP_SW_Stat < 1)
        {
            AMP_SW_Stat++;
            HAL_GPIO_WritePin(AMP_SD_GPIO_Port,AMP_SD_Pin, GPIO_PIN_SET);
        }
        else
        {
            AMP_SW_Stat = 0;
            HAL_GPIO_WritePin(AMP_SD_GPIO_Port,AMP_SD_Pin, GPIO_PIN_RESET);
        }

    }
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_CHG_GPIO_Port, LED_CHG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AMP_SD_Pin|BST_EN_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FAN_SW_Pin */
  GPIO_InitStruct.Pin = FAN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FAN_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_CHG_Pin */
  GPIO_InitStruct.Pin = LED_CHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_CHG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AMP_SD_Pin BST_EN_Pin LED_G_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = AMP_SD_Pin|BST_EN_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VBUS_DET_Pin CHG_STAT_Pin AMP_SW_Pin */
  GPIO_InitStruct.Pin = VBUS_DET_Pin|CHG_STAT_Pin|AMP_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_R_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
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
