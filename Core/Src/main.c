/* USER CODE BEGIN Header */

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
typedef enum eLedStatus
{
    led_off = GPIO_PIN_SET,
    led_on = GPIO_PIN_RESET
} eLedStatus_t;
typedef enum eLedColor
{
    color_none,      // LED All OFF
    color_red,      // LED Color Red
    color_green,    // LED Color Green
    color_blue,     // LED Color Blue
    color_yellow,   // LED Color Yellow
    color_magenta,  // LED Color Magenta
    color_cyan,     // LED Color Cyan
    color_white     // LED Color White
} eLedColor_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t fanModeCount = 0;
uint8_t ampModeCount = 0;

uint8_t voltIn = 0;
uint8_t chgStat = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void pwmDutySet(int duty);
void ledColor(eLedStatus_t led_color);
void ledControl(eLedStatus_t led_r, eLedStatus_t led_g, eLedStatus_t led_b);
void boostEnable(GPIO_PinState bstStatus);

int uartStartMsg(void);
int gpioInit(void);
int ledInit(void);
int pwmInit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PWM Duty Set
void pwmDutySet(int duty)
{
    if((duty >= 0) && (duty <= 100))
    {
        if((duty == 0) || (duty == 100))
        {
            TIM3->CCR1 = ((100 / 100) * duty);
        }
        else
        {
            TIM3->CCR1 = ((100 / 100) * duty) - 1;
        }
    }
    else
    {
        TIM3->CCR1 = 0;
    }
}

// LED Color Set
void ledColor(eLedStatus_t led_color)
{
    switch(led_color)
    {
        case color_none:
            ledControl(led_off, led_off, led_off); break;
        case color_red:
            ledControl(led_on, led_off, led_off); break;
        case color_green:
            ledControl(led_off, led_on, led_off); break;
        case color_blue:
            ledControl(led_off, led_off, led_on); break;
        case color_yellow:
            ledControl(led_on, led_on, led_off); break;
        case color_magenta:
            ledControl(led_on, led_off, led_on); break;
        case color_cyan:
            ledControl(led_off, led_on, led_on); break;
        case color_white:
            ledControl(led_on, led_on, led_on); break;
        default:
            ledControl(led_off, led_off, led_off); break;
    }
}

// LED Control
void ledControl(eLedStatus_t led_r, eLedStatus_t led_g, eLedStatus_t led_b)
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, led_r);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, led_g);
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, led_b);
}

void boostEnable(GPIO_PinState bstStatus)
{
    HAL_GPIO_WritePin(BST_EN_GPIO_Port, BST_EN_Pin, bstStatus);
}

// ALL OUTPUT OFF
int gpioInit(void)
{
    HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_RESET); //AMP OFF
    HAL_GPIO_WritePin(BST_EN_GPIO_Port, BST_EN_Pin, GPIO_PIN_RESET); // BOOSTER OFF
    HAL_GPIO_WritePin(LED_CHG_GPIO_Port, LED_CHG_Pin, GPIO_PIN_RESET); // CHG LED OFF
}

// LED ROTATE 100ms
int ledInit(void)
{
    // RED
    ledColor(color_red);
    HAL_Delay(1000);
    // GRN
    ledColor(color_green);
    HAL_Delay(1000);
    // BLU
    ledColor(color_blue);
    HAL_Delay(1000);
    // YEL
    ledColor(color_yellow);
    HAL_Delay(1000);
    // MAGENTA
    ledColor(color_magenta);
    HAL_Delay(1000);
    // CYAN
    ledColor(color_cyan);
    HAL_Delay(1000);
    // WHT
    ledColor(color_white);
    HAL_Delay(1000);
    // None
    ledColor(color_none);
}

// PWM Start
int pwmInit(void)
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}

// FAN Control
int fanControl(void)
{
    if(HAL_GPIO_ReadPin(FAN_SW_GPIO_Port, FAN_SW_Pin) == 0)
    {
        while(HAL_GPIO_ReadPin(FAN_SW_GPIO_Port, FAN_SW_Pin) == 0);
        fanModeCount++;
        if(fanModeCount >= 4) fanModeCount = 0;
    }

    switch (fanModeCount)
    {
        case 0: // PWM OFF
            ledColor(color_none);
            boostEnable(GPIO_PIN_RESET);
            pwmDutySet(0);
            break;
        case 1: // PWM 30%
            ledColor(color_red);
            boostEnable(GPIO_PIN_SET);
            pwmDutySet(65);
            break;
        case 2: // PWM 50%
            ledColor(color_green);
            boostEnable(GPIO_PIN_SET);
            pwmDutySet(80);
            break;
        case 3: // PWM 80%
            ledColor(color_blue);
            boostEnable(GPIO_PIN_SET);
            pwmDutySet(100);
            break;
    }
}

int ampControl()
{
    if(HAL_GPIO_ReadPin(AMP_SW_GPIO_Port,AMP_SW_Pin) == 0)
    {
        while(HAL_GPIO_ReadPin(AMP_SW_GPIO_Port, AMP_SW_Pin) == 0);
        ampModeCount++;
        if(ampModeCount >= 2) ampModeCount = 0;
    }

    switch (ampModeCount)
    {
        case 0:
            HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_SET);
            break;

        case 1:
            HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_RESET);
    }



}

int chargeDetect()
{
    voltIn = HAL_GPIO_ReadPin(VBUS_DET_GPIO_Port, VBUS_DET_Pin);
    chgStat = HAL_GPIO_ReadPin(CHG_STAT_GPIO_Port, CHG_STAT_Pin);

    if(voltIn == 1)
    {
        if (chgStat == 0)
        {
            HAL_GPIO_WritePin(LED_CHG_GPIO_Port, LED_CHG_Pin, GPIO_PIN_SET);
        } else
        {
            HAL_GPIO_WritePin(LED_CHG_GPIO_Port, LED_CHG_Pin, GPIO_PIN_RESET);
        }
    }
    else
    {
        HAL_GPIO_WritePin(LED_CHG_GPIO_Port,LED_CHG_Pin,GPIO_PIN_RESET);
    }
}
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    pwmInit();
    gpioInit();
    ledInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        chargeDetect();
        fanControl();
        ampControl();
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  sConfigOC.Pulse = 0;
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

  /*Configure GPIO pins : VBUS_DET_Pin AMP_SW_Pin */
  GPIO_InitStruct.Pin = VBUS_DET_Pin|AMP_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHG_STAT_Pin */
  GPIO_InitStruct.Pin = CHG_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CHG_STAT_GPIO_Port, &GPIO_InitStruct);

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
