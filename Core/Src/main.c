/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TEMP_MAX  30
#define TEMP_MIN  10

#define SPEED_MAX 1000
#define SPEED_MIN 3000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint32_t counterOutside = 0; //For testing only
uint32_t counterInside = 0; //For testing only
float temper_checkpoint=28;
uint32_t currentSpeed = 2000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char uartData[200];

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float temperature, humidity;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
 Lcd_PortType ports[] = { GPIOA, GPIOA, GPIOA, GPIOA};
 // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
 Lcd_PinType pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
 Lcd_HandleTypeDef lcd;
 // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
 lcd = Lcd_create(ports, pins, GPIOC, GPIO_PIN_13, GPIOC, GPIO_PIN_15, LCD_4_BIT_MODE);
// Lcd_cursor(&lcd, 0,2);
// Lcd_string(&lcd, "Temp C= ");
//
// Lcd_cursor(&lcd, 1,2);
// Lcd_string(&lcd, "RH % = ");

 	HAL_TIM_Base_Start(&htim1);

   snprintf(uartData, sizeof(uartData), "\r\nStart connection..\n\r");
   HAL_UART_Transmit(&huart1, (uint8_t *)uartData, sizeof(uartData), 10);

    sprintf(uartData, "Hi and welcome...");
	Lcd_cursor(&lcd, 0,0);
	Lcd_string(&lcd, uartData);
	HAL_Delay(1000);
	Lcd_clear(&lcd);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  StartAcqusition(&temperature, &humidity);

		sprintf(uartData, "\r\n(C)= %.1f%%\r\n ", temperature);
		// Ensure null-termination of the string
		uartData[sizeof(uartData) - 1] = '\0';
		HAL_UART_Transmit(&huart1, (uint8_t *)uartData, strlen(uartData), 10);
		sprintf(uartData, "Temp C= %.1f", temperature);
		Lcd_cursor(&lcd, 0,2);
		Lcd_string(&lcd, uartData);

		sprintf(uartData, "RH(%%)= %.1f%%\r\n", humidity);
		// Ensure null-termination of the string
		uartData[sizeof(uartData) - 1] = '\0';
		HAL_UART_Transmit(&huart1, (uint8_t *)uartData, strlen(uartData), 10);
		sprintf(uartData, "RH (%%)= %.1f", humidity);
		Lcd_cursor(&lcd, 1,2);
		Lcd_string(&lcd, uartData);

		if(temperature < temper_checkpoint)
		{
		stepCV (512, currentSpeed);
//		HAL_Delay(200);
		}else
		{
		stepCCV (512, currentSpeed);
//		HAL_Delay(200);
//		Lcd_clear(&lcd);
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2*/

  /* USER CODE END TIM1_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED-PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*
   * Init for Buttons PA11 PA12 (regulate step motor)
   */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*
   * Init for Buttons PA8 PA9 (regulate boarder temperature)
   */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*
    * Initi for a DHT22 sensor
    */
   /*Configure GPIO pin : Out- PB12 */
   GPIO_InitStruct.Pin = GPIO_PIN_12;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*
    * Initializatin for a Stepmotor inputs
 	*/
   /*Configure GPIO pin : IN1-PB6, IN2-PB5, IN3-PB4, IN4-PB3*/
 	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3;
 	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 	GPIO_InitStruct.Pull = GPIO_PULLUP;
 	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*
    * Initializatin for a LCD 1604
    */
   /*Configure GPIO pins : RS-PA13 and E-PA15 */
   GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   /*Configure GPIO pins : D4-A4, D5-A5, D6-A6, D7-A7*/
   GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  counterOutside++; //For testing only
  currentMillis = HAL_GetTick();
  if (GPIO_Pin == GPIO_PIN_9 && (currentMillis - previousMillis > 10))
  {
    counterInside++; //For testing only
    if(temper_checkpoint < TEMP_MAX)temper_checkpoint+=0.5;
    else temper_checkpoint = TEMP_MAX;

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    previousMillis = currentMillis;
  }
  else if (GPIO_Pin == GPIO_PIN_8 && (currentMillis - previousMillis > 10))
    {
      counterInside++; //For testing only
      if(temper_checkpoint > TEMP_MIN)temper_checkpoint-=0.5;
      else temper_checkpoint = TEMP_MIN;

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      previousMillis = currentMillis;
    }
  else if (GPIO_Pin == GPIO_PIN_12 && (currentMillis - previousMillis > 10))
    {
      counterInside++; //For testing only
      if(SPEED_MAX < currentSpeed) currentSpeed-=500;
      else currentSpeed = SPEED_MAX;

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      previousMillis = currentMillis;
    }
  else if (GPIO_Pin == GPIO_PIN_11 && (currentMillis - previousMillis > 10))
    {
      counterInside++; //For testing only
      if(currentSpeed < SPEED_MIN)currentSpeed+=500;
      else currentSpeed = SPEED_MIN;

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      previousMillis = currentMillis;
    }
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
