/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t alarmflag = 0;
RTC_TimeTypeDef sTime = {0};
RTC_AlarmTypeDef sAlarm = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
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
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  sAlarm.AlarmTime.Hours = 1;
  sAlarm.AlarmTime.Minutes = 13;
  sAlarm.AlarmTime.Seconds = 5;
  sAlarm.Alarm = RTC_ALARM_A;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(100);
	  RTC_TimeTypeDef tmpTime;
	  RTC_AlarmTypeDef tmpAlarm;
	  HAL_RTC_GetTime(&hrtc, &tmpTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetAlarm(&hrtc, &tmpAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);

	  if(alarmflag == 0) {
		  uint8_t secFirstDigit = tmpTime.Seconds%10;
		  uint8_t secSecondDigit = tmpTime.Seconds/10;

	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, secFirstDigit & 0x01);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, secFirstDigit & 0x02);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, secFirstDigit & 0x04);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, secFirstDigit & 0x08);

	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, secSecondDigit & 0x01);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, secSecondDigit & 0x02);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, secSecondDigit & 0x04);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, secSecondDigit & 0x08);

	  	  uint8_t minFirstDigit = tmpTime.Minutes%10;
	  	  uint8_t minSecondDigit = tmpTime.Minutes/10;

	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, minFirstDigit & 0x01);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, minFirstDigit & 0x02);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, minFirstDigit & 0x04);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, minFirstDigit & 0x08);

	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, minSecondDigit & 0x01);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, minSecondDigit & 0x02);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, minSecondDigit & 0x04);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, minSecondDigit & 0x08);

	  	  uint8_t hourFirstDigit = tmpTime.Hours%10;
	  	  uint8_t hourSecondDigit = tmpTime.Hours/10;

	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, hourFirstDigit & 0x01);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, hourFirstDigit & 0x02);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, hourFirstDigit & 0x04);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, hourFirstDigit & 0x08);

	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, hourSecondDigit & 0x01);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, hourSecondDigit & 0x02);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, hourSecondDigit & 0x04);
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, hourSecondDigit & 0x08);

	  }
	  else if (alarmflag == 1) {
		  uint8_t secFirstDigit = tmpAlarm.AlarmTime.Seconds%10;
		  uint8_t secSecondDigit = tmpAlarm.AlarmTime.Seconds/10;

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, secFirstDigit & 0x01);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, secFirstDigit & 0x02);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, secFirstDigit & 0x04);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, secFirstDigit & 0x08);

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, secSecondDigit & 0x01);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, secSecondDigit & 0x02);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, secSecondDigit & 0x04);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, secSecondDigit & 0x08);

		  uint8_t minFirstDigit = tmpAlarm.AlarmTime.Minutes%10;
		  uint8_t minSecondDigit = tmpAlarm.AlarmTime.Minutes/10;

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, minFirstDigit & 0x01);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, minFirstDigit & 0x02);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, minFirstDigit & 0x04);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, minFirstDigit & 0x08);

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, minSecondDigit & 0x01);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, minSecondDigit & 0x02);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, minSecondDigit & 0x04);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, minSecondDigit & 0x08);

		  uint8_t hourFirstDigit = tmpAlarm.AlarmTime.Hours%10;
		  uint8_t hourSecondDigit = tmpAlarm.AlarmTime.Hours/10;

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, hourFirstDigit & 0x01);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, hourFirstDigit & 0x02);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, hourFirstDigit & 0x04);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, hourFirstDigit & 0x08);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, hourSecondDigit & 0x01);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, hourSecondDigit & 0x02);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, hourSecondDigit & 0x04);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, hourSecondDigit & 0x08);
	  }
	  if (tmpAlarm.AlarmTime.Hours == tmpTime.Hours) {
		  if (tmpAlarm.AlarmTime.Minutes == tmpTime.Minutes) {
			  if(tmpAlarm.AlarmTime.Seconds == tmpTime.Seconds) {
				  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
				  HAL_Delay(5000);
				  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			  }
		  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 1;
  sTime.Minutes = 12;
  sTime.Seconds = 00;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  DateToUpdate.Month = RTC_MONTH_MAY;
  DateToUpdate.Date = 6;
  DateToUpdate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 1;
  sAlarm.AlarmTime.Minutes = 13;
  sAlarm.AlarmTime.Seconds = 5;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HourMinus_Pin AlarmSet_Pin SecondMinus_Pin SecondPlus_Pin 
                           MinuteMinus_Pin MinutePlus_Pin HourPlus_Pin */
  GPIO_InitStruct.Pin = HourMinus_Pin|AlarmSet_Pin|SecondMinus_Pin|SecondPlus_Pin 
                          |MinuteMinus_Pin|MinutePlus_Pin|HourPlus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	UNUSED(GPIO_Pin);
	switch(GPIO_Pin) {
		case AlarmSet_Pin: if (alarmflag) alarmflag = 0; else alarmflag = 1; break;
		case SecondPlus_Pin: if (alarmflag) sAlarm.AlarmTime.Seconds++; break;
		case MinutePlus_Pin: if (alarmflag) sAlarm.AlarmTime.Minutes++; break;
		case HourPlus_Pin: if (alarmflag) sAlarm.AlarmTime.Hours++; break;
		case SecondMinus_Pin: if (alarmflag) sAlarm.AlarmTime.Seconds--; break;
		case MinuteMinus_Pin: if (alarmflag) sAlarm.AlarmTime.Minutes--; break;
		case HourMinus_Pin: if (alarmflag) sAlarm.AlarmTime.Hours--; break;
	}
	if(alarmflag) HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN);

}
void HAL_RTC_AlarmEventCallBack(RTC_HandleTypeDef *hrtc) {
	UNUSED(hrtc);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
