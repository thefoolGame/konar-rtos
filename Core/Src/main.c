/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lsm6dsl_register_map.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for blinkLD2 */
osThreadId_t blinkLD2Handle;
const osThreadAttr_t blinkLD2_attributes = {
  .name = "blinkLD2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for printUART2 */
osThreadId_t printUART2Handle;
const osThreadAttr_t printUART2_attributes = {
  .name = "printUART2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for printB1 */
osThreadId_t printB1Handle;
const osThreadAttr_t printB1_attributes = {
  .name = "printB1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SemB1 */
osSemaphoreId_t SemB1Handle;
const osSemaphoreAttr_t SemB1_attributes = {
  .name = "SemB1"
};
/* Definitions for SemDRDY */
osSemaphoreId_t SemDRDYHandle;
const osSemaphoreAttr_t SemDRDY_attributes = {
  .name = "SemDRDY"
};
/* USER CODE BEGIN PV */
const uint8_t lsm6dls_address = 0b11010110;
const uint8_t bttn_prssd_mssg[] = "BTTN PRSSD\r\n";

uint8_t acc_data[6] = { 0 };
int16_t acc_x = 0;
int16_t acc_y = 0;
int16_t acc_z = 0;

uint8_t sensor_str[17];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartBlinkLD2(void *argument);
void StartPrintUART2(void *argument);
void StartPrintB1(void *argument);
void StartTask04(void *argument);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemB1 */
  SemB1Handle = osSemaphoreNew(1, 1, &SemB1_attributes);

  /* creation of SemDRDY */
  SemDRDYHandle = osSemaphoreNew(1, 1, &SemDRDY_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinkLD2 */
  blinkLD2Handle = osThreadNew(StartBlinkLD2, NULL, &blinkLD2_attributes);

  /* creation of printUART2 */
  printUART2Handle = osThreadNew(StartPrintUART2, NULL, &printUART2_attributes);

  /* creation of printB1 */
  printB1Handle = osThreadNew(StartPrintB1, NULL, &printB1_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_INT1_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
		osSemaphoreRelease(SemB1Handle);
	}
	if (GPIO_Pin == LSM6DSL_INT1_Pin) {
		osSemaphoreRelease(SemDRDYHandle);
	}
}
void vApplicationDaemonTaskStartupHook(void) {
	{	// CHECK SENSOR ADDRESS
		uint8_t real_chcksm = 0;
		uint8_t exp_chcksm = 0b01101010;	// WHO_AM_I value
		HAL_I2C_Mem_Read(&hi2c1, lsm6dls_address, LSM6DSL_WHO_AM_I, 1,
				&real_chcksm, 1, HAL_MAX_DELAY);

		if (real_chcksm == exp_chcksm) {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
	}
	{	// RESET SENSOR
		uint8_t reset = BOOT | SW_RESET;
		HAL_I2C_Mem_Write(&hi2c1, lsm6dls_address, LSM6DSL_CTRL3_C, 1, &reset,
				1, HAL_MAX_DELAY);
	}
	HAL_Delay(1);	// without delay sensor don't update values
	{	// SET XL OUTPUT RATE AND SCALE
		uint8_t output_rate_and_scale = F26H | G4;
		HAL_I2C_Mem_Write(&hi2c1, lsm6dls_address, LSM6DSL_CTRL1_XL, 1,
				&output_rate_and_scale, 1,
				HAL_MAX_DELAY);
	}
	{	// CONFIGURE INT1
		uint8_t int1_settings = INT1_DRDY_XL;
		HAL_I2C_Mem_Write(&hi2c1, lsm6dls_address, LSM6DSL_INT1_CTRL, 1,
				&int1_settings, 1,
				HAL_MAX_DELAY);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkLD2 */
/**
 * @brief  Function implementing the blinkLD2 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinkLD2 */
void StartBlinkLD2(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		/*
		 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		 osDelay(77);	// 13Hz
		 */
		osSemaphoreAcquire(SemDRDYHandle, osWaitForever);
		HAL_I2C_Mem_Read(&hi2c1, lsm6dls_address, LSM6DSL_OUTX_L_XL, 1,
				acc_data, sizeof(acc_data), HAL_MAX_DELAY);
		acc_x = (int16_t) (acc_data[1] << 8 | acc_data[0]) * 1000
				/ (INT16_MAX / 4 + 1);	// hard coded to 0.000g format for G4
		acc_y = (int16_t) (acc_data[3] << 8 | acc_data[2]) * 1000
				/ (INT16_MAX / 4 + 1);// in order to change scaling change 4 to other max value of G
		acc_z = (int16_t) (acc_data[5] << 8 | acc_data[4]) * 1000
				/ (INT16_MAX / 4 + 1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPrintUART2 */
/**
 * @brief Function implementing the printUART2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPrintUART2 */
void StartPrintUART2(void *argument)
{
  /* USER CODE BEGIN StartPrintUART2 */
	/* Infinite loop */
	for (;;) {
		snprintf((char*) sensor_str, sizeof(sensor_str), "%d %d %d\r\n", acc_x,
				acc_y, acc_z);
		HAL_UART_Transmit(&huart2, sensor_str, strlen((char*) sensor_str),
		HAL_MAX_DELAY);
		osDelay(91);	// 11Hz
	}
  /* USER CODE END StartPrintUART2 */
}

/* USER CODE BEGIN Header_StartPrintB1 */
/**
 * @brief Function implementing the printB1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPrintB1 */
void StartPrintB1(void *argument)
{
  /* USER CODE BEGIN StartPrintB1 */
	/* Infinite loop */
	for (;;) {
		// duuno why but function executes on start
		// guess semaphore initialize with 1
		osSemaphoreAcquire(SemB1Handle, osWaitForever); // doesn't work with osSemaphoreWait
		HAL_UART_Transmit(&huart2, bttn_prssd_mssg,
				strlen((char*) bttn_prssd_mssg), HAL_MAX_DELAY);
	}
  /* USER CODE END StartPrintB1 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		osDelay(38);	// 13Hz
	}
  /* USER CODE END StartTask04 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
