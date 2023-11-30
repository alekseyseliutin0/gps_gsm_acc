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
#include "string.h"
#include <stdbool.h>
#include <stdlib.h>
#include "gps.h"
#include "gsm.h"
#include "lis2dh12_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_R_ON  HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_RESET)
#define LED_R_OFF HAL_GPIO_WritePin(GPIOB, LED_R_Pin, GPIO_PIN_SET)
#define LED_G_ON  HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET)
#define LED_G_OFF HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_SET)
#define LED_B_ON  HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_RESET)
#define LED_B_OFF HAL_GPIO_WritePin(GPIOB, LED_B_Pin, GPIO_PIN_SET)

#define PWR_GPS_ON  HAL_GPIO_WritePin(PWR_GPS_EN_GPIO_Port, PWR_GPS_EN_Pin, GPIO_PIN_SET)
#define PWR_GPS_OFF HAL_GPIO_WritePin(PWR_GPS_EN_GPIO_Port, PWR_GPS_EN_Pin, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GPS */
osThreadId_t GPSHandle;
uint32_t GPS_TaskBuffer[ 128 ];
osStaticThreadDef_t GPS_TaskControlBlock;
const osThreadAttr_t GPS_attributes = {
  .name = "GPS",
  .stack_mem = &GPS_TaskBuffer[0],
  .stack_size = sizeof(GPS_TaskBuffer),
  .cb_mem = &GPS_TaskControlBlock,
  .cb_size = sizeof(GPS_TaskControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GSM */
osThreadId_t GSMHandle;
uint32_t GSMBuffer[ 128 ];
osStaticThreadDef_t GSMControlBlock;
const osThreadAttr_t GSM_attributes = {
  .name = "GSM",
  .stack_mem = &GSMBuffer[0],
  .stack_size = sizeof(GSMBuffer),
  .cb_mem = &GSMControlBlock,
  .cb_size = sizeof(GSMControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LIS */
osThreadId_t LISHandle;
uint32_t LISBuffer[ 128 ];
osStaticThreadDef_t LISControlBlock;
const osThreadAttr_t LIS_attributes = {
  .name = "LIS",
  .stack_mem = &LISBuffer[0],
  .stack_size = sizeof(LISBuffer),
  .cb_mem = &LISControlBlock,
  .cb_size = sizeof(LISControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void GPS_Task(void *argument);
void GSM_Task(void *argument);
void LIS_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t gps_data[200];
extern GPS_INFO gps_info;
extern uint8_t gps_pack[200];

extern uint8_t gsm_data[200];
uint8_t u3;
extern uint8_t p_w;
extern uint8_t signal_level;


static axis3bit16_t data_raw_acceleration;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float temperature_degC;
static uint8_t whoamI;


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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GPS */
  GPSHandle = osThreadNew(GPS_Task, NULL, &GPS_attributes);

  /* creation of GSM */
  GSMHandle = osThreadNew(GSM_Task, NULL, &GSM_attributes);

  /* creation of LIS */
  LISHandle = osThreadNew(LIS_Task, NULL, &LIS_attributes);

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
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWR_GSM_Pin|PWR_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_G_Pin|LED_R_Pin|LED_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACC_EN_GPIO_Port, ACC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_GPS_EN_GPIO_Port, PWR_GPS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PWR_GSM_Pin PWR_KEY_Pin */
  GPIO_InitStruct.Pin = PWR_GSM_Pin|PWR_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin PWR_GPS_EN_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|PWR_GPS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_EN_Pin */
  GPIO_InitStruct.Pin = ACC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACC_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GPS_Task */
/**
* @brief Function implementing the GPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPS_Task */
void GPS_Task(void *argument)
{
  /* USER CODE BEGIN GPS_Task */
	PWR_GPS_ON;
	PWR_GSM_ON;
	KEY_GSM_OFF;
	osDelay(150);
	KEY_GSM_ON;
	osDelay(800);
	KEY_GSM_OFF;
  /* Infinite loop */
  for(;;)
  {
		HAL_UART_Receive_IT(&huart1, gps_data, sizeof(gps_data)/sizeof(gps_data[0]));
		while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY) vTaskDelay(50);
		for (uint8_t i=0; i<sizeof(gps_data)/sizeof(gps_data[0]); i++)
		{
			if (gps_data[i] == '$')
			{
				uint8_t p=1;
				gps_pack[0]='$';
				i++;
				while(i<sizeof(gps_data)/sizeof(gps_data[0]))
				{
					if (gps_data[i]==0x0D)
					{
						Check_GPS_Pack();
						break;
					}
					gps_pack[p++]=gps_data[i++];
				}
			}

		}

		if (gps_info.fix_valid)
			LED_R_ON;
		else
			gps_info.time_pack.sec & 0x01?LED_R_ON:LED_R_OFF;

  }
  /* USER CODE END GPS_Task */
}

/* USER CODE BEGIN Header_GSM_Task */
/**
* @brief Function implementing the GSM thread.
* @param argument: Not used
* @retval None
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     if (huart == &huart3)
     {
	 gsm_data[p_w++] = u3;
	 if (p_w > sizeof(gsm_data)/sizeof(gsm_data[0]))
		 p_w = 0;
	 HAL_UART_Receive_IT(&huart3,&u3,1);
     }
}

uint8_t error;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
    	error++;
    	HAL_UART_Receive_IT(&huart3,&u3,1);
    }
}

/* USER CODE END Header_GSM_Task */
void GSM_Task(void *argument)
{
  /* USER CODE BEGIN GSM_Task */
  /* Infinite loop */

	HAL_UART_Receive_IT(&huart3, &u3, 1);
  for(;;)
  {
	  gsm();
	  vTaskDelay(10);
	  if (signal_level)
		  LED_G_ON;
	  else
		 p_w & 0x10?LED_G_ON:LED_G_OFF;
  }
  /* USER CODE END GSM_Task */
}

/* USER CODE BEGIN Header_LIS_Task */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c2)
  {
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LIS2DH12_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c2)
  {
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LIS2DH12_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  }
  return 0;
}
/**
* @brief Function implementing the LIS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LIS_Task */
void LIS_Task(void *argument)
{
  /* USER CODE BEGIN LIS_Task */
	  lis2dh12_ctx_t dev_ctx;

	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.handle = &hi2c2;

	  uint8_t state=0;
	  HAL_GPIO_WritePin(GPIOD, ACC_EN_Pin, GPIO_PIN_SET);
	  /*
	   *  Check device ID
	   */
	  /* Infinite loop */
	  for(;;)
	  {
		  vTaskDelay(50);
	  switch (state)
	  {
	  case 0:
		  lis2dh12_device_id_get(&dev_ctx, &whoamI);
		  if (whoamI == LIS2DH12_ID)
			  state = 1;
		  break;
	  case 1:

		/*
		 *  Enable Block Data Update
		 */
		lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

		/*
		 * Set Output Data Rate to 100Hz
		 */
		lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_100Hz);

		/*
		 * Set full scale to 2g
		 */
		lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);

		/*
		 * Enable temperature sensor
		 */
		lis2dh12_temperature_meas_set(&dev_ctx, LIS2DH12_TEMP_ENABLE);

		/*
		 * Set device in continuous mode with 12 bit resol.
		 */
		lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);
	    state = 2;
	    break;

	  case 2:
		/*
		 * Read samples in polling mode (no int)
		 */

		  lis2dh12_reg_t reg;

		  /*
		   * Read output only if new value available
		   */
		  lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);
		  if (reg.byte)
		  {
			/* Read accelerometer data */
			memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
			lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
			acceleration_mg[0] = lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[0]);
			acceleration_mg[1] = lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[1]);
			acceleration_mg[2] = lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[2]);

		  }

		  lis2dh12_temp_data_ready_get(&dev_ctx, &reg.byte);
		  if (reg.byte)
		  {
			/* Read temperature data */
			memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
			lis2dh12_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
			temperature_degC =
			  lis2dh12_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);

		  }

		break;
	  }
	  if (acceleration_mg[0]*acceleration_mg[1]*acceleration_mg[2]<0 )
	    LED_B_ON;
	  else
	    LED_B_OFF;
  }
  /* USER CODE END LIS_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
