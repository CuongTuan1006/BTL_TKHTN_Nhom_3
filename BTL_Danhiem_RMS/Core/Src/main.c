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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "i2c-lcd.h"
#include <stdio.h>
#include "ccs811.h"
#include <stdlib.h>
#include <string.h>
#include "warning.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define QUEUE_SIZE 9
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId TVOC_meaHandle;
osThreadId CO2_meaHandle;
osThreadId CO_meaHandle;
osThreadId Task_ISRHandle;
osThreadId Uart_SendHandle;
osThreadId LCD_TVOCHandle;
osThreadId LCD_COHandle;
osThreadId LCD_CO2Handle;
osThreadId WarningHandle;
osMutexId MutexMeasureHandle;
osMutexId MutexLCDHandle;
osSemaphoreId BinarySem_ISRHandle;
/* USER CODE BEGIN PV */
typedef struct {
	char unit[4];
    char sensorName[10];  // Tên cảm biến
    uint16_t value;          // Giá trị đo
} SensorData;

SensorData CO = {
        .unit = "ppm",
        .sensorName = "CO",
        .value = 0
};
SensorData CO2 = {
        .unit = "ppm",
        .sensorName = "CO2",
        .value = 0
};
SensorData TVOC = {
        .unit = "ppb",
        .sensorName = "TVOC",
        .value = 0
};


uint16_t Tvoc ;
uint16_t Tvoc_ppb;
uint32_t CO_adc_val;
uint16_t eCO2 ;
uint16_t CO2_ppm;
float CO_vol;
uint16_t CO_ppm;
char Lcd_buf[16];
uint8_t Uart_TX_Buf[40];
uint8_t Uart_RX_Buf[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void StartTVOC_mea(void const * argument);
void StartCO2_mea(void const * argument);
void StartCO_mea(void const * argument);
void StartTask_ISR(void const * argument);
void StartUart_Send(void const * argument);
void StartLCD_TVOC(void const * argument);
void StartLCD_CO(void const * argument);
void StartLCD_CO2(void const * argument);
void StartWarning(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int CO_ppm_convert(float CO_vol) {
    float a = ((5.0 - CO_vol) / CO_vol) * (10/10);
    return (int)round(100 * pow(a, -1.559));
}
void CO_measure()
{
	CO_adc_val = HAL_ADC_GetValue(&hadc1);
	CO_vol = ((float)CO_adc_val / 4095.0) * 5;
	CO_ppm = (uint16_t)(CO_ppm_convert(CO_vol));
}
void TVOC_measure()
{
	CCS811_Read_TVOC(&Tvoc);
	Tvoc_ppb=Tvoc;
}
void CO2_measure()
{
	CCS811_Read_Co2(&eCO2);
	CO2_ppm=eCO2;
}
void LCD_Display(SensorData data)
{
	lcd_clear();
	lcd_put_cur(0,0);
	snprintf(Lcd_buf, sizeof(Lcd_buf), "%s:", data.sensorName);
	lcd_send_string (Lcd_buf);
	lcd_put_cur(1,0);
	snprintf(Lcd_buf, sizeof(Lcd_buf), "%d %s", (int)data.value, data.unit);
	lcd_send_string (Lcd_buf);
}

void Uart_Send()
{
	sprintf((char*)Uart_TX_Buf, "TVOC:%d ppb\r\n",(int)Tvoc_ppb);
	HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
	sprintf((char*)Uart_TX_Buf, "CO:%d ppm\r\n",(int)CO_ppm);
	HAL_UART_Transmit(&huart1, (uint8_t*)Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
	sprintf((char*)Uart_TX_Buf, "CO2:%d ppm\r\n",(int)CO2_ppm);
	HAL_UART_Transmit(&huart1,(uint8_t*)Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
	sprintf((char*)Uart_TX_Buf, "--------\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)Uart_TX_Buf, strlen((char*)Uart_TX_Buf), HAL_MAX_DELAY);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1)
    {
    	osSemaphoreRelease(BinarySem_ISRHandle);
    }
    HAL_UART_Receive_IT(&huart1, Uart_RX_Buf, 1);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CCS811_Init();
  CCS811_Reset();
  CCS811_Set_Mode(CCS811_MODE_1SEC);
  CCS811_EnableInt();
  HAL_ADC_Start(&hadc1);
  lcd_init();
  HAL_UART_Receive_IT(&huart1, Uart_RX_Buf, 1);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MutexMeasure */
  osMutexDef(MutexMeasure);
  MutexMeasureHandle = osMutexCreate(osMutex(MutexMeasure));

  /* definition and creation of MutexLCD */
  osMutexDef(MutexLCD);
  MutexLCDHandle = osMutexCreate(osMutex(MutexLCD));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem_ISR */
  osSemaphoreDef(BinarySem_ISR);
  BinarySem_ISRHandle = osSemaphoreCreate(osSemaphore(BinarySem_ISR), 1);

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
  /* definition and creation of TVOC_mea */
  osThreadDef(TVOC_mea, StartTVOC_mea, osPriorityAboveNormal, 0, 128);
  TVOC_meaHandle = osThreadCreate(osThread(TVOC_mea), NULL);

  /* definition and creation of CO2_mea */
  osThreadDef(CO2_mea, StartCO2_mea, osPriorityAboveNormal, 0, 128);
  CO2_meaHandle = osThreadCreate(osThread(CO2_mea), NULL);

  /* definition and creation of CO_mea */
  osThreadDef(CO_mea, StartCO_mea, osPriorityHigh, 0, 128);
  CO_meaHandle = osThreadCreate(osThread(CO_mea), NULL);

  /* definition and creation of Task_ISR */
  osThreadDef(Task_ISR, StartTask_ISR, osPriorityRealtime, 0, 128);
  Task_ISRHandle = osThreadCreate(osThread(Task_ISR), NULL);

  /* definition and creation of Uart_Send */
  osThreadDef(Uart_Send, StartUart_Send, osPriorityAboveNormal, 0, 128);
  Uart_SendHandle = osThreadCreate(osThread(Uart_Send), NULL);

  /* definition and creation of LCD_TVOC */
  osThreadDef(LCD_TVOC, StartLCD_TVOC, osPriorityNormal, 0, 128);
  LCD_TVOCHandle = osThreadCreate(osThread(LCD_TVOC), NULL);

  /* definition and creation of LCD_CO */
  osThreadDef(LCD_CO, StartLCD_CO, osPriorityAboveNormal, 0, 128);
  LCD_COHandle = osThreadCreate(osThread(LCD_CO), NULL);

  /* definition and creation of LCD_CO2 */
  osThreadDef(LCD_CO2, StartLCD_CO2, osPriorityAboveNormal, 0, 128);
  LCD_CO2Handle = osThreadCreate(osThread(LCD_CO2), NULL);

  /* definition and creation of Warning */
  osThreadDef(Warning, StartWarning, osPriorityRealtime, 0, 128);
  WarningHandle = osThreadCreate(osThread(Warning), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L1_LED_Pin|L2_LED_Pin|L3_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : L1_LED_Pin L2_LED_Pin L3_LED_Pin */
  GPIO_InitStruct.Pin = L1_LED_Pin|L2_LED_Pin|L3_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTVOC_mea */
/**
  * @brief  Function implementing the TVOC_mea thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTVOC_mea */
void StartTVOC_mea(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	if (osMutexWait(MutexMeasureHandle, osWaitForever) == osOK){
		TVOC_measure();
		TVOC.value = Tvoc_ppb;

		osMutexRelease(MutexMeasureHandle);
	}
    osDelay(3995);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCO2_mea */
/**
* @brief Function implementing the CO2_mea thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCO2_mea */
void StartCO2_mea(void const * argument)
{
  /* USER CODE BEGIN StartCO2_mea */
  /* Infinite loop */
  for(;;)
  {
	if (osMutexWait(MutexMeasureHandle, osWaitForever) == osOK){
	  CO2_measure();
	  CO2.value = CO2_ppm;
	  osMutexRelease(MutexMeasureHandle);
	}
	osDelay(2995);
  }
  /* USER CODE END StartCO2_mea */
}

/* USER CODE BEGIN Header_StartCO_mea */
/**
* @brief Function implementing the CO_mea thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCO_mea */
void StartCO_mea(void const * argument)
{
  /* USER CODE BEGIN StartCO_mea */
  /* Infinite loop */
  for(;;)
  {
	if (osMutexWait(MutexMeasureHandle, osWaitForever) == osOK){
		CO_measure();
		CO.value = CO_ppm;
		osMutexRelease(MutexMeasureHandle);
	}
    osDelay(1995);
  }
  /* USER CODE END StartCO_mea */
}

/* USER CODE BEGIN Header_StartTask_ISR */
/**
* @brief Function implementing the Task_ISR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ISR */
void StartTask_ISR(void const * argument)
{
  /* USER CODE BEGIN StartTask_ISR */
  /* Infinite loop */
	for(;;)
	  {
		 osSemaphoreWait(BinarySem_ISRHandle,osWaitForever);
		 if((char)Uart_RX_Buf[0] == '3')
		 {
			TVOC_measure();
			sprintf((char*)Uart_TX_Buf, "TVOC:%d ppb\r\n",(int)Tvoc_ppb);
			HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
		 }
		 else if((char)Uart_RX_Buf[0]  == '2')
		 {
			CO2_measure();
			sprintf((char*)Uart_TX_Buf, "CO2:%d ppm\r\n",(int)CO2_ppm);
			HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
		 }
		 else if((char)Uart_RX_Buf[0]  == '1')
		 {
			CO_measure();
			sprintf((char*)Uart_TX_Buf, "CO:%d ppm\r\n",(int)CO_ppm);
			HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
		 }
		 osDelay(1000);
	  }
  /* USER CODE END StartTask_ISR */
}

/* USER CODE BEGIN Header_StartUart_Send */
/**
* @brief Function implementing the Uart_Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart_Send */
void StartUart_Send(void const * argument)
{
  /* USER CODE BEGIN StartUart_Send */
  /* Infinite loop */
  for(;;)
  {
	Uart_Send();
    osDelay(3000);
  }
  /* USER CODE END StartUart_Send */
}

/* USER CODE BEGIN Header_StartLCD_TVOC */
/**
* @brief Function implementing the LCD_TVOC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD_TVOC */
void StartLCD_TVOC(void const * argument)
{
  /* USER CODE BEGIN StartLCD_TVOC */
  /* Infinite loop */
  for(;;)
  {

	if (osMutexWait(MutexLCDHandle, osWaitForever) == osOK){
		LCD_Display(TVOC);
		HAL_Delay(900);

		osMutexRelease(MutexLCDHandle);
	}
	osDelay(2000);
  }
  /* USER CODE END StartLCD_TVOC */
}

/* USER CODE BEGIN Header_StartLCD_CO */
/**
* @brief Function implementing the LCD_CO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD_CO */
void StartLCD_CO(void const * argument)
{
  /* USER CODE BEGIN StartLCD_CO */
  /* Infinite loop */
  for(;;)
  {
	  if (osMutexWait(MutexLCDHandle, osWaitForever) == osOK){
		LCD_Display(CO);
	  	HAL_Delay(900);
	  	osMutexRelease(MutexLCDHandle);
	  }
	  osDelay(2000);
  }
  /* USER CODE END StartLCD_CO */
}

/* USER CODE BEGIN Header_StartLCD_CO2 */
/**
* @brief Function implementing the LCD_CO2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD_CO2 */
void StartLCD_CO2(void const * argument)
{
  /* USER CODE BEGIN StartLCD_CO2 */
  /* Infinite loop */
  for(;;)
  {
	if (osMutexWait(MutexLCDHandle, osWaitForever) == osOK){
	  	LCD_Display(CO2);
	  	HAL_Delay(900);
	  	osMutexRelease(MutexLCDHandle);
	}
	osDelay(2000);
  }
  /* USER CODE END StartLCD_CO2 */
}

/* USER CODE BEGIN Header_StartWarning */
/**
* @brief Function implementing the Warning thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWarning */
void StartWarning(void const * argument)
{
  /* USER CODE BEGIN StartWarning */
  /* Infinite loop */
  for(;;)
  {
	  Warning();
	  osDelay(1000);
  }
  /* USER CODE END StartWarning */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
