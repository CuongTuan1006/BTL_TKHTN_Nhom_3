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
#define MAX_TASK 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId CO_meaHandle;
osThreadId WarningHandle;
osThreadId LCDHandle;
osThreadId CO2_meaHandle;
osThreadId TVOC_meaHandle;
osThreadId UART_SendHandle;
osThreadId Task_ISRHandle;
osMessageQId LCD_QueueHandle;
osSemaphoreId BinarySem_ISRHandle;
/* USER CODE BEGIN PV */
typedef struct {
	char unit[4];
    char sensorName[10];  // Tên cảm biến
    uint16_t value;          // Giá trị đo
} SensorData;

typedef struct {
    const char *name;
    uint32_t execution_time;
    uint32_t deadline;
    uint32_t period;
} TaskConfig;

TaskConfig tasks[] = {
    {"StartCO_mea", 10, 100, 3000},
    {"StartCO2_mea", 10, 1100, 3000},
    {"StartTVOC_mea", 10, 2100, 3000},
    {"StartWarning", 5, 150, 1000},
	{"StartLCD_1", 60, 300, 1000},
    {"StartUart_Send", 10, 2400, 3000},
};
osThreadId task_ids[MAX_TASK];
uint32_t deadlines[MAX_TASK]={100,1100,2100,150,300,2400};

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
void StartCO_mea(void const * argument);
void StartWarning(void const * argument);
void StartLCD(void const * argument);
void StartCO2_mea(void const * argument);
void StartTVOC_mea(void const * argument);
void StartUART_Send(void const * argument);
void StartTask_ISR(void const * argument);

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
	sprintf((char*)Uart_TX_Buf, "CO:%d ppm\r\n",(int)CO_ppm);
	HAL_UART_Transmit(&huart1, (uint8_t*)Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
	sprintf((char*)Uart_TX_Buf, "CO2:%d ppm\r\n",(int)CO2_ppm);
	HAL_UART_Transmit(&huart1,(uint8_t*)Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
	sprintf((char*)Uart_TX_Buf, "TVOC:%d ppb\r\n",(int)Tvoc_ppb);
	HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
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
void EDFScheduler();
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

  /* Create the queue(s) */
  /* definition and creation of LCD_Queue */
  osMessageQDef(LCD_Queue, 16, SensorData);
  LCD_QueueHandle = osMessageCreate(osMessageQ(LCD_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CO_mea */
  osThreadDef(CO_mea, StartCO_mea, osPriorityNormal, 0, 128);
  CO_meaHandle = osThreadCreate(osThread(CO_mea), NULL);

  /* definition and creation of Warning */
  osThreadDef(Warning, StartWarning, osPriorityIdle, 0, 128);
  WarningHandle = osThreadCreate(osThread(Warning), NULL);

  /* definition and creation of LCD */
  osThreadDef(LCD, StartLCD, osPriorityIdle, 0, 128);
  LCDHandle = osThreadCreate(osThread(LCD), NULL);

  /* definition and creation of CO2_mea */
  osThreadDef(CO2_mea, StartCO2_mea, osPriorityIdle, 0, 128);
  CO2_meaHandle = osThreadCreate(osThread(CO2_mea), NULL);

  /* definition and creation of TVOC_mea */
  osThreadDef(TVOC_mea, StartTVOC_mea, osPriorityIdle, 0, 128);
  TVOC_meaHandle = osThreadCreate(osThread(TVOC_mea), NULL);

  /* definition and creation of UART_Send */
  osThreadDef(UART_Send, StartUART_Send, osPriorityIdle, 0, 128);
  UART_SendHandle = osThreadCreate(osThread(UART_Send), NULL);

  /* definition and creation of Task_ISR */
  osThreadDef(Task_ISR, StartTask_ISR, osPriorityRealtime, 0, 128);
  Task_ISRHandle = osThreadCreate(osThread(Task_ISR), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  task_ids[0] = CO_meaHandle;
  task_ids[1] = CO2_meaHandle;
  task_ids[2] = TVOC_meaHandle;
  task_ids[3] = WarningHandle;
  task_ids[4] = LCDHandle;
  task_ids[5] = UART_SendHandle;

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	EDFScheduler();
	osDelay(100);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CCS_RST_GPIO_Port, CCS_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L1_LED_Pin|L2_LED_Pin|L3_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CCS_RST_Pin */
  GPIO_InitStruct.Pin = CCS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CCS_RST_GPIO_Port, &GPIO_InitStruct);

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
void EDFScheduler() {
    uint32_t current_time = osKernelSysTick(); // Lấy th�?i gian hiện tại

    // Cập nhật deadline nếu đã quá hạn
    for (int i = 0; i < MAX_TASK; i++) {
        if (current_time >= deadlines[i]) {
            deadlines[i] += tasks[i].period; // Reset deadline
        }
    }

    // Sắp xếp các task theo deadline gần nhất (Bubble Sort)
    for (int i = 0; i < MAX_TASK-1; i++) {
        for (int j = i + 1; j < MAX_TASK; j++) {
            if (deadlines[i] > deadlines[j]) {
                uint32_t temp = deadlines[i];
                deadlines[i] = deadlines[j];
                deadlines[j] = temp;

                TaskConfig temp_task = tasks[i];
                tasks[i] = tasks[j];
                tasks[j] = temp_task;
            }
        }
    }
    // Cập nhật độ ưu tiên
    for (int i = 0; i < MAX_TASK; i++) {
        osPriority priority = osPriorityIdle + (osPriorityRealtime - i-1);
        osThreadSetPriority(task_ids[i], priority);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCO_mea */
/**
* @brief Function implementing the CO_mea thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCO_mea */
void StartCO_mea(void const * argument)
{
  /* USER CODE BEGIN 5 */
	SensorData data3;
	snprintf(data3.sensorName, sizeof(data3.sensorName), "CO");
	snprintf(data3.unit, sizeof(data3.unit), "ppm");
	uint32_t task_index = 0;
	uint32_t execution_time = tasks[task_index].execution_time;  // Th ?i gian thực thi của task
	uint32_t period = tasks[task_index].period;  // Chu kỳ của task
  /* Infinite loop */
  for(;;)
  {
	  CO_measure();
	  data3.value = CO_ppm;
	  osMessagePut(LCD_QueueHandle, (uint32_t)(uintptr_t)&data3, osWaitForever);
	  osDelay(period - execution_time);
  }
  /* USER CODE END 5 */
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
	uint32_t task_index = 3;
	uint32_t execution_time = tasks[task_index].execution_time;  // Th ?i gian thực thi của task
	uint32_t period = tasks[task_index].period;  // Chu kỳ của task
  /* Infinite loop */
  for(;;)
  {
	Warning();
    osDelay(period-execution_time);
  }
  /* USER CODE END StartWarning */
}

/* USER CODE BEGIN Header_StartLCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD */
void StartLCD(void const * argument)
{
  /* USER CODE BEGIN StartLCD */
	osEvent evt;
	SensorData receivedData;
	uint32_t task_index = 4;
	uint32_t execution_time = tasks[task_index].execution_time;  // Th ?i gian thực thi của task
	uint32_t period = tasks[task_index].period;  // Chu kỳ của task
  /* Infinite loop */
  for(;;)
  {
	evt = osMessageGet(LCD_QueueHandle, osWaitForever);
	if (evt.status == osEventMessage)
	{
	   receivedData = *(SensorData *)evt.value.p;
	   LCD_Display(receivedData);
	}
    osDelay(period-execution_time);
  }
  /* USER CODE END StartLCD */
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
	SensorData data2;
	snprintf(data2.sensorName, sizeof(data2.sensorName), "CO2");
	snprintf(data2.unit, sizeof(data2.unit), "ppm");
	uint32_t task_index = 1;
	uint32_t execution_time = tasks[task_index].execution_time;  // Th ?i gian thực thi của task
	uint32_t period = tasks[task_index].period;  // Chu kỳ của task
  /* Infinite loop */
  for(;;)
  {
	CO2_measure();
	data2.value = CO2_ppm;
	osMessagePut(LCD_QueueHandle, (uint32_t)(uintptr_t)&data2, osWaitForever);
	osDelay(period - execution_time);
  }
  /* USER CODE END StartCO2_mea */
}

/* USER CODE BEGIN Header_StartTVOC_mea */
/**
  * @brief  Function implementing the TVOC_mea thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTVOC_mea */
void StartTVOC_mea(void const * argument)
{
  /* USER CODE BEGIN StartTVOC_mea */
	SensorData data1;
	snprintf(data1.sensorName, sizeof(data1.sensorName), "TVOC");
	snprintf(data1.unit, sizeof(data1.unit), "ppb");
	uint32_t task_index = 2;
	uint32_t execution_time = tasks[task_index].execution_time;  // Th ?i gian thực thi của task
	uint32_t period = tasks[task_index].period;  // Chu kỳ của task
  /* Infinite loop */
  for(;;)
  {
	  TVOC_measure();
	  data1.value = Tvoc_ppb;
	  osMessagePut(LCD_QueueHandle, (uint32_t)(uintptr_t)&data1, osWaitForever);
	  osDelay(period - execution_time);
  }
  /* USER CODE END StartTVOC_mea */
}

/* USER CODE BEGIN Header_StartUART_Send */
/**
* @brief Function implementing the UART_Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_Send */
void StartUART_Send(void const * argument)
{
  /* USER CODE BEGIN StartUART_Send */
	uint32_t task_index = 5;
	uint32_t execution_time = tasks[task_index].execution_time;  // Th ?i gian thực thi của task
	uint32_t period = tasks[task_index].period;  // Chu kỳ của task
  /* Infinite loop */
  for(;;)
  {
	Uart_Send();
    osDelay(period-execution_time);
  }
  /* USER CODE END StartUART_Send */
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
			sprintf((char*)Uart_TX_Buf, "--------\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)Uart_TX_Buf, strlen((char*)Uart_TX_Buf), HAL_MAX_DELAY);
		 }
		 else if((char)Uart_RX_Buf[0]  == '2')
		 {
			CO2_measure();
			sprintf((char*)Uart_TX_Buf, "CO2:%d ppm\r\n",(int)CO2_ppm);
			HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
			sprintf((char*)Uart_TX_Buf, "--------\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)Uart_TX_Buf, strlen((char*)Uart_TX_Buf), HAL_MAX_DELAY);
		 }
		 else if((char)Uart_RX_Buf[0]  == '1')
		 {
			CO_measure();
			sprintf((char*)Uart_TX_Buf, "CO:%d ppm\r\n",(int)CO_ppm);
			HAL_UART_Transmit(&huart1,(uint8_t*) Uart_TX_Buf,strlen((char*)Uart_TX_Buf) , HAL_MAX_DELAY);
			sprintf((char*)Uart_TX_Buf, "--------\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)Uart_TX_Buf, strlen((char*)Uart_TX_Buf), HAL_MAX_DELAY);
		 }
		 osDelay(1000);
	  }

  /* USER CODE END StartTask_ISR */
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
