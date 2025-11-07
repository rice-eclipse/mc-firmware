/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_DEPTH 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char TxBuffer[300];
osMessageQueueId_t sensorData_queue;
osStatus_t queue_status;
int sensor_count;
uint32_t = samplingPeriod_ticks;
/* USER CODE END Variables */
/* Definitions for dataReading */
osThreadId_t dataReadingHandle;
const osThreadAttr_t dataReading_attributes = {
  .name = "dataReading",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for cmdHandling */
osThreadId_t cmdHandlingHandle;
const osThreadAttr_t cmdHandling_attributes = {
  .name = "cmdHandling",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for dataSending */
osThreadId_t dataSendingHandle;
const osThreadAttr_t dataSending_attributes = {
  .name = "dataSending",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDataReading(void *argument);
void StartCmdHandling(void *argument);
void StartDataSending(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	sensor_count = sizeof(sensor_list)/sizeof(sensor_list[0]);
	uint32_t tick_frequency = osKernelGetTickFreq();
	samplingPeriod_ticks = (tick_frequency/sampling_freq_ign) / 1000;
  /* USER CODE END Init */

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
	sensorData_queue = osMessageQueueNew(QUEUE_DEPTH, sensor_count*sizeof(float), NULL);
	if (sensorData_queue == NULL){
		while(1);

	}
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of dataReading */
  dataReadingHandle = osThreadNew(StartDataReading,NULL, &dataReading_attributes);

  /* creation of cmdHandling */
  cmdHandlingHandle = osThreadNew(StartCmdHandling, NULL, &cmdHandling_attributes);

  /* creation of dataSending */
  dataSendingHandle = osThreadNew(StartDataSending, NULL, &dataSending_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDataReading */
/**
  * @brief  Function implementing the dataReading thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDataReading */
void StartDataReading(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDataReading */
  float sensor_readings[sensor_count];
  /* Infinite loop */
  for(;;)
  {
	for (int i = 0; i < sensor_count;i++){
		sensor_readings[i] = read_adc(sensor_list[i].gpio_port, sensor_list[i].gpio_pin);
		sensor_readings[i] = sensor_readings[i]*sensor_list[i].calibration_slope + sensor_list[i].calibration_intercept;
	}
	queue_status = osMessageQueuePut(sensorData_queue, sensor_readings, 0U,0U);
	if (queue_status != osOK){
		sprintf(TxBuffer, "unable to add to queue. Error: %d\n", queue_status);
	}
    osDelay(samplingPeriod_ticks);
  }
  /* USER CODE END StartDataReading */
}

/* USER CODE BEGIN Header_StartCmdHandling */
/**
* @brief Function implementing the cmdHandling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdHandling */
void StartCmdHandling(void *argument)
{
  /* USER CODE BEGIN StartCmdHandling */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCmdHandling */
}

/* USER CODE BEGIN Header_StartDataSending */
/**
* @brief Function implementing the dataSending thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataSending */
void StartDataSending(void *argument)
{
  /* USER CODE BEGIN StartDataSending */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDataSending */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

