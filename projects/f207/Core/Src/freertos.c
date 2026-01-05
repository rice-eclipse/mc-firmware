/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "interface.h"
#include "tim.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRST_BUF_READY (1 << 0)
#define SECOND_BUF_READY (1 << 1)
#define SAMPLE_NOW (1 << 2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float sensor_vals[2*MAX_SENSOR_COUNT];
char tx_buffer[300];
/* USER CODE END Variables */
/* Definitions for collectionTask */
osThreadId_t collectionTaskHandle;
const osThreadAttr_t collectionTask_attributes = {
  .name = "collectionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for processingTask */
osThreadId_t processingTaskHandle;
const osThreadAttr_t processingTask_attributes = {
  .name = "processingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCollectionTask(void *argument);
void StartProcessingTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of collectionTask */
  collectionTaskHandle = osThreadNew(StartCollectionTask, NULL, &collectionTask_attributes);

  /* creation of processingTask */
  processingTaskHandle = osThreadNew(StartProcessingTask, NULL, &processingTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCollectionTask */
/**
  * @brief  Waits for the sampling event to take data from the ADCs
  * 		Notifies the processing task when any of the double buffers
  * 		are filled
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCollectionTask */
void StartCollectionTask(void *argument)
{
  /* USER CODE BEGIN StartCollectionTask */
	 HAL_TIM_Base_Start_IT(&htim14);
	int sample_count = 0;
	uint32_t sampling_flag;
  /* Infinite loop */
  for(;;)
  {
	  sampling_flag = osThreadFlagsWait(SAMPLE_NOW, osFlagsWaitAny, osWaitForever);
	 sensor_vals[sample_count] = get_sensorval_interface(&sensor_list[sample_count%sensor_count]);
	 sample_count = (sample_count + 1) % (sensor_count*2);

	 //First Buffer has been filled
	 if (sample_count == sensor_count){
		 osThreadFlagsSet(processingTaskHandle, FIRST_BUF_READY);
	 }
	 else if (sample_count == 0){
		 osThreadFlagsSet(processingTaskHandle, SECOND_BUF_READY);
	 }
	 osDelay(100);
  }
  /* USER CODE END StartCollectionTask */
}

/* USER CODE BEGIN Header_StartProcessingTask */
/**
* @brief Function implementing the processingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessingTask */
void StartProcessingTask(void *argument)
{
  /* USER CODE BEGIN StartProcessingTask */
	uint32_t processing_flag;
	float *current_buffer;
  /* Infinite loop */
  for(;;)
  {
	  processing_flag = osThreadFlagsWait((FIRST_BUF_READY | SECOND_BUF_READY),
			  	  	  	  	  	  	  	  osFlagsWaitAny, osWaitForever);
	  //set the current buf pointer to the first part of the data buffer
	  if (processing_flag & FIRST_BUF_READY){
		  current_buffer = &sensor_vals[0];
	  }
	  else if (processing_flag & SECOND_BUF_READY){
		  current_buffer = &sensor_vals[sensor_count];
	  }
	  //performs the filtering and decimation of the data
	  filter_and_decimate_interface(current_buffer, sensor_count);
	  //writes the data to SD card
	  for (int i = 0; i < sensor_count; i++){
		  sprintf(tx_buffer, "Sensor %s value: %f\r\n", sensor_list[i].name,
				  	  	  	  	  	  	  	  	  	   current_buffer[i]);
		  HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), 200);
	  }
	  osDelay(100);
  }
  /* USER CODE END StartProcessingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM14){
	  /*sampling event has occured*/
	  osThreadFlagsSet(collectionTaskHandle, SAMPLE_NOW);
	  HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
  }

  /* USER CODE END Callback 1 */
}


/* USER CODE END Application */

