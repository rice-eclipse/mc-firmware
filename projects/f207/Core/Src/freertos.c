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
#include "cmsis_os.h"
#include "fatfs.h"
#include "sdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRST_BUF_READY (1 << 0)
#define SECOND_BUF_READY (1 << 1)
#define SAMPLE_NOW (1 << 2)
#define SHUTDOWN (1 << 3)
#define LOGGING_STOPPED (1 << 4)
#define MSG_RECEIVED (1 << 5)
#define SENDING_BUF_READY (1 << 6)
#define SEND_NOW (1 << 7)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float sensor_vals[2*MAX_SENSOR_COUNT];
float vals_to_send[MAX_SENSOR_COUNT];
char tx_buffer[300];
//stores the sensor values in a single string to write to the sd card
char sdcard_data[4096];
char data_log[100];
char data_header_str[300];
char cmd_str[100];
FIL data_file;
FIL log_file;
FRESULT fres;
long tim14_tick_count;
int tim13_tick_count;
volatile int stop_logging;

/* USER CODE END Variables */
/* Definitions for collectionTask */
osThreadId_t collectionTaskHandle;
const osThreadAttr_t collectionTask_attributes = {
  .name = "collectionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for processingTask */
osThreadId_t processingTaskHandle;
const osThreadAttr_t processingTask_attributes = {
  .name = "processingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for shutdownTask */
osThreadId_t shutdownTaskHandle;
const osThreadAttr_t shutdownTask_attributes = {
  .name = "shutdownTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for cmdHandlingTask */
osThreadId_t cmdHandlingTaskHandle;
const osThreadAttr_t cmdHandlingTask_attributes = {
  .name = "cmdHandlingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for valsToSendMutex */
osMutexId_t valsToSendMutexHandle;
const osMutexAttr_t valsToSendMutex_attributes = {
  .name = "valsToSendMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void add_datafile_header();
/* USER CODE END FunctionPrototypes */

void StartCollectionTask(void *argument);
void StartProcessingTask(void *argument);
void startShutdownTask(void *argument);
void StartCmdHandlingTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	stop_logging = 0;
	tim13_tick_count = 0;
	create_file_interface(&data_file,"data.csv");
	create_file_interface(&log_file,"console.log");
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of valsToSendMutex */
  valsToSendMutexHandle = osMutexNew(&valsToSendMutex_attributes);

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

  /* creation of shutdownTask */
  shutdownTaskHandle = osThreadNew(startShutdownTask, NULL, &shutdownTask_attributes);

  /* creation of cmdHandlingTask */
  cmdHandlingTaskHandle = osThreadNew(StartCmdHandlingTask, NULL, &cmdHandlingTask_attributes);

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
	//used to determine which of the buffers have been filled when copying to the 'vals_to_send' buffer
	 float *curr_buffer;
	 HAL_TIM_Base_Start_IT(&htim14);
	 HAL_TIM_Base_Start_IT(&htim13);
	 HAL_TIM_Base_Start_IT(&htim11);
	int sample_count = 0;
	uint32_t sending_flag;
	osStatus_t mutex_status;
  /* Infinite loop */
  for(;;)
  {
	 osThreadFlagsWait(SAMPLE_NOW, osFlagsWaitAny, osWaitForever);
	 sensor_vals[sample_count] = get_sensorval_interface(&sensor_list[sample_count%sensor_count]);
	 sample_count = (sample_count + 1) % (sensor_count*2);

	 //First Buffer has been filled
	 if (sample_count == sensor_count){
		 osThreadFlagsSet(processingTaskHandle, FIRST_BUF_READY);
		 curr_buffer = &sensor_vals[0];
	 }
	 else if (sample_count == 0){
		 osThreadFlagsSet(processingTaskHandle, SECOND_BUF_READY);
		 curr_buffer = &sensor_vals[sensor_count];
	 }
	 //check if it is time to send the data to mission control
	 sending_flag = osThreadFlagsWait(SEND_NOW, osFlagsWaitAny, 0);
	 if (sending_flag & SEND_NOW){
		 mutex_status = osMutexAcquire(valsToSendMutexHandle, 0);
		 if (mutex_status == osOK)
			 memcpy(vals_to_send, curr_buffer, sensor_count);
		 osMutexRelease(valsToSendMutexHandle);
	 }
	 osDelay(1);
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
	int sd_card_pos = 0;
	int log_count = 0;
	float *current_buffer;
	open_file_interface(&data_file, "data.csv");
	add_datafile_header();
	open_file_interface(&log_file, "console.log");
	/*We sync the file to the sd card every second*/
	int sync_count = 0;
#ifndef TEST_LOGIC
	FRESULT fres;
#endif
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

	  if (stop_logging == 0){
		  //performs the filtering and decimation of the data
		  filter_and_decimate_interface(current_buffer, sensor_count);
		  sdcard_data[0] = '\0';
		  //writes the data to SD card
		  for (int i = 0; i < sensor_count; i++){

			  sprintf(tx_buffer, "Sensor %s value: %f\r\n", sensor_list[i].name,
																	   current_buffer[i]);
			   //HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), 200);

			  sd_card_pos += snprintf(sdcard_data + sd_card_pos, sizeof(sdcard_data)-sd_card_pos, "%.3f,",current_buffer[i]);
			  if (sd_card_pos < 0 || sd_card_pos >= sizeof(sdcard_data)){
				  sprintf(tx_buffer, "buffer full!\r\n");
				  HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
				  //drop the data in this cycle
				  sd_card_pos = 0;
				  break;
			  }
		  }

		  sd_card_pos += snprintf(sdcard_data + sd_card_pos, sizeof(sdcard_data)-sd_card_pos, "\r\n");
		  if (sd_card_pos < 0 || sd_card_pos >= sizeof(sdcard_data)){
				  sprintf(tx_buffer, "buffer full!\r\n");
				  HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
				  //drop the data in this cycle
				  sd_card_pos = 0;
			  }
		  log_count++;
		  if (log_count == 20){
			  sprintf(tx_buffer, "sd write\r\n");
			  sprintf(data_log,"sd write\r\n");
			  HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), 200);
			  log_count = 0;

#ifndef TEST_LOGIC
			  fres = append_file_interface(&data_file, sdcard_data,sd_card_pos);
			  fres = append_file_interface(&log_file, data_log, strlen(data_log));
#endif
			  sd_card_pos = 0;
			  memset(sdcard_data, 0, sizeof(sdcard_data));
		  }
		  /*TODO: include timestamps from RTC*/
		  sync_count = (sync_count + 1) % sampling_freq_ign;
		  if (sync_count == 0){
			  sprintf(tx_buffer, "synced data\r\n");
			  HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
		  }
#ifndef TEST_LOGIC
		  /*sync data to SD card every second*/
		  if (sync_count == 0){
			  f_sync(&data_file);
			  f_sync(&log_file);
		  }
	#endif
	  }
	  else{
		  osThreadFlagsSet(shutdownTaskHandle,LOGGING_STOPPED);
	  }
	  osDelay(1);
  }
  /* USER CODE END StartProcessingTask */
}

/* USER CODE BEGIN Header_startShutdownTask */
/**
* @brief stops logging & processing, and closes all the files when a shutdown condition is met
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startShutdownTask */
void startShutdownTask(void *argument)
{
  /* USER CODE BEGIN startShutdownTask */
	uint32_t shutdown_flag;
  /* Infinite loop */
  for(;;)
  {
	shutdown_flag = osThreadFlagsWait(SHUTDOWN, osFlagsWaitAny, osWaitForever);
	/*Tell processing task to stop*/
	stop_logging = 1;

	/*Waits for processing to stop*/
	shutdown_flag = osThreadFlagsWait(LOGGING_STOPPED, osFlagsWaitAny, osWaitForever);

	close_file_interface(&data_file);
	close_file_interface(&log_file);
	unmount_sd_interface();
	if (collectionTaskHandle != NULL){
		osThreadTerminate(collectionTaskHandle);
	}
	if (processingTaskHandle != NULL){
		osThreadTerminate(processingTaskHandle);
	}
	sprintf(tx_buffer, "All tasks closed\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
	osThreadExit();
    osDelay(1);
  }
  /* USER CODE END startShutdownTask */
}

/* USER CODE BEGIN Header_StartCmdHandlingTask */
/**
* @brief Function implementing the cmdHandlingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdHandlingTask */
void StartCmdHandlingTask(void *argument)
{
  /* USER CODE BEGIN StartCmdHandlingTask */
	uint32_t msg_received_flag;
	int ignition_flag;
	int shutdown_flag;
	int stop_ignition_flag;
	int driver_id;
	int direction;
  /* Infinite loop */
  for(;;)
  {
	msg_received_flag = osThreadFlagsWait(MSG_RECEIVED, osFlagsWaitAny, osWaitForever);
	parse_command_interface(cmd_str, &driver_id, &direction, driver_list, &ignition_flag,
			  	  	  	  	  	&shutdown_flag, &stop_ignition_flag);
	if (shutdown_flag == 1){
		osThreadFlagsSet(shutdownTaskHandle, SHUTDOWN);
	}
	else if (stop_ignition_flag == 1){
		HAL_GPIO_WritePin(ignition.GPIO_Port, ignition.GPIO_Pin, 0);
	}
	else if (ignition_flag == 1){
		ignition_sequence();
	}
    osDelay(1);
  }
  /* USER CODE END StartCmdHandlingTask */
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
  if (htim->Instance  == TIM11){
	  /*Data sending event has occured*/
	  osThreadFlagsSet(collectionTaskHandle, SEND_NOW);
	  HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
  }

  if (htim->Instance == TIM13){
	  /*shutdown condition has occured*/
	  if (tim13_tick_count == 1){
		  osThreadFlagsSet(shutdownTaskHandle,SHUTDOWN);
	  }
	  else{
		  tim13_tick_count = 1;
	  }
	  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
  }

  /* USER CODE END Callback 1 */
}

void add_datafile_header(){
	int card_pos = 0;
	data_header_str[0] = '\0';
	for (int i = 0; i < sensor_count; i++){
		card_pos += snprintf(data_header_str+card_pos,sizeof(data_header_str) - card_pos,"%s,",
							 sensor_list[i].name);
		if (card_pos <= 0 || card_pos > sizeof(data_header_str)){
			 sprintf(tx_buffer, "sensor list array corrupted!\r\n");
			 HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
		}
	}
	card_pos += snprintf(data_header_str+card_pos, sizeof(data_header_str)-card_pos, "\r\n");
	if (card_pos <= 0 || card_pos > sizeof(data_header_str)){
			 sprintf(tx_buffer, "sensor list array corrupted!\r\n");
			 HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);

		}
#ifndef TEST_LOGIC
	append_file_interface(&data_file, data_header_str, card_pos);
#endif
}


/* USER CODE END Application */

