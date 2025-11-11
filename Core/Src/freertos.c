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
#include "mcp3208.h"
#include "utils.h"
#include "fatfs.h"

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
uint32_t samplingPeriod_ticks;
osTimerId_t sampling_timer;
osEventFlagsId_t sampling_event;
int driver_id;
int direction;
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
void samplingTimer_Callback (void *argument);
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
	samplingPeriod_ticks = (tick_frequency/sampling_freq_ign);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	sampling_timer = osTimerNew(samplingTimer_Callback, osTimerPeriodic,1U, NULL);
	osTimerStart(sampling_timer, samplingPeriod_ticks);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	sensorData_queue = osMessageQueueNew(QUEUE_DEPTH, sizeof(float*), NULL);
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
  sampling_event = osEventFlagsNew(NULL);
  if (sampling_event == NULL){
	  while(1);
  }
  command_event = osEventFlagsNew(NULL);
    if (command_event == NULL){
  	  while(1);
    }
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
  float sensor_readings_A[sensor_count];
  float sensor_readings_B[sensor_count];
  //alternate between the two bufers so you can continue sampling while data is being written to SD
  float *buffer_ptr = sensor_readings_A;
  /* Infinite loop */

  for(;;)
  {
	osEventFlagsWait(sampling_event, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);
	for (int i = 0; i < sensor_count;i++){
		uint16_t adc_val =  MCP3208_GetAdcVal(sensor_list[i].channel, sensor_list[i].cs, &hspi3);
		float adc_voltage = ((float)adc_val *4.7)/4096.0;
		*(buffer_ptr+i) = adc_voltage*sensor_list[i].calibration_slope + sensor_list[i].calibration_intercept;
	}
	queue_status = osMessageQueuePut(sensorData_queue, &buffer_ptr, 0U,0U);
	if (queue_status != osOK){
		sprintf(TxBuffer, "unable to add to queue. Error: %d\n", queue_status);
		//skip this sample
		continue;
	}
	//swap buffers
	else{
		buffer_ptr = (buffer_ptr == sensor_readings_A) ? sensor_readings_B : sensor_readings_A;
	}
    //osDelay(samplingPeriod_ticks);
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
	//get command from user
	  osEventFlagsWait(command_event, FLAGS_MSK2, osFlagsWaitAny, osWaitForever);
	  parse_command(cmd_buffer, &driver_id, &direction);
	  HAL_GPIO_WritePin(driver_list[driver_id].GPIO_Port, driver_list[driver_id].GPIO_Pin, direction);

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
  float *msg_ptr; // pointer to data from queue
  FIL logFile; // file obj
  FRESULT fr; // result code
  UINT numBytes; // # of bytes actually written
  float *copy_msg_ptr;

  float current_buffer[sensor_count];

  int websocket_interval = 500; // 500 ms, edit actual value?
  uint32_t system_time; // real-time
  int last_websocket_time = 0;

  fr = f_open(&logFile, "datalog.txt", FA_WRITE | FA_OPEN_APPEND);
  if (fr != FR_OK) {
    osThreadTerminate(NULL); // quit if the file open fails
  }

  /* Infinite loop */
  for(;;)
  {
    queue_status = osMessageQueueGet(sensorData_queue, &msg_ptr, 0U, osWaitForever);

    if (queue_status == osOK) {
      memcpy(current_buffer, msg_ptr, sensor_count * sizeof(float));

      fr = f_write(&logFile, current_buffer, sensor_count * sizeof(float), &numBytes); // Write Data from the current buffer to the SD Card
      if (fr == FR_OK) {
        f_sync(&logFile);
      }

      system_time = osKernelGetTickCount();
      if (system_time - last_websocket_time > websocket_interval) { // websocket
          // Send msg_ptr over Websocket: websocket_send_function(current_buffer)
          last_websocket_time = system_time;
      }
    }
  }
  /* USER CODE END StartDataSending */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*Callback just notifies the data reading thread when it can take the next sample */
void samplingTimer_Callback (void *argument){
	osEventFlagsSet(sampling_event, FLAGS_MSK1);
}
/* USER CODE END Application */

