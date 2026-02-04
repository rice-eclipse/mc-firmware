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
#include "lwip.h"
#include "mongoose.h"
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
char logging_str[100];
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for websocketTask */
osThreadId_t websocketTaskHandle;
const osThreadAttr_t websocketTask_attributes = {
  .name = "websocketTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for valsToSendMutex */
osMutexId_t valsToSendMutexHandle;
const osMutexAttr_t valsToSendMutex_attributes = {
  .name = "valsToSendMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void add_datafile_header();
static void fn(struct mg_connection *c, int ev, void *ev_data);
void mg_random(void *buf, size_t len);
/* USER CODE END FunctionPrototypes */

void StartCollectionTask(void *argument);
void StartWebsocketTask(void *argument);

extern void MX_LWIP_Init(void);
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

  /* creation of websocketTask */
  websocketTaskHandle = osThreadNew(StartWebsocketTask, NULL, &websocketTask_attributes);

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
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartCollectionTask */

  extern struct netif gnetif;
    while(ip4_addr_isany_val(*netif_ip4_addr(&gnetif)))
   	  osDelay(200); // CMSIS-RTOS v1 uses milliseconds
     MG_INFO(("READY, IP: %s", ip4addr_ntoa(netif_ip4_addr(&gnetif))));
     websocketTaskHandle = osThreadNew(StartWebsocketTask, NULL, &websocketTask_attributes);
     osThreadTerminate(collectionTaskHandle);
	//used to determine which of the buffers have been filled when copying to the 'vals_to_send' buffer
	 float *curr_buffer;
	 //HAL_TIM_Base_Start_IT(&htim14);
	 //HAL_TIM_Base_Start_IT(&htim13);
	 //HAL_TIM_Base_Start_IT(&htim11);
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
		 //osThreadFlagsSet(processingTaskHandle, FIRST_BUF_READY);
		 curr_buffer = &sensor_vals[0];
	 }
	 else if (sample_count == 0){
		 //osThreadFlagsSet(processingTaskHandle, SECOND_BUF_READY);
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

/* USER CODE BEGIN Header_StartWebsocketTask */
/**
* @brief Function implementing the websocketTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWebsocketTask */
void StartWebsocketTask(void *argument)
{
  /* USER CODE BEGIN StartWebsocketTask */
	struct mg_mgr mgr;
		  mg_mgr_init(&mgr);
		  mg_http_listen(&mgr, "http://192.168.0.121:8000", fn, NULL);
  /* Infinite loop */
  for(;;)
  {
	  mg_mgr_poll(&mgr, 10);
    osDelay(1);
  }
  /* USER CODE END StartWebsocketTask */
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
		  //osThreadFlagsSet(shutdownTaskHandle,SHUTDOWN);
		  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
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

static void fn(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG){
	  struct mg_http_message *hm = (struct mg_http_message *) ev_data;
	  if (mg_match(hm->uri, mg_str("/websocket"), NULL)) {
		// Upgrade to websocket. From now on, a connection is a full-duplex
		// Websocket connection, which will receive MG_EV_WS_MSG events.
		mg_ws_upgrade(c, hm, NULL);
	  }
  }
  else if (ev == MG_EV_WS_MSG) {
      // Got websocket frame. Received data is wm->data. Echo it back!
      struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
      mg_ws_send(c, wm->data.buf, wm->data.len, WEBSOCKET_OP_TEXT);
  }
}

void mg_random(void *buf, size_t len) {  // Use on-board RNG
  extern RNG_HandleTypeDef hrng;
  for (size_t n = 0; n < len; n += sizeof(uint32_t)) {
    uint32_t r;
    HAL_RNG_GenerateRandomNumber(&hrng, &r);
    memcpy((char *) buf + n, &r, n + sizeof(r) > len ? len - n : sizeof(r));
  }
}

/* USER CODE END Application */

