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
#include "mongoose.h"
#include "lwip.h"
#include "interface.h"
#include "usart.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	char cmd_buf[200];
	uint8_t cmd_idx;
} CMDQUEUE_OBJ_T;

typedef struct{
	float cal_sensor_vals[MAX_SENSOR_COUNT];
	int driver_states[MAX_DRIVER_COUNT];
} telemetry_snapshot_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CMD_BACKLOG 5
#define FIRST_BUF_READY (1 << 0)
#define SECOND_BUF_READY (1 << 1)
#define SAMPLE_NOW (1 << 2)
#define SHUTDOWN (1 << 3)
#define LOGGING_STOPPED (1 << 4)
#define STOP_LOGGING (1 << 5)
#define SEND_NOW (1 << 7)
//flags for the sending task
#define ACTUATED (1 << 8)
#define INCORRECT_PASSWORD (1 << 9)
#define IGNITION_COUNT (1 << 10)
#define IGNITION_IP (1 << 11)

#define DECIMATED_LOGGING_FACTOR 10
#define DECIMATED_TELEMETRY_FACTOR 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char ip_addr[100];
uint8_t current_cmd_idx;
static char TxBuffer[300];
static char sensor_data_str[1072];
//Ping-pong buffer for data to write to sd
uint16_t sensor_vals[2*MAX_SENSOR_COUNT];
int driver_states[MAX_DRIVER_COUNT];
telemetry_snapshot_t telem_snapshot;

//stores the filtered and decimated outputs and the calibrated values respectively
uint16_t decimated_sensor_vals[MAX_SENSOR_COUNT];
float calibrated_sensor_vals[MAX_SENSOR_COUNT];

//pointer to the 'last filled buffer'
uint16_t *filled_buffer;
/*Make vals to log and vals to send the same*/
/*Current sample in the current sampling cycle*/
int sample_count;
//tx and rx buffers for ADC sample retrieval
uint8_t tx[3];
uint8_t rx[3];

//decimation count for the CIC filter
uint32_t decimation_count;


//stores the sensor values in a single string to write to the sd card
char sdcard_data[3072];
char data_log[250];
char cmd_log[400];

//str for data header and logging each line
char line_buf[0];
char data_header_str[300];
//timestamp for logging
char timestamp[50];
//log and data file handles
FIL data_file;
FIL log_file;

//ignition counter
int ignition_count;
//tracks the number of samples collected
unsigned long samples_collected;
osMessageQueueId_t cmdMessageQueueHandle;
const osMessageQueueAttr_t cmdMessageQueue_attributes = {
  .name = "cmdMessageQueue"
};

osTimerId_t ignition_timer;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for serverTask */
osThreadId_t serverTaskHandle;
const osThreadAttr_t serverTask_attributes = {
  .name = "serverTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for cmdHandlingTask */
osThreadId_t cmdHandlingTaskHandle;
const osThreadAttr_t cmdHandlingTask_attributes = {
  .name = "cmdHandlingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for processingTask */
osThreadId_t processingTaskHandle;
const osThreadAttr_t processingTask_attributes = {
  .name = "processingTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for shutdownTask */
osThreadId_t shutdownTaskHandle;
const osThreadAttr_t shutdownTask_attributes = {
  .name = "shutdownTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for loggingMutex */
osMutexId_t loggingMutexHandle;
const osMutexAttr_t loggingMutex_attributes = {
  .name = "loggingMutex"
};
/* Definitions for driversMutex */
osMutexId_t driversMutexHandle;
const osMutexAttr_t driversMutex_attributes = {
  .name = "driversMutex"
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void fn(struct mg_connection *c, int ev, void *ev_data);
void mg_random(void *buf, size_t len);
void add_datafile_header();
static void ignition_timer_Callback(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartServerTask(void *argument);
void StartCmdHandlingTask(void *argument);
void StartProcessingTask(void *argument);
void StartShutdownTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	samples_collected = 0;
	sample_count = 0;
	decimation_count = 0;
	ignition_count = 10;


  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of loggingMutex */
  loggingMutexHandle = osMutexNew(&loggingMutex_attributes);

  /* creation of driversMutex */
  driversMutexHandle = osMutexNew(&driversMutex_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
 	cmdMessageQueueHandle = osMessageQueueNew (4, sizeof(CMDQUEUE_OBJ_T), &cmdMessageQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  extern struct netif gnetif;
  while(ip4_addr_isany_val(*netif_ip4_addr(&gnetif)))
 	  osDelay(200);
   MG_INFO(("READY, IP: %s", ip4addr_ntoa(netif_ip4_addr(&gnetif))));
   /*Start the timers and the rest of the tasks and creates the necessary files*/
   create_file_interface(&data_file,data_filename);
   create_file_interface(&log_file,console_filename);
  cmdHandlingTaskHandle = osThreadNew(StartCmdHandlingTask, NULL, &cmdHandlingTask_attributes);
  serverTaskHandle = osThreadNew(StartServerTask, NULL, &serverTask_attributes);
  processingTaskHandle = osThreadNew(StartProcessingTask, NULL, &processingTask_attributes);
  shutdownTaskHandle = osThreadNew(StartShutdownTask, NULL, &shutdownTask_attributes);
  ignition_timer = osTimerNew(ignition_timer_Callback, osTimerOnce,NULL, NULL);
   HAL_TIM_Base_Start_IT(&htim14);
   HAL_TIM_Base_Start_IT(&htim13);
   osThreadExit();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartServerTask */
/**
* @brief Function implementing the serverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServerTask */
void StartServerTask(void *argument)
{
  /* USER CODE BEGIN StartServerTask */
	sprintf(ip_addr, "http://%s:%d",host_ip,port);
	uint32_t sending_flag;
	 struct mg_mgr mgr;
	  mg_mgr_init(&mgr);
	  mg_http_listen(&mgr, ip_addr, fn, NULL);  // Create HTTP listener
  /* Infinite loop */
  for(;;)
  {
	  mg_mgr_poll(&mgr, 10);
	  //package the data in json for the websocket and send at each sending interval
	  sending_flag = osThreadFlagsWait((SEND_NOW|ACTUATED|INCORRECT_PASSWORD|IGNITION_IP|IGNITION_COUNT), osFlagsWaitAny, 0);
	  if ((sending_flag & SEND_NOW) != 0){
		  sensor_message_interface(sensor_data_str, sizeof(sensor_data_str),
				  telem_snapshot.cal_sensor_vals,telem_snapshot.driver_states);
		  for (struct mg_connection *client = mgr.conns; client != NULL; client = client->next){
			  if (client->data[0] == 'W'){
				  mg_ws_send(client, (void *)sensor_data_str,strlen(sensor_data_str), WEBSOCKET_OP_TEXT);
			  }
		  }

	  }
	  if ((sending_flag & (ACTUATED|INCORRECT_PASSWORD)) != 0){
		  for (struct mg_connection *client = mgr.conns; client != NULL; client = client->next){
			  if (client->data[0] == 'W'){
				  //shitty solution to avoid sending the timestamp as well lol.
				  mg_ws_send(client, (void *)(cmd_log+50),strlen(cmd_log)-50, WEBSOCKET_OP_TEXT);
			  }
		  }
	  }
	  if ((sending_flag &IGNITION_COUNT) != 0){
		  sprintf(TxBuffer, "IGNITION in %d\r\n", ignition_count);
		  for (struct mg_connection *client = mgr.conns; client != NULL; client = client->next){
		  			  if (client->data[0] == 'W'){
		  				  mg_ws_send(client, (void *)TxBuffer,strlen(TxBuffer), WEBSOCKET_OP_TEXT);
		  			  }
		  		  }
		  osTimerStart(ignition_timer, 1000U);
	  }
	  if ((sending_flag & IGNITION_IP) != 0){
		  sprintf(TxBuffer, "IGNITION in Progress!\r\n");
		  for (struct mg_connection *client = mgr.conns; client != NULL; client = client->next){
			  if (client->data[0] == 'W'){
				  mg_ws_send(client, (void *)TxBuffer,strlen(TxBuffer), WEBSOCKET_OP_TEXT);
			  }
		  }

	  }
    osDelay(1);
  }
  /* USER CODE END StartServerTask */
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
	int ignition_flag;
	int shutdown_flag;
	int stop_ignition_flag;
	int actuation_flag;
	int driver_id;
	int direction;
	static char cmd_timestamp[50];
	CMDQUEUE_OBJ_T cmd;
#ifndef TEST_LOGIC
	FRESULT fres;
#endif
  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(cmdMessageQueueHandle, &cmd, NULL, osWaitForever);
	get_timestamp_interface(cmd_timestamp, 50);
	int result = parse_command_interface(cmd.cmd_buf, &driver_id, &direction, driver_list, &ignition_flag,
				  	  	  	  	  	&shutdown_flag, &stop_ignition_flag, &actuation_flag);
	//HAL_UART_Transmit(&huart3, (uint8_t *)cmd.cmd_buf, strlen(cmd.cmd_buf), HAL_MAX_DELAY);
	if (result == -1){
		sprintf(cmd_log, "%s Incorrect password\r\n", cmd_timestamp);
		osThreadFlagsSet(serverTaskHandle, INCORRECT_PASSWORD);
		continue;
	}
	sprintf(cmd_log, "%s Received command: %s\r\n",cmd_timestamp, cmd.cmd_buf);
	osMutexAcquire(loggingMutexHandle, osWaitForever);
#ifndef TEST_LOGIC
			  fres = append_file_interface(&log_file, cmd_log, strlen(cmd_log));
#endif
	osMutexRelease(loggingMutexHandle);
	if (shutdown_flag == 1){
		osThreadFlagsSet(shutdownTaskHandle, SHUTDOWN);
	}
	if (stop_ignition_flag == 1){
		HAL_GPIO_WritePin(ignition.GPIO_Port, ignition.GPIO_Pin, 0);
		osTimerStop(ignition_timer);
	}
	else if (ignition_flag == 1){
		//Ignition timer
		osTimerStart(ignition_timer, 1000U);
		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
	}
	else if (actuation_flag == 1){
		if (driver_id >= 0 && driver_id < MAX_DRIVER_COUNT){
			HAL_GPIO_WritePin(driver_list[driver_id].GPIO_Port, driver_list[driver_id].GPIO_Pin, direction);
			//TODO: use driver current monitors to verify this
			osMutexAcquire(driversMutexHandle, osWaitForever);
			driver_states[driver_id] = (direction == 1) ? 1 : 0;
			osMutexRelease(driversMutexHandle);

			sprintf(cmd_log, "%s Actuating driver id  %d - %d\r\n",cmd_timestamp, driver_list[driver_id].GPIO_Pin, direction);
				osMutexAcquire(loggingMutexHandle, osWaitForever);
#ifndef TEST_LOGIC
			  fres = append_file_interface(&log_file, cmd_log, strlen(cmd_log));
#endif
	osMutexRelease(loggingMutexHandle);
		}
	}
    osDelay(1);
  }
  /* USER CODE END StartCmdHandlingTask */
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
	uint32_t stopLogging_flag;
	int sd_card_pos = 0;
	uint16_t *current_buffer;
	open_file_interface(&data_file, data_filename);
	add_datafile_header();
	open_file_interface(&log_file, console_filename);
	/*We sync the file to the sd card every second*/
	int sync_count = 0;
	int log_count =0;
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
	  stopLogging_flag = osThreadFlagsWait(STOP_LOGGING, (osFlagsWaitAny|osFlagsNoClear), 10);
	  //if a shutdown flag is set, we don't perform any more logging operations and let the shutdown task know it can access the sd card
	  if (stopLogging_flag == STOP_LOGGING){
		  osThreadFlagsSet(shutdownTaskHandle,LOGGING_STOPPED);
	  }
	  //perform filtering and decimation and log data to sd card
	  else{
		  /*TODO: implement filtering and decimation*/
		  	/***Raw sampling buffer is now decoupled from processing **/
		  	 if (decimation_count == DECIMATED_LOGGING_FACTOR){
		  		 memcpy(decimated_sensor_vals,current_buffer,sensor_count*sizeof(uint16_t));
		  		int line_len = 0;
		  		get_timestamp_interface(timestamp,50);
		  		line_len+= snprintf(line_buf, sizeof(line_buf),"%s,",timestamp);
		  		 for (int i = 0; i < sensor_count; i++){
		  			float voltage_val = decimated_sensor_vals[i]*4096/4096;
		  			 calibrated_sensor_vals[i] = voltage_val*sensor_list[i].calibration_slope + sensor_list[i].calibration_int;
		  			line_len += snprintf(line_buf + line_len, sizeof(line_buf)-line_len, "%.3f,",calibrated_sensor_vals[i]);
		  		 }
		  		line_len += snprintf(line_buf + line_len, sizeof(line_buf)-line_len, "\r\n");
		  		 //only add the new line if it doesn't cause an overflow
				  if (sd_card_pos+line_len < sizeof(sdcard_data)){
					  memcpy(sdcard_data+sd_card_pos, line_buf, line_len);
					  sd_card_pos += line_len;
					  log_count++;
					  samples_collected++;
					  }
				  //otherwise, we just write what we have to the sd card first
				  else{
					  log_count = 20;
				  }
				  if (log_count == 20){
					  osMutexAcquire(loggingMutexHandle, osWaitForever);
		#ifndef TEST_LOGIC
					  fres = append_file_interface(&data_file, sdcard_data,sd_card_pos);
		#endif
					  osMutexRelease(loggingMutexHandle);
					  //clear the sd write buffer for the next cycle
					  sd_card_pos = 0;
					  log_count = 0;
				  }
				  //Log whenever 1000 samples have been collected
				  if ((samples_collected % 1000) == 0){
						  get_timestamp_interface(timestamp, 50);
						  sprintf(data_log,"%s %lu samples obtained\r\n",timestamp,samples_collected);
						  osMutexAcquire(loggingMutexHandle, osWaitForever);
		  #ifndef TEST_LOGIC
						  fres = append_file_interface(&log_file, data_log, strlen(data_log));
		  #endif
						  osMutexRelease(loggingMutexHandle);
						  }
			  //flush cache back to sd card every 10 seconds
		  sync_count = (sync_count + DECIMATED_LOGGING_FACTOR) % (10*sampling_freq_ign);
				  if (sync_count == 0){
				  }
				  osMutexAcquire(loggingMutexHandle, osWaitForever);
	  #ifndef TEST_LOGIC
			  /*sync data to SD card every second*/
				  f_sync(&data_file);
				  f_sync(&log_file);
		#endif
				  osMutexRelease(loggingMutexHandle);

		  		 if (decimation_count == DECIMATED_TELEMETRY_FACTOR){
		  			 memcpy(telem_snapshot.cal_sensor_vals, calibrated_sensor_vals, sensor_count*sizeof(float));
		  			 osMutexAcquire(driversMutexHandle, 100U);
		  			 memcpy(telem_snapshot.driver_states, driver_states, driver_count*sizeof(int));
		  			 osMutexRelease(driversMutexHandle);
		  		 }
		  	 }

	 }
	  osDelay(1);
  }
  /* USER CODE END StartProcessingTask */
}

/* USER CODE BEGIN Header_StartShutdownTask */
/**
* @brief Function implementing the ShutdownTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShutdownTask */
void StartShutdownTask(void *argument)
{
  /* USER CODE BEGIN StartShutdownTask */
	uint32_t shutdown_flag;
	uint32_t stoppedLogging_flag;
  /* Infinite loop */
  for(;;)
  {
	  shutdown_flag = osThreadFlagsWait(SHUTDOWN, osFlagsWaitAny, osWaitForever);
	  	/*Tell processing task to stop*/
	  HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
	  osThreadFlagsSet(processingTaskHandle, STOP_LOGGING);

	  	/*Waits for processing to stop*/
	  	stoppedLogging_flag = osThreadFlagsWait(LOGGING_STOPPED, osFlagsWaitAny, osWaitForever);
	  	close_file_interface(&data_file);
	  	close_file_interface(&log_file);
	  	unmount_sd_interface();
	  	if (processingTaskHandle != NULL){
	  		osThreadTerminate(processingTaskHandle);
	  	}
	  	if (serverTaskHandle != NULL){
	  		osThreadTerminate(serverTaskHandle);
	  	}
	  	if (cmdHandlingTaskHandle != NULL){
	  		osThreadTerminate(cmdHandlingTaskHandle);
	  	}
	  	if (ignition_timer != NULL){
	  		osTimerDelete(ignition_timer);
	  	}
	  	sprintf(TxBuffer, "All tasks closed\r\n");
	  	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
	  	osThreadExit();
	      osDelay(1);
  }
  /* USER CODE END StartShutdownTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char *pcTaskName ){
	sprintf(TxBuffer, "%s\r\n", pcTaskName);
	 HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
}

static void fn(struct mg_connection *c, int ev, void *ev_data) {
  if (ev == MG_EV_HTTP_MSG){
	  struct mg_http_message *hm = (struct mg_http_message *) ev_data;
	  if (mg_match(hm->uri, mg_str("/websocket"), NULL)) {
		// Upgrade to websocket. From now on, a connection is a full-duplex
		// Websocket connection, which will receive MG_EV_WS_MSG events.
		mg_ws_upgrade(c, hm, NULL);
		//mark connection
		c->data[0] = 'W';
	  }
  }
  else if (ev == MG_EV_WS_MSG) {
      // Got websocket frame. Received data is wm->data.
	  CMDQUEUE_OBJ_T cmd;
      struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
      mg_ws_send(c, wm->data.buf, wm->data.len, WEBSOCKET_OP_TEXT);
      size_t len = wm->data.len;
      if (len >= sizeof(cmd.cmd_buf)) {
          len = sizeof(cmd.cmd_buf) - 1;
      }
      memcpy(cmd.cmd_buf, wm->data.buf, len);
      cmd.cmd_buf[len] = '\0';
      cmd.cmd_idx = current_cmd_idx;
      //sprintf(TxBuffer, "received data %s\r\n",cmd.cmd_buf);
      //HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
      current_cmd_idx++;
      if (current_cmd_idx > MAX_CMD_BACKLOG){
    	  //trigger shutdown condition
    	  osMessageQueueReset(cmdMessageQueueHandle);
      }
      osMessageQueuePut(cmdMessageQueueHandle, &cmd,0U, 0U);
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
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /*Sampling timer: collects the data from the ADC*/
  if (htim->Instance == TIM14){
	  int curr_channel = sensor_list[sample_count%sensor_count].channel;
	  	tx[0] = 0x06 | ((curr_channel & 0x04) >> 2);
	  	tx[1] = (curr_channel & 0x03) << 6;
	  	tx[2] = 0x00;

	  	HAL_GPIO_WritePin(GPIOF,sensor_list[sample_count%sensor_count].adc_cs, GPIO_PIN_RESET);
	  	HAL_SPI_TransmitReceive_IT(&hspi1, tx, rx, 3);
  }
  if (htim->Instance == TIM13){
	  osThreadFlagsSet(serverTaskHandle, SEND_NOW);

  }
  if (htim->Instance == TIM11){
	  osThreadFlagsSet(shutdownTaskHandle, SHUTDOWN);
  }

  /* USER CODE END Callback 1 */
}
/*Retrieves the data from the ADC and writes it to the ping pong buffer*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if (hspi->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOF,sensor_list[sample_count%sensor_count].adc_cs, GPIO_PIN_SET);
		sensor_vals[sample_count] = ((rx[1] & 0xF) << 8) | rx[2];
		sample_count = (sample_count + 1) % (sensor_count*2);

		//notify processing task if either of the ping pong buffers is full
		if (sample_count == sensor_count){
			osThreadFlagsSet(processingTaskHandle, FIRST_BUF_READY);
			filled_buffer = &sensor_vals[0];
			decimation_count = (decimation_count+1) % DECIMATED_TELEMETRY_FACTOR;
		}
		else if (sample_count == 0){
			osThreadFlagsSet(processingTaskHandle, SECOND_BUF_READY);
			filled_buffer = &sensor_vals[sensor_count];
			decimation_count = (decimation_count+1) % DECIMATED_TELEMETRY_FACTOR;
		}
	}
}

static void ignition_timer_Callback(void *argument){
	if (ignition_count > 0){
		osThreadFlagsSet(serverTaskHandle, IGNITION_COUNT);
		ignition_count--;
	}
	else if (ignition_count == 0){
		osThreadFlagsSet(serverTaskHandle, IGNITION_IP);
	}
}
void add_datafile_header(){
	int card_pos = 0;
	data_header_str[0] = '\0';
	for (int i = 0; i < sensor_count; i++){
		card_pos += snprintf(data_header_str+card_pos,sizeof(data_header_str) - card_pos,"%s,",
							 sensor_list[i].name);
		if (card_pos <= 0 || card_pos > sizeof(data_header_str)){
			 sprintf(TxBuffer, "sensor list array corrupted!\r\n");
			 HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		}
	}
	card_pos += snprintf(data_header_str+card_pos, sizeof(data_header_str)-card_pos, "\r\n");
	if (card_pos <= 0 || card_pos > sizeof(data_header_str)){
			 sprintf(TxBuffer, "sensor list array corrupted!\r\n");
			 HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);

		}
#ifndef TEST_LOGIC
	append_file_interface(&data_file, data_header_str, card_pos);
#endif
}
/* USER CODE END Application */

