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
#include "tim.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	char cmd_buf[200];
	uint8_t cmd_idx;
} CMDQUEUE_OBJ_T;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_CMD_BACKLOG 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t current_cmd_idx;
static char TxBuffer[300];

osMessageQueueId_t cmdMessageQueueHandle;
const osMessageQueueAttr_t cmdMessageQueue_attributes = {
  .name = "cmdMessageQueue"
};
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void fn(struct mg_connection *c, int ev, void *ev_data);
void mg_random(void *buf, size_t len);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartServerTask(void *argument);
void StartCmdHandlingTask(void *argument);

extern void MX_LWIP_Init(void);
extern void MX_USB_DEVICE_Init(void);
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
	 cmdMessageQueueHandle = osMessageQueueNew (MAX_CMD_BACKLOG, sizeof(CMDQUEUE_OBJ_T), &cmdMessageQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of serverTask */
  serverTaskHandle = osThreadNew(StartServerTask, NULL, &serverTask_attributes);

  /* creation of cmdHandlingTask */
  cmdHandlingTaskHandle = osThreadNew(StartCmdHandlingTask, NULL, &cmdHandlingTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  extern struct netif gnetif;
   while(ip4_addr_isany_val(*netif_ip4_addr(&gnetif)))
  	  osDelay(200); // CMSIS-RTOS v1 uses milliseconds
    MG_INFO(("READY, IP: %s", ip4addr_ntoa(netif_ip4_addr(&gnetif))));
    serverTaskHandle = osThreadNew(StartServerTask, NULL, &serverTask_attributes);
    osThreadTerminate(defaultTaskHandle);
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
	 struct mg_mgr mgr;
	 mg_mgr_init(&mgr);
	mg_http_listen(&mgr, "http://192.168.0.122:8000", fn, NULL);  // Create HTTP listener
  /* Infinite loop */
  for(;;)
  {
	mg_mgr_poll(&mgr, 100);
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
	CMDQUEUE_OBJ_T cmd;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(cmdMessageQueueHandle, &cmd, NULL, osWaitForever);
	  	parse_command_interface(cmd.cmd_buf, &driver_id, &direction, driver_list, &ignition_flag,
	  				  	  	  	  	  	&shutdown_flag, &stop_ignition_flag, &actuation_flag);
	  	/*
	  	if (shutdown_flag == 1){
	  		osThreadFlagsSet(shutdownTaskHandle, SHUTDOWN);
	  	}
	  	*/
	  	if (stop_ignition_flag == 1){
	  		HAL_GPIO_WritePin(ignition.GPIO_Port, ignition.GPIO_Pin, 0);
	  	}
	  	else if (ignition_flag == 1){
	  		//ignition_sequence();
	  		HAL_GPIO_WritePin(ignition.GPIO_Port, ignition.GPIO_Pin, 1);
	  	}
	  	else if (actuation_flag == 1){
	  		HAL_GPIO_WritePin(driver_list[driver_id].GPIO_Port, driver_list[driver_id].GPIO_Pin, direction);
	  		sprintf(TxBuffer, "Actuating Driver %u. Direction: %d", driver_list[driver_id].GPIO_Pin, direction);
	  		HAL_GPIO_TogglePin(GPIOA, HBT_Pin);
	  		print_buffer(TxBuffer, strlen(TxBuffer));
    osDelay(1);
  }
  /* USER CODE END StartCmdHandlingTask */
}
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void fn(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_HTTP_MSG){
  	  struct mg_http_message *hm = (struct mg_http_message *) ev_data;
  	  if (mg_match(hm->uri, mg_str("/websocket"), NULL)) {
  		// Upgrade to websocket. From now on, a connection is a full-duplex
  		// Websocket connection, which will receive MG_EV_WS_MSG events.
  		mg_ws_upgrade(c, hm, NULL);
  	  }
    }
    else if (ev == MG_EV_WS_MSG) {
        // Got websocket frame. Received data is wm->data.
  	  CMDQUEUE_OBJ_T cmd;
        struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
        mg_ws_send(c, wm->data.buf, wm->data.len, WEBSOCKET_OP_TEXT);
        snprintf(cmd.cmd_buf, sizeof(cmd.cmd_buf),wm->data.buf);
        cmd.cmd_idx = current_cmd_idx;
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

/* USER CODE END Application */

