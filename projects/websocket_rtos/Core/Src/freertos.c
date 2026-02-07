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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const char *s_listen_on = "http://192.168.0.121";
static const char *s_web_root = ".";
static const char *s_ca_path = "ca.pem";
static const char *s_cert_path = "cert.pem";
static const char *s_key_path = "key.pem";
struct mg_str s_ca, s_cert, s_key;
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void fn(struct mg_connection *c, int ev, void *ev_data);
void mg_random(void *buf, size_t len);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartServerTask(void *argument);

extern void MX_LWIP_Init(void);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of serverTask */
  serverTaskHandle = osThreadNew(StartServerTask, NULL, &serverTask_attributes);

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
	  mg_http_listen(&mgr, "http://192.168.0.121:8000", fn, NULL);  // Create HTTP listener
  /* Infinite loop */
  for(;;)
  {
	  mg_mgr_poll(&mgr, 1000);
    osDelay(1);
  }
  /* USER CODE END StartServerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

