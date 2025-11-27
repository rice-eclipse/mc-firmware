/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	char* name;
	int channel;
	uint16_t adc_cs;
	float calibration_slope;
	float calibration_int;
} sensor;

typedef struct {
	GPIO_TypeDef *GPIO_Port;
	uint16_t GPIO_Pin;
} driver;

typedef struct{
	char* name;
	int channel;
	uint16_t adc_cs;
	float calibration_slope;
	float calibration_int;
	int sample_rate;
} monitor;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MAX_DRIVER_COUNT 6
#define MAX_SENSOR_COUNT 12
#define MAX_MONITOR_COUNT 7
#define MAX_IP_LEN 16

extern sensor sensor_list[MAX_SENSOR_COUNT];
extern driver driver_list[MAX_DRIVER_COUNT];
extern monitor monitor_list[MAX_MONITOR_COUNT];
extern char host_ip[MAX_IP_LEN];
extern int port;
extern int sampling_freq_ign;
extern int sampling_freq_standby;
extern char *cmd_buffer;
extern char *console_filename;
extern char *data_filename;
extern osEventFlagsId_t command_event;
extern int sensor_count;
extern int driver_count;
extern int monitor_count;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_1
#define SD_CS_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADC_SCK_Pin GPIO_PIN_12
#define ADC_SCK_GPIO_Port GPIOB
#define IGN_Pin GPIO_PIN_13
#define IGN_GPIO_Port GPIOB
#define ADC3_CS_Pin GPIO_PIN_8
#define ADC3_CS_GPIO_Port GPIOC
#define ADC2_CS_Pin GPIO_PIN_9
#define ADC2_CS_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ADC1_CS_Pin GPIO_PIN_10
#define ADC1_CS_GPIO_Port GPIOC
#define ADC_MISO_Pin GPIO_PIN_11
#define ADC_MISO_GPIO_Port GPIOC
#define ADC_MOSI_Pin GPIO_PIN_12
#define ADC_MOSI_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DRV0_Pin GPIO_PIN_5
#define DRV0_GPIO_Port GPIOB
#define DRV1_Pin GPIO_PIN_6
#define DRV1_GPIO_Port GPIOB
#define DRV2_Pin GPIO_PIN_7
#define DRV2_GPIO_Port GPIOB
#define DRV3_Pin GPIO_PIN_8
#define DRV3_GPIO_Port GPIOB
#define DRV4_Pin GPIO_PIN_9
#define DRV4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
