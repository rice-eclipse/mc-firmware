/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define FIR_LENGTH 16
#define MAX_PWD_LENGTH 15


extern sensor sensor_list[MAX_SENSOR_COUNT];
extern driver driver_list[MAX_DRIVER_COUNT];
extern monitor monitor_list[MAX_MONITOR_COUNT];
extern driver ignition;
extern char host_ip[MAX_IP_LEN];
extern char cmd_password[MAX_PWD_LENGTH];
extern int port;
extern int sampling_freq_ign;
extern int sampling_freq_standby;
extern char *cmd_buffer;
extern char *console_filename;
extern char *data_filename;
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
void print_buffer(char *buffer, int buf_len);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CS_Pin GPIO_PIN_2
#define SPI2_CS_GPIO_Port GPIOE
#define DRV0_Pin GPIO_PIN_7
#define DRV0_GPIO_Port GPIOE
#define DRV1_Pin GPIO_PIN_8
#define DRV1_GPIO_Port GPIOE
#define DRV2_Pin GPIO_PIN_9
#define DRV2_GPIO_Port GPIOE
#define DRV3_Pin GPIO_PIN_10
#define DRV3_GPIO_Port GPIOE
#define DRV4_Pin GPIO_PIN_11
#define DRV4_GPIO_Port GPIOE
#define IGN_Pin GPIO_PIN_12
#define IGN_GPIO_Port GPIOE
#define HBT_Pin GPIO_PIN_9
#define HBT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
