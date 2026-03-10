/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "fatfs.h"
#include "lwip.h"
#include "sdio.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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

/* USER CODE BEGIN PV */
uint32_t error_cnt;
uint32_t phy_addr;
uint8_t txBuffer[200];
uint8_t TxBuffer[300];
char RW_Buffer[200];
static char config_str[4000];
UINT WWC;
FATFS FatFs;
FIL Fil;
FRESULT fres;
FIL config_file;
int counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t get_mcp3208_adcval(int channel, uint16_t cs, SPI_HandleTypeDef *spiHandle);
float get_sensorval(int channel, uint16_t adc_cs);
int read_file(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	counter = 0;
	phy_addr = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_LWIP_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /*
  fres = (f_mount(&FatFs, "", 1));
  	if (fres != FR_OK){
  		sprintf((char *)txBuffer, "f_mount error (%i)\r\n", fres);
  		CDC_Transmit_FS(txBuffer,strlen((char *)txBuffer));
  	}else{
  		sprintf((char *)txBuffer, "SD Card Mounted Successfully! \r\n\n");
  		CDC_Transmit_FS(txBuffer, strlen((char *)txBuffer));
  	}
  	fres = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
  	    if(fres != FR_OK)
  	    {
  	      sprintf((char *)txBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", fres);
  	      CDC_Transmit_FS(txBuffer, strlen((char *)txBuffer));

  	    } else{
  	    	strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDIO, Using f_write()\r\n");
  	    	 f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
  	    }
  	    f_close(&Fil);
  	    read_file(&config_file, "config.json", config_str, 4000);
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	 HAL_GPIO_WritePin(HBT_GPIO_Port, HBT_Pin, GPIO_PIN_SET);
	 HAL_Delay(500);
	 HAL_GPIO_WritePin(HBT_GPIO_Port, HBT_Pin, GPIO_PIN_RESET);
	 HAL_Delay(500);
	 */
	  /*
	 float lc_voltage = get_sensorval(1, SPI2_CS_Pin);
	 if (counter == 0)
		 CDC_Transmit_FS(txBuffer, sprintf((char *)txBuffer, "%f\r\n",lc_voltage));
	 HAL_Delay(1);
	 counter = (counter+1)%400;
	 */
	 MX_LWIP_Process();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t get_mcp3208_adcval(int channel, uint16_t cs, SPI_HandleTypeDef *spiHandle){
	uint8_t tx[3];
	uint8_t rx[3];

	//start + single-ended + D2
	tx[0] = 0x06 | ((channel & 0x04) >> 2);
	//D1 + D0 shifted to B7 and B6
	tx[1] = (channel & 0x03) << 6;
	//don't care
	tx[2] = 0x00;

	HAL_GPIO_WritePin(GPIOE, cs, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spiHandle, tx, rx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOE, cs, GPIO_PIN_SET);

	uint16_t dataBuff = ((rx[1] & 0x0F) << 8) | rx[2];

	return dataBuff;
}
float get_sensorval(int channel, uint16_t adc_cs){
	uint16_t adc_val = get_mcp3208_adcval(channel, adc_cs, &hspi2);
	float voltage = (adc_val)*4.096/4096;
	//float sensor_val = (voltage*current_sensor->calibration_slope) + current_sensor->calibration_int;
	return voltage;
}

int read_file(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size)
{

	FRESULT fres;
	fres = f_open(target_file, filename, FA_READ);
	   if(fres != FR_OK) {
		sprintf((char *)TxBuffer,"f_open error (%i)\r\n", fres);
		CDC_Transmit_FS(TxBuffer, strlen((char *)TxBuffer));
		return -2;
	   }

	   //get the number of characters to allocate to this string
	   long size = f_size(target_file);
	   if (size > buffer_size){
		   sprintf((char *)TxBuffer, "Error: Input buffer size too small \r\n");
		   CDC_Transmit_FS(TxBuffer, strlen((char *)TxBuffer));
		   f_close(target_file);
		   return -2;
	   }
	   UINT br = 0;
	   FRESULT rres = f_read(target_file,(void *)data_buffer,size, &br);
	   if(rres != FR_OK) {
		   sprintf((char *)TxBuffer,"f_gets error (%i)\r\n", fres);
		   CDC_Transmit_FS(TxBuffer, strlen((char *)TxBuffer));
		   f_close(target_file);
		   return -2;
	   }
	   if (br != size){
		   sprintf((char *)TxBuffer,"File not read fully!\r\n");
		   CDC_Transmit_FS(TxBuffer, strlen((char *)TxBuffer));
	   }
	   f_close(target_file);
	   return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
