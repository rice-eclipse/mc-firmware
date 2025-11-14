/*
 * utils.c
 *
 *  Created on: Nov 9, 2025
 *      Author: Deepak
 */
#include "utils.h"

int parse_config(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list,
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby){
	const cJSON *sensor = NULL;
	const cJSON* host = NULL;
	const cJSON* host_children = NULL;
	const cJSON* sampling_f_ign = NULL;
	const cJSON* sampling_f_standby = NULL;
	const cJSON *sensors = NULL;
	const cJSON *drivers = NULL;
	const cJSON *driver = NULL;
    const cJSON *ignition_obj = NULL;
    const cJSON *monitors = NULL;
    const cJSON *monitor = NULL;
	int status = 0;

	cJSON *config_json = cJSON_Parse(config_string);
	if (config_json == NULL){
		status = 1;
		goto end;
	}
	host = cJSON_GetObjectItemCaseSensitive(config_json, "host");
    if (host == NULL){
        status = 1;
        goto end;
    }
    host_children = host->child;
    *port = cJSON_GetObjectItemCaseSensitive(config_json, "port")->valueint;

    sampling_f_ign = cJSON_GetObjectItemCaseSensitive(config_json, "sampling_freq_ignition");
    if (sampling_f_ign == NULL){
        status = 1;
        goto end;
    }
    *sampling_freq_ign = sampling_f_ign->valueint;

    sampling_f_standby = cJSON_GetObjectItemCaseSensitive(config_json, "sampling_freq_standby");
    if (sampling_freq_standby == NULL){
        status = 1;
        goto end;
    }
    *sampling_freq_standby = sampling_f_standby->valueint;

    //get all the sensor information
    sensors = cJSON_GetObjectItemCaseSensitive(config_json,"sensors");
    if (!cJSON_IsArray(sensors)){
        status = 1;
        goto end;
    }

    int sensor_count = cJSON_GetArraySize(sensors);
    sensor_list = (sensor *)malloc(sensor_count*sizeof(sensor));
    int curr_sensor = 0;
    cJSON_ArrayForEach(sensor, sensors){
        char* enabled = cJSON_GetObjectItemCaseSensitive(sensor, "enabled")->valuestring;
        if (enabled){
            sensor new_sensor;
            new_sensor.name = cJSON_GetObjectItemCaseSensitive(sensor,"sensor")->valuestring;
            new_sensor.channel = cJSON_GetObjectItemCaseSensitive(sensor,"channel")->valueint;
            new_sensor.adc_cs = cJSON_GetObjectItemCaseSensitive(sensor,"adc_cs")->valueint;
            new_sensor.calibration_int = (float)cJSON_GetObjectItemCaseSensitive(sensor, "calibration_intercept")->valuedouble;
            new_sensor.calibration_slope = (float)cJSON_GetObjectItemCaseSensitive(sensor,"calibration_slope")->valuedouble;
            sensor_buff[curr_sensor] = new_sensor;
            curr_sensor++;
        }
    }

    //get all the driver information
    int driver_count = cJSON_GetArraySize(drivers);
    drv_buff = (driver *)malloc(driver_count*sizeof(driver));
    int curr_driver = 0;
    cJSON_ArrayForEach(driver, drivers){
        char* enabled = cJSON_GetObjectItemCaseSensitive(driver, "enabled")->valuestring;
        if (enabled){
            driver new_driver;
            char* gpio_port = cJSON_GetObjectItemCaseSensitive(driver,"gpio_port")->valuestring;
            if (strcmp(gpio_port, "GPIOA") == 0){
                new_driver.GPIO_Port = GPIOA;
            } else if (strcmp(gpio_port,"GPIOB")){
                new_driver.GPIO_Port = GPIOB;
            } else{
                new_driver.GPIO_Port = GPIOC;
            }
            new_driver.GPIO_Pin = (uint16_t)cJSON_GetObjectItemCaseSensitive(driver,"gpio_pin")->valueint;
            curr_driver++;
        }
    }

    ignition_obj = cJSON_GetObjectItemCaseSensitive(config_json, "ignition");
    char *gpio_port = ignition_obj->valuestring;
    if (strcmp(gpio_port, "GPIOA") == 0){
            ignition.GPIO_Port = GPIOA;
            } else if (strcmp(gpio_port,"GPIOB")){
                ignition.GPIO_Port = GPIOB;
            } else{
                ignition.GPIO_Port = GPIOC;
            }
    ignition.GPIO_Pin = (uint16_t)cJSON_GetObjectItemCaseSensitive(config_json,"gpio_pin")->valueint;

    int monitor_count = cJSON_GetArraySize(monitors);
    monitor_buff = (monitor *)malloc(monitor_count*sizeof(monitor));
    int curr_monitor = 0;
    cJSON_ArrayForEach(monitor, monitors){
        char* enabled = cJSON_GetObjectItemCaseSensitive(monitor, "enabled")->valuestring;
        if (enabled){
            monitor new_monitor;
            new_monitor.name = cJSON_GetObjectItemCaseSensitive(monitor,"monitor")->valuestring;
            new_monitor.channel = cJSON_GetObjectItemCaseSensitive(monitor,"channel")->valueint;
            new_monitor.adc_cs = cJSON_GetObjectItemCaseSensitive(monitor,"adc_cs")->valueint;
            new_monitor.calibration_int = (float)cJSON_GetObjectItemCaseSensitive(monitor, "calibration_intercept")->valuedouble;
            new_monitor.calibration_slope = (float)cJSON_GetObjectItemCaseSensitive(monitor,"calibration_slope")->valuedouble;
            monitor_buff[curr_monitor] = new_monitor;
            curr_monitor++;
        }
    }
    end:
		cJSON_Delete(config_json);
		return status;
}

int parse_command(const char* json_string, int* driver_id, int* direction){
//CJSON variables to extract the relevant fields
	cJSON *cmd_type = NULL;
	cJSON *drv_id = NULL;
	cJSON *dir = NULL;
	int status = 0;

	cJSON *cmd = cJSON_Parse(json_string);
	if (cmd == NULL){
		const char* error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL){
			sprintf(TxBuffer, "Error before: %s\n", error_ptr);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		}
		status = 1;
		goto end;
	}

	cmd_type = cJSON_GetObjectItemCaseSensitive(cmd, "type");
	/*If an actuate command is given, parse the target driver and the direction*/
	if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring,"actuate") == 0){
		drv_id = cJSON_GetObjectItemCaseSensitive(cmd, "driver-id");
		//if the input driver id is valid and the direction is valid, save the id and direction
		if(cJSON_IsNumber(drv_id)){
			dir = cJSON_GetObjectItemCaseSensitive(cmd,"direction");
			if (cJSON_IsNumber(dir)){
				*driver_id = drv_id->valueint;
				*direction = dir->valueint;
				status = 0;
			}
			else{
				status = 1;
				goto end;
			}
		}
		else{
			status = 1;
			goto end;
		}
	}
	end:
		cJSON_Delete(cmd);
		return status;
}

char *read_file(const char *filename){
	FIL fil;
	FRESULT fres;
	fres = f_open(&fil, filename, FA_READ);
	   if(fres != FR_OK) {
		sprintf(buffer,"f_open error (%i)\r\n", fres);
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		while (1);
	   }

	   //get the number of characters to allocate to this string
	   f_lseek(&fil,SEEK_END);
	   long size = f_tell(&fil);
	   char readBuf[size];

	   TCHAR *rres = f_gets((TCHAR*)readBuf,size, &fil);
	   if(rres != 0) {
			sprintf(TxBuffer,"Read string from 'test.txt' contents: %s\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)TxBuffer, sizeof(*TxBuffer), HAL_MAX_DELAY);

	   } else {
			sprintf(TxBuffer,"f_gets error (%i)\r\n", fres);
			TxBuffer;
	   }
	   f_close(&fil);
	   return readBuf;
}
char *create_file(const char *filename){
	FIL fil;
	FRESULT fres;
	fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	   if(fres != FR_OK) {
	 	sprintf(buffer,"f_open error (%i)\r\n", fres);
	 	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), -1);
	   }
	f_close(&fil);
}

int mount_sd(FATFS* Fatfs){
	return(f_mount(FatFs, "", 1));
}



