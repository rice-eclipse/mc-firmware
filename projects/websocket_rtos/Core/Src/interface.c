/*
 * interface.c
 *
 *  Created on: Nov 22, 2025
 *      Author: Deepak
 */

#include "interface.h"
#include <string.h>
#include "cJSON.h"
#include "usart.h"
#include <stdio.h>


static char TxBuffer[300];
/******************************************************************************************/
/*Interface definitions if we are only testing the RTOS logic without the physical drivers*/
/******************************************************************************************/

/*Config parsing doesn't depend on any hardware*/
int parse_config_interface(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list, driver *ignition,
				 char *host_ip,char *password, int *port,int *sampling_freq_ign,int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count){
	const cJSON *host = NULL;
	    const cJSON *pwd = NULL;
	    const cJSON *host_children = NULL;
	    const cJSON *sampling_f_ign = NULL;
	    const cJSON *sampling_f_standby = NULL;
	    const cJSON *sensors = NULL;
	    const cJSON* sensor_obj = NULL;
	    const cJSON *drivers = NULL;
	    const cJSON *driver_obj = NULL;
	    const cJSON *ignition_obj = NULL;
	    const cJSON *monitors = NULL;
	    const cJSON *monitor_obj = NULL;
	    int status = 0;

		cJSON *config_json = cJSON_Parse(config_str);
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


	    if (host_children != NULL && host_children->valuestring != NULL && host_ip != NULL) {
	        strcpy(host_ip, host_children->valuestring);
	    }

	    *port = cJSON_GetObjectItemCaseSensitive(config_json, "port")->valueint;

	    sampling_f_ign = cJSON_GetObjectItemCaseSensitive(config_json, "sampling_freq_ignition");
	    if (sampling_f_ign == NULL) {
	        status = 1;
	        goto end;
	    }
	    *sampling_freq_ign = sampling_f_ign->valueint;

	    pwd = cJSON_GetObjectItemCaseSensitive(config_json, "password");
	    if (pwd == NULL){
	    	status = 1;
	    	goto end;
	    }
	    strcpy(password, pwd->valuestring);

	    sampling_f_standby = cJSON_GetObjectItemCaseSensitive(config_json, "sampling_freq_standby");
	    if (sampling_f_standby == NULL) {
	        status = 1;
	        goto end;
	    }
	    *sampling_freq_standby = sampling_f_standby->valueint;
	    sensors = cJSON_GetObjectItemCaseSensitive(config_json, "sensors");
	    if (cJSON_IsArray(sensors)) {
			int num_sensors = cJSON_GetArraySize(sensors);
			if (num_sensors > MAX_SENSOR_COUNT){
				sprintf(TxBuffer,"config has too many sensors");
				HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer,strlen(TxBuffer),HAL_MAX_DELAY);
				return -2;
			}
			int curr_sensor = 0;
			int cs_pin =0;
			cJSON_ArrayForEach(sensor_obj, sensors) {
				char *enabled = cJSON_GetObjectItemCaseSensitive(sensor_obj, "enabled")->valuestring;
				if (strcmp(enabled,"true") == 0) {
					sensor new_sensor;
					new_sensor.name = strdup(cJSON_GetObjectItemCaseSensitive(sensor_obj, "sensor")->valuestring);
					new_sensor.channel = cJSON_GetObjectItemCaseSensitive(sensor_obj, "channel")->valueint;
					cs_pin = cJSON_GetObjectItemCaseSensitive(sensor_obj, "adc_cs")->valueint;
					new_sensor.calibration_int =
						(float)cJSON_GetObjectItemCaseSensitive(sensor_obj, "calibration_intercept")->valuedouble;
					new_sensor.calibration_slope =
						(float)cJSON_GetObjectItemCaseSensitive(sensor_obj, "calibration_slope")->valuedouble;

					switch (cs_pin){
					case 1:
						new_sensor.adc_cs = 1;
						break;
					case 2:
						new_sensor.adc_cs = 1;
						break;
						break;
					//need to do some error handling here
					default:
						new_sensor.adc_cs = 1;
						break;
					}

					sensor_list[curr_sensor] = new_sensor;
					curr_sensor++;
				}
			}
			//update the sensor count with the true number of sensors
			*sensor_count = curr_sensor;
	    }
	    drivers = cJSON_GetObjectItemCaseSensitive(config_json, "drivers");
	    if (cJSON_IsArray(drivers)) {
	        int num_drivers = cJSON_GetArraySize(drivers);
	        if (num_drivers > MAX_DRIVER_COUNT){
	        	sprintf(TxBuffer, "config has too many drivers");
	        	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer),HAL_MAX_DELAY);
	        	return -2;
	        }

	        int curr_driver = 0;
	        int gpio_pin = 0;
	        cJSON_ArrayForEach(driver_obj, drivers) {
	            char *enabled = cJSON_GetObjectItemCaseSensitive(driver_obj, "enabled")->valuestring;
	            if (strcmp(enabled,"true") == 0) {
	                driver new_driver;
	                char *gpio_port = cJSON_GetObjectItemCaseSensitive(driver_obj, "gpio_port")->valuestring;
	                if (strcmp(gpio_port, "GPIOA") == 0) {
	                    new_driver.GPIO_Port = GPIOA;
	                } else if (strcmp(gpio_port, "GPIOB") == 0) {
	                    new_driver.GPIO_Port = GPIOB;
	                } else {
	                    new_driver.GPIO_Port = GPIOC;
	                }

	                gpio_pin = cJSON_GetObjectItemCaseSensitive(driver_obj, "gpio_pin")->valueint;

	                switch (gpio_pin){
	                case 0:
	                	new_driver.GPIO_Pin = DRV0_Pin;
	                	break;
	                case 1:
	                	new_driver.GPIO_Pin = DRV1_Pin;
	                	break;
	                case 2:
	                	new_driver.GPIO_Pin = DRV2_Pin;
	                	break;
	                case 3:
	                	new_driver.GPIO_Pin = DRV3_Pin;
	                	break;
	                case 4:
	                	new_driver.GPIO_Pin = DRV4_Pin;
	                	break;
	                // Need to do error handling here
	                default:
	                	new_driver.GPIO_Pin = DRV0_Pin;
	                	break;
	                }
	                driver_list[curr_driver] = new_driver;
	                curr_driver++;
	            }
	        }
	        //update the driver count with the true number of drivers
	        *driver_count = curr_driver;
	    }



	    ignition_obj = cJSON_GetObjectItemCaseSensitive(config_json, "ignition");
	    if (ignition_obj != NULL) {
	        char *gpio_port = cJSON_GetObjectItemCaseSensitive(ignition_obj, "gpio_port")->valuestring;
	        if (strcmp(gpio_port, "GPIOA") == 0) {
	            ignition->GPIO_Port = GPIOA;
	        } else if (strcmp(gpio_port, "GPIOB") == 0) {
	            ignition->GPIO_Port = GPIOB;
	        } else {
	            ignition->GPIO_Port = GPIOC;
	        }
	        ignition->GPIO_Pin = IGN_Pin;
	    }

	    monitors = cJSON_GetObjectItemCaseSensitive(config_json, "monitors");
	    if (cJSON_IsArray(monitors)) {
	        int num_monitors = cJSON_GetArraySize(monitors);
	        if (num_monitors > MAX_MONITOR_COUNT){
	               	sprintf(TxBuffer, "config has too many monitors");
	               	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer),HAL_MAX_DELAY);
	               	return -2;
	               }
	        int curr_monitor = 0;

	        int cs_pin = 0;
	        cJSON_ArrayForEach(monitor_obj, monitors) {
	            char *enabled = cJSON_GetObjectItemCaseSensitive(monitor_obj, "enabled")->valuestring;
	            if (strcmp(enabled,"true")) {
	                monitor new_monitor;
	                new_monitor.name = strdup(cJSON_GetObjectItemCaseSensitive(monitor_obj, "monitor")->valuestring);
	                new_monitor.channel = cJSON_GetObjectItemCaseSensitive(monitor_obj, "channel")->valueint;
	                cs_pin = cJSON_GetObjectItemCaseSensitive(monitor_obj, "adc_cs")->valueint;
	                new_monitor.calibration_int =
	                    (float)cJSON_GetObjectItemCaseSensitive(monitor_obj, "calibration_intercept")->valuedouble;
	                new_monitor.calibration_slope =
	                    (float)cJSON_GetObjectItemCaseSensitive(monitor_obj, "calibration_slope")->valuedouble;

	                switch (cs_pin){
						case 1:
							new_monitor.adc_cs =1;
							break;
						case 2:
							new_monitor.adc_cs = 1;
							break;
						//need to do some error handling here
						default:
							new_monitor.adc_cs = 1;
							break;
						}
	                monitor_list[curr_monitor] = new_monitor;
	                curr_monitor++;
	            }
	        }
	        //update the monitor count with the true number of monitors
	        *monitor_count = curr_monitor;
	    }

	end:
	    cJSON_Delete(config_json);
	    return status;
}
int read_file_interface(const char *filename, char *data_buffer, size_t buffer_size){

	const char *config_str2 =
	"{"
	"\"host\": {\"ip\": \"127.0.0.1\"},"
	"\"password\": \"quonk\","
	"\"port\": 1234,"
	"\"sampling_freq_ignition\": 5000,"
	"\"sampling_freq_standby\": 1,"
	"\"drivers\": ["
	"  {"
	"    \"driver\": \"D1: Ox Fill\","
	"    \"gpio_pin\": 0,"
	"    \"gpio_port\": \"GPIOB\","
	"    \"channel\": 2,"
	"    \"adc_cs\": 2,"
	"    \"enabled\": \"true\""
	"  },"
	"  {"
	"    \"driver\": \"D2: Ground Vent\","
	"    \"gpio_pin\": 1,"
	"    \"gpio_port\": \"GPIOB\","
	"    \"calibration_intercept\": 32,"
	"    \"channel\": 1,"
	"    \"adc_cs\": 2,"
	"    \"enabled\": \"true\""
	"  },"
	"  {"
	"    \"driver\": \"D3: Engine Vent\","
	"    \"gpio_pin\": 2,"
	"    \"gpio_port\": \"GPIOC\","
	"    \"channel\": 3,"
	"    \"adc_cs\": 2,"
	"    \"enabled\": \"true\""
	"  }"
	"]"
	"}";



	int config_len = strlen(config_str2);
	if (config_len > buffer_size){
		sprintf(TxBuffer, "Error: Input buffer size too small \r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		return -1;
	} else{
		strcpy(data_buffer, config_str2);
	}
	return 0;
}

/*
int create_file_interface(FIL *target_file, const char *filename){
	sprintf(TxBuffer,"[TEST] file created successfully \r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
	return 0;
}

int open_file_interface(FIL *target_file, const char *filename){
	sprintf(TxBuffer,"[TEST] %s opened successfully \r\n", filename);
		HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		return 0;
}

int append_file_interface(FIL *target_file, char *data, UINT btw){
	sprintf(TxBuffer, "wrote %s\r\n", data);
	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
	return 0;
}

int close_file_interface(FIL *target_file){
	sprintf(TxBuffer, "closed file\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
	return 0;
}

int mount_sd_interface(FATFS* FatFs){
	sprintf(TxBuffer, "Mounted sd \r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
	return 0;
}
*/
float get_sensorval_interface(sensor *current_sensor){
	uint16_t adc_val =0;
	static float sensor_val = 0;
	return sensor_val;
}
void filter_and_decimate_interface(float *sensor_vals, int sensor_count){
	return;
}
int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list, int *ignition_flag,
  	  	  	int *shutdown_flag, int *cancel_ignition_flag, int *actuation_flag){
	// CJSON variables to extract the relevant fields
	cJSON *cmd_type = NULL;
	cJSON *drv_id = NULL;
	cJSON *dir = NULL;
	cJSON *password = NULL;
	int status = 0;

	cJSON *cmd = cJSON_Parse(json_string);
	if (cmd == NULL) {
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL) {
			sprintf(TxBuffer, "Error before: %s\n", error_ptr);
			//HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		}
		status = 1;
		goto end;
	}

	password = cJSON_GetObjectItemCaseSensitive(cmd,"password");
	if (strcmp(password->valuestring, "quonk") != 0){
		sprintf(TxBuffer, "Incorrect Password\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		*actuation_flag = 0;
		status = 1;
		goto end;
	}
	cmd_type = cJSON_GetObjectItemCaseSensitive(cmd, "type");
	/* If an actuate command is given, parse the target driver and the direction */
	if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring, "actuate") == 0) {
		drv_id = cJSON_GetObjectItemCaseSensitive(cmd, "driver-id");
		// if the input driver id is valid and the direction is valid, save the id and direction
		if (cJSON_IsNumber(drv_id)) {
			dir = cJSON_GetObjectItemCaseSensitive(cmd, "value");
			if (cJSON_IsNumber(dir)) {
				*driver_id = drv_id->valueint;
				*direction = dir->valueint;
				*actuation_flag = 1;

				status = 0;
			} else {
				status = 1;
				goto end;
			}
		} else {
			status = 1;
			goto end;
		}

	}
	end:
		cJSON_Delete(cmd);
		return status;
	}

