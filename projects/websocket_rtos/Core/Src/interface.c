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
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

static char TxBuffer[300];
#ifdef TEST_LOGIC
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
int read_file_interface(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size){

	const char *config_str2 =
	    "{"
	    "\"host\": {\"ip\": \"127.0.0.1\"},"
	    "\"password\":\"quonk\","
	    "\"port\": 1234,"
	    "\"sampling_freq_ignition\": 5000,"
	    "\"sampling_freq_standby\": 1,"
	    "\"sensors\": ["
	    "  {"
	    "    \"sensor\": \"lc1:main_loac_cell\","
	    "    \"channel\": 0,"
	    "    \"adc_cs\": 1,"
	    "    \"calibration_intercept\": 32,"
	    "    \"calibration_slope\": 1.8,"
	    "    \"enabled\": \"true\""
	    "  },"
	    "  {"
	    "    \"sensor\": \"pt2:feed_line\","
	    "    \"channel\": 1,"
	    "    \"adc_cs\": 1,"
	    "    \"calibration_intercept\": 32,"
	    "    \"calibration_slope\": 1.8,"
	    "    \"enabled\": \"true\""
	    "  },"
	    "  {"
	    "    \"sensor\": \"pt1:combustion_chamber\","
	    "    \"channel\": 0,"
	    "    \"adc_cs\": 2,"
	    "    \"calibration_intercept\": 0.4664,"
	    "    \"calibration_slope\": 1.0019,"
	    "    \"enabled\": \"true\""
	    "  }"
	    "],"
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

float get_sensorval_interface(sensor *current_sensor){
	static uint16_t adc_val = 0;
	//uint16_t adc_val = get_mcp3208_adcval(current_sensor->channel, current_sensor->adc_cs, &hspi1);
	adc_val = (adc_val + 1) % 100;
	float voltage = (adc_val)*4.096/4096;
	float sensor_val = (voltage*current_sensor->calibration_slope) + current_sensor->calibration_int;
	return sensor_val;
}
uint16_t get_mcp3208_adcval(int channel, uint16_t cs, SPI_HandleTypeDef *spiHandle){
	uint8_t tx[3];
	uint8_t rx[3];

	//start + single-ended + D2
	tx[0] = 0x06 | ((channel & 0x04) >> 2);
	//D1 + D0 shifted to B7 and B6
	tx[1] = (channel & 0x03) << 6;
	//don't care
	tx[2] = 0x00;

	HAL_GPIO_WritePin(GPIOC, cs, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spiHandle, tx, rx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, cs, GPIO_PIN_SET);

	uint16_t dataBuff = ((rx[1] & 0x0F) << 8) | rx[2];

	return dataBuff;
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
	*actuation_flag = 0;
	*ignition_flag = 0;
	*shutdown_flag = 0;
	*cancel_ignition_flag = 0;

	cJSON *cmd = cJSON_Parse(json_string);
	if (cmd == NULL) {
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL) {
			sprintf(TxBuffer, "Error before: %s\n", error_ptr);
			HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		}
		status = 1;
		goto end;
	}

	password = cJSON_GetObjectItemCaseSensitive(cmd,"password");
	if (strcmp(password->valuestring, "quonk") != 0){
		sprintf(TxBuffer, "Incorrect Password\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		status = 1;
		goto end;
	}
	cmd_type = cJSON_GetObjectItemCaseSensitive(cmd, "type");
	/* If an actuate command is given, parse the target driver and the direction */
	if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring, "Actuate") == 0) {
		drv_id = cJSON_GetObjectItemCaseSensitive(cmd, "driver_id");
		// if the input driver id is valid and the direction is valid, save the id and direction
		if (cJSON_IsString(drv_id)) {
			dir = cJSON_GetObjectItemCaseSensitive(cmd, "value");
			if (cJSON_IsBool(dir)) {
				*driver_id = atoi(drv_id->valuestring);
				*direction = (cJSON_IsTrue(dir)) ? 1:0;
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
	else if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring,"Proxima Ignition") == 0){
		*ignition_flag = 1;
	}
	else if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring,"EmergencyStop") == 0){
		*shutdown_flag = 1;
	}
	else if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring, "CancelIgnition") == 0){
		*cancel_ignition_flag = 1;
	}
	end:
		cJSON_Delete(cmd);
		return status;
	}
#else
/******************************************************************************************/
/*actual implementations*/
/******************************************************************************************/
int parse_config_interface(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list, driver *ignition,
				 char *host_ip, char *password, int *port,int *sampling_freq_ign,int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count){
	return parse_config(config_str, driver_list, sensor_list, monitor_list, ignition,
					 host_ip, password, port, sampling_freq_ign, sampling_freq_standby,
					 driver_count, sensor_count, monitor_count);

}

int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list, int *ignition_flag,
  	  	  	int *shutdown_flag, int *cancel_ignition_flag, int *actuation_flag){
	return parse_command(json_string, driver_id, direction, driver_list, ignition_flag, shutdown_flag, cancel_ignition_flag, actuation_flag);
}

int read_file_interface(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size){
	return read_file(target_file, filename, data_buffer, buffer_size);
}

int create_file_interface(FIL *target_file, char *filename){
	return create_file(target_file, filename);
}

int open_file_interface(FIL *target_file, char *filename){
		return open_file(target_file, filename);
 }

int append_file_interface(FIL *target_file, char *data, UINT btw){
	return append_file(target_file, data,btw);
}
int mount_sd_interface(FATFS* FatFs){
	return mount_sd(FatFs);
}
int close_file_interface(FIL *target_file){
	return close_file(target_file);
}
float get_sensorval_interface(sensor *current_sensor){
	return get_sensorval(current_sensor);
}
int gen_filename_interface(char *target_filename,const char *config_filename, char *target_type){
	return gen_filename(target_filename,config_filename, target_type);
}
int gen_rtc_start_params_interface(RTC_TimeTypeDef *time_field, RTC_DateTypeDef *date_field, const char *config_filename){
	return gen_rtc_start_params(time_field, date_field, config_filename);
}
int get_timestamp_interface(char *timestamp_str, int timestamp_str_size){
	return get_timestamp(timestamp_str, timestamp_str_size);
}

void filter_and_decimate_interface(float *sensor_vals, int sensor_count){
	return filter_and_decimate(sensor_vals, sensor_count);
}
#endif



/*Creates the json string to send over the websocket*/
void sensor_message_interface(char *json_buf, int json_buf_size,float *sensor_vals, int *driver_states){
	int json_len = 0;
	json_len += snprintf(json_buf + json_len, json_buf_size - json_len, "{");
	// inserts first part of each sensor json file
	const char *sensor_types[] = {"tcs", "pts", "lcs"};
	const char groups[] = {'T', 'P', 'L'};

	for (int i = 0; i < 3; i++) {
	    json_len += snprintf(json_buf + json_len, json_buf_size - json_len,
	            "\"%s\":{\"type\":\"SensorValue\",\"group_id\":%d,\"readings\":[",
	            sensor_types[i], i);
	    // inserts each sensor
	    int first_in_group = 1;  // Track if this is the first sensor in the group
	    for (int j = 0; j < sensor_count; j++) {
	        if (sensor_list[j].name[0] == groups[i]) {
	            // inserts comma only after first sensor in each sensor group
	            if (first_in_group) {
	                json_len += snprintf(json_buf + json_len, json_buf_size - json_len,
	                        "{\"sensor_id\":%d,\"reading\":%d}",
	                        sensor_list[j].channel, (int)(sensor_vals[j] * 1000));
	                first_in_group = 0;
	            }
	            else {
	                json_len += snprintf(json_buf + json_len, json_buf_size - json_len,
	                        ",{\"sensor_id\":%d,\"reading\":%d}",
	                        sensor_list[j].channel, (int)(sensor_vals[j] * 1000));
	            }
	        }
	    }
	    // Add closing bracket for readings array and closing brace for sensor group
	    json_len += snprintf(json_buf + json_len, json_buf_size - json_len, "]},");
	}

	// inserts drivers
	json_len += snprintf(json_buf + json_len, json_buf_size - json_len,
	        "\"driver\":{\"type\":\"DriverValue\",\"values\":[");
	for (int i = 0; i < driver_count; i++) {
	    if (i == driver_count - 1) {
	        json_len += snprintf(json_buf + json_len, json_buf_size - json_len, "%d", driver_states[i]);
	    }
	    else {
	        json_len += snprintf(json_buf + json_len, json_buf_size - json_len, "%d,", driver_states[i]);
	    }
	}
	json_len += snprintf(json_buf + json_len, json_buf_size - json_len, "]}}");
	if (json_len >= json_buf_size) {
	    sprintf(TxBuffer, "Buffer Overflow!\r\n");
	    HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), 100);
	}
}

