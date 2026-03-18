/*
 * utils.c
 *
 *  Created on: Nov 9, 2025
 *      Author: Deepak
 */
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static char TxBuffer[300];

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

int parse_config(const char *config_str,
                 driver *driver_list,
                 sensor *sensor_list,
                 monitor *monitor_list,
				 driver *ignition,
                 char *host_ip,
				 char *password,
                 int *port,
                 int *sampling_freq_ign,
                 int *sampling_freq_standby,
				 int *driver_count,
				 int *sensor_count,
				 int *monitor_count)
{
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
		                if (strcmp(gpio_port, "GPIOF") == 0) {
		                    new_driver.GPIO_Port = GPIOF;
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

int parse_command(const char *json_string, int *driver_id, int *direction, driver *driver_list, int *ignition_flag,
				  int *shutdown_flag, int *cancel_ignition_flag, int *actuation_flag){
		int status = 0;
		*actuation_flag = 0;
		*ignition_flag = 0;
		*shutdown_flag = 0;
		*cancel_ignition_flag = 0;

		jsmn_parser p;
		    jsmntok_t t[64]; // Ensure this is large enough for your expected JSON complexity

		    jsmn_init(&p);
		    int r = jsmn_parse(&p, json_string, strlen(json_string), t, sizeof(t) / sizeof(t[0]));

		    if (r < 0) {
		        sprintf(TxBuffer, "Failed to parse JSON: %d\r\n", r);
		        HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		        return 1;
		    }

		    // Assume the top-level element is an object
		    if (r < 1 || t[0].type != JSMN_OBJECT) {
		        return 1;
		    }

		    // Token pointers to store the locations of our values
		    jsmntok_t *type_tok = NULL;
		    jsmntok_t *drv_id_tok = NULL;
		    jsmntok_t *val_tok = NULL;
		    int pass_valid = 0;

		    // Loop through all keys of the flat root object
		    // i=1 is the first key. We increment by 2 to jump from key to the next key.
		    for (int i = 1; i < r; i += 2) {
		        if (jsoneq(json_string, &t[i], "password") == 0) {
		            if (jsoneq(json_string, &t[i + 1], "quonk") == 0) {
		                pass_valid = 1;
		            }
		        } else if (jsoneq(json_string, &t[i], "type") == 0) {
		            type_tok = &t[i + 1];
		        } else if (jsoneq(json_string, &t[i], "driver_id") == 0) {
		            drv_id_tok = &t[i + 1];
		        } else if (jsoneq(json_string, &t[i], "value") == 0) {
		            val_tok = &t[i + 1];
		        }
		    }

		    // 1. Check Password
		    if (!pass_valid) {
		        sprintf(TxBuffer, "Incorrect Password\r\n");
		        HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		        return 1;
		    }
		    if (type_tok != NULL) {
		        if (jsoneq(json_string, type_tok, "Actuate") == 0) {

		            // Check if we got both driver_id and value tokens
		            if (drv_id_tok != NULL && val_tok != NULL) {

		                char id_str[16] = {0};
		                int id_len = drv_id_tok->end - drv_id_tok->start;
		                if (id_len < sizeof(id_str)) {
		                    strncpy(id_str, json_string + drv_id_tok->start, id_len);
		                    *driver_id = atoi(id_str);
		                } else {
		                    return 1;
		                }

		                // Extract boolean value
		                if (val_tok->type == JSMN_PRIMITIVE) {
		                    if (json_string[val_tok->start] == 't') {
		                        *direction = 1;
		                    } else if (json_string[val_tok->start] == 'f') {
		                        *direction = 0;
		                    } else {
		                        return 1;
		                    }
		                } else {
		                    return 1;
		                }

		                *actuation_flag = 1;
		                status = 0;
		            } else {
		                status = 1;
		            }

		        } else if (jsoneq(json_string, type_tok, "Proxima Ignition") == 0) {
		            *ignition_flag = 1;
		        } else if (jsoneq(json_string, type_tok, "EmergencyStop") == 0) {
		            *shutdown_flag = 1;
		        } else if (jsoneq(json_string, type_tok, "CancelIgnition") == 0) {
		            *cancel_ignition_flag = 1;
		        } else {
		            status = 1;
		        }
		    } else {
		        status = 1;
		    }

		    return status;
}

/* ========== SD + FILE HELPERS WITH TEST PLACEHOLDERS ========== */

int read_file(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size)
{

	FRESULT fres;
	fres = f_open(target_file, filename, FA_READ);
	   if(fres != FR_OK) {
		sprintf(TxBuffer,"f_open error (%i)\r\n", fres);
		HAL_UART_Transmit(&huart3, (uint8_t*)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		return -2;
	   }

	   //get the number of characters to allocate to this string
	   long size = f_size(target_file);
	   if (size > buffer_size){
		   sprintf(TxBuffer, "Error: Input buffer size too small \r\n");
		   HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		   f_close(target_file);
		   return -2;
	   }
	   UINT br = 0;
	   FRESULT rres = f_read(target_file,(void *)data_buffer,size, &br);
	   if(rres != FR_OK) {
		   sprintf(TxBuffer,"f_gets error (%i)\r\n", fres);
		   HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer),HAL_MAX_DELAY);
		   f_close(target_file);
		   return -2;
	   }
	   if (br != size){
		   sprintf(TxBuffer,"File not read fully!\r\n");
		   HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
	   }
	   f_close(target_file);
	   return 0;
}

int create_file(FIL *target_file,char *filename)
{
    FRESULT fres;
    fres = f_open(target_file, filename, FA_WRITE |FA_CREATE_NEW);
    if (fres != FR_OK) {
        sprintf(TxBuffer, "f_open error (%i)\r\n", fres);
        HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
        return -2;
    }
    f_close(target_file);
    return 0;
}

int open_file(FIL *target_file, char *filename){
	FRESULT fres;
	fres = f_open(target_file, filename, FA_OPEN_ALWAYS|FA_WRITE);
	if (fres != FR_OK){
		sprintf(TxBuffer, "Failed to open %s for appending\r\n", filename);
		HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		return -2;
	}
	return 0;
}

int append_file(FIL *target_file, char *data, UINT btw){
	FRESULT fres;
	UINT bytes_written;
	fres = f_write(target_file, data, btw, &bytes_written);
	if (fres != FR_OK){
		return -2;

	}
	return 0;
}

int mount_sd(FATFS *Fatfs)
{	FRESULT fres;
    fres = (f_mount(Fatfs, "", 1));
    if (fres != FR_OK){
    	sprintf(TxBuffer, "f_mount error (%i)\r\n", fres);
    	HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
    	return -1;
    }
    return 0;
}

int close_file(FIL *target_file){
	return f_close(target_file);
}
float get_sensorval(sensor *current_sensor){
	static uint16_t adc_val = 0;
	//uint16_t adc_val = get_mcp3208_adcval(current_sensor->channel, current_sensor->adc_cs, &hspi1);
	adc_val = (adc_val + 10) % 200;
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
int gen_rtc_start_params(RTC_TimeTypeDef *time_field, RTC_DateTypeDef *date_field, char *config_filename){
	FILINFO fno;
		FRESULT fres;
		fres = f_stat(config_filename, &fno);
		if (fres != FR_OK){
			return -1;
		}
		date_field->Year = (uint8_t)((fno.fdate >> 9) & 0x7F) -20;
		date_field->Month = (fno.fdate >> 5) & 0xF;
		date_field->Date = fno.fdate & 0x1F;
		time_field->Hours = (fno.ftime >> 11) & 0x1F;
		time_field->Minutes = (fno.ftime >> 5) & 0x3F;
		time_field->Seconds = ((fno.ftime) & 0x1F) *2;
		return 0;

}
int gen_filename(char *target_filename,char *config_filename, char *target_type){
	FILINFO fno;
	FRESULT fres;
	fres = f_stat(config_filename, &fno);
	if (fres != FR_OK){
		return -1;
	}
	int year = ((fno.fdate >> 9) & 0x7F) + 1980;
	int month = (fno.fdate >> 5) & 0xF;
	int date = fno.fdate & 0x1F;
	int hour = (fno.ftime >> 11) & 0x1F;
	int minute = (fno.ftime >> 5) & 0x3F;
	int second = ((fno.ftime) & 0x1F) *2;

	int bytes_written;
	const char *extension = (target_type[0] == 'l') ? ".txt" : ".csv";
	bytes_written = snprintf(target_filename, MAX_FILENAME_LENGTH, "%s_%d_%d_%d_%d-%d-%d%s", target_type,
			 month, date, year, hour, minute, second,extension);
	if (bytes_written < 0 || bytes_written >= MAX_FILENAME_LENGTH){
			return -1;
		}
	/*checks if the file already exists and creates a copy if necessary*/
	fres = f_stat(target_filename, &fno);
	if (fres == FR_NO_FILE){
		return 0;
	}
	int copy  = 1;
	while (1){
		bytes_written = snprintf(target_filename, MAX_FILENAME_LENGTH, "%s_%d_%d_%d_%d-%d-%d_%d%s", target_type,
					 month, date, year, hour, minute, second,copy,extension);
		if (bytes_written < 0 || bytes_written > MAX_FILENAME_LENGTH){
				return -1;
			}
		fres = f_stat(target_filename,&fno);
		if (fres == FR_NO_FILE){
			return 0;
		}
		else if (fres == FR_OK){
			copy++;
		}
		else{
			return -1;
		}
	}

	return 0;
}
void filter_and_decimate(float *sensor_vals, int sensor_count){
	return;
}

void ignition_sequence(){
	HAL_GPIO_WritePin(ignition.GPIO_Port, ignition.GPIO_Pin, 1);
	return;
}
