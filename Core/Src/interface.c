/*
 * interface.c
 *
 *  Created on: Nov 22, 2025
 *      Author: Deepak
 */

#include "interface.h"
#include <string.h>
#include "utils.h"
#include <stdio.h>

#ifdef TEST_LOGIC

static char TxBuffer[300];
/******************************************************************************************/
/*Interface definitions if we are only testing the RTOS logic without the physical drivers*/
/******************************************************************************************/

/*Config parsing doesn't depend on any hardware*/
int parse_config_interface(const char *config_str, driver **driver_list,sensor **sensor_list,monitor **monitor_list,
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby){
	parse_config(config_str, driver_list, sensor_list, monitor_list,
					 host_ip, port, sampling_freq_ign, sampling_freq_standby);

}

int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list){
	// CJSON variables to extract the relevant fields
	cJSON *cmd_type = NULL;
	cJSON *drv_id = NULL;
	cJSON *dir = NULL;
	int status = 0;

	cJSON *cmd = cJSON_Parse(json_string);
	if (cmd == NULL) {
		const char *error_ptr = cJSON_GetErrorPtr();
		if (error_ptr != NULL) {
			sprintf(TxBuffer, "Error before: %s\n", error_ptr);
			HAL_UART_Transmit(&huart2, (uint8_t *)TxBuffer, strlen(TxBuffer), HAL_MAX_DELAY);
		}
		status = 1;
		goto end;
	}

	cmd_type = cJSON_GetObjectItemCaseSensitive(cmd, "type");
	/* If an actuate command is given, parse the target driver and the direction */
	if (cJSON_IsString(cmd_type) && strcmp(cmd_type->valuestring, "actuate") == 0) {
		drv_id = cJSON_GetObjectItemCaseSensitive(cmd, "driver-id");
		// if the input driver id is valid and the direction is valid, save the id and direction
		if (cJSON_IsNumber(drv_id)) {
			dir = cJSON_GetObjectItemCaseSensitive(cmd, "direction");
			if (cJSON_IsNumber(dir)) {
				*driver_id = drv_id->valueint;
				*direction = dir->valueint;
				sprintf(TxBuffer, "Actuating Driver %d. Direction: %d", driver_list[*driver_id].GPIO_Pin, *direction);
				HAL_UART_Transmit(&huart2, (uint8_t *)TxBuffer, strlen(TxBuffer), -1);
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
}

int read_file_interface(const char *filename, char *data_buffer, size_t buffer_size){

	const char *config_str2 =
        "{"
        "\"host\": {\"ip\": \"127.0.0.1\"},"
        "\"port\": 1234,"
        "\"sampling_freq_ignition\": 10,"
        "\"sampling_freq_standby\": 1,"
        "\"sensors\": ["
        "  {"
        "    \"enabled\": \"true\","
        "    \"sensor\": \"test_sensor\","
        "    \"channel\": 0,"
        "    \"adc_cs\": 1,"
        "    \"calibration_intercept\": 0.0,"
        "    \"calibration_slope\": 1.0"
        "  }"
        "],"
        "\"drivers\": [],"
        "\"ignition\": {\"gpio_port\":\"GPIOA\",\"gpio_pin\":0},"
        "\"monitors\": []"
        "}";

	int config_len = strlen(config_str2);
	if (config_len > buffer_size){
		sprintf(TxBuffer, "Error: Input buffer size too small \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)TxBuffer, strlen(TxBuffer), -1);
		return -1;
	} else{
		strcpy(data_buffer, config_str2);
	}
	return 0;
}


int create_file_interface(const char *filename){
	sprintf(TxBuffer,"[TEST] file created successfully \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)TxBuffer, strlen(TxBuffer), -1);
	return 0;
}

int mount_sd_interface(FATFS* FatFs){
	sprintf(TxBuffer, "Mounted sd \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)TxBuffer, strlen(TxBuffer), -1)
	return 0;
}
#else

/******************************************************************************************/
/*actual implementations*/
/******************************************************************************************/
int parse_config_interface(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list,
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count){
	return parse_config(config_str, driver_list, sensor_list, monitor_list,
					 host_ip, port, sampling_freq_ign, sampling_freq_standby,
					 driver_count, sensor_count, monitor_count);

}

int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list){
	return parse_command(json_string, driver_id, direction, driver_list);
}

int read_file_interface(const char *filename, char *data_buffer, size_t buffer_size){
	return read_file(filename, data_buffer, buffer_size);
}

int create_file_interface(const char *filename){
	return create_file(filename);
}
int mount_sd_interface(FATFS* FatFs){
	return mount_sd_interface(FatFs);
}
#endif
