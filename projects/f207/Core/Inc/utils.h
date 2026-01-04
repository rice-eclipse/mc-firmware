/*
 * utils.h
 *
 *  Created on: Nov 9, 2025
 *      Author: Deepak
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_
#include "main.h"
#include "fatfs.h"
#include "cJSON.h"
#include "usart.h"

int parse_config(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list,
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count);

int parse_command(const char* json_string, int* driver_id, int* direction, driver *driver_list);
int read_file(const char *filename, char *data_buffer, size_t buffer_size);
int create_file(const char *filename);
int mount_sd(FATFS* FatFs);
float get_sensorval(sensor *current_sensor);
void filter_and_decimate(float *sensor_vals, int sensor_count);

#endif /* INC_UTILS_H_ */
