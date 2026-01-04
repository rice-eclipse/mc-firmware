/*
 * interface.h
 *
 *  Created on: Nov 22, 2025
 *      Author: Deepak
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include "main.h"
#include "Fatfs.h"

int parse_config_interface(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list,
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count);

int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list);
int read_file_interface(const char *filename, char *data_buffer, size_t buffer_size);
int create_file_interface(const char *filename);
int mount_sd_interface(FATFS* FatFs);
float get_sensorval_interface(sensor *current_sensor);
void filter_and_decimate_interface(float *sensor_vals, int sensor_count);

#endif /* INC_INTERFACE_H_ */
