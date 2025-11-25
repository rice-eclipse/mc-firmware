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

int parse_config_interface(const char *config_str, driver **driver_list,sensor **sensor_list,monitor **monitor_list,
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby);

int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list);
int read_file_interface(const char *filename, char *data_buffer, size_t buffer_size);
int create_file_interface(const char *filename);
int mount_sd_interface(FATFS* FatFs);


#endif /* INC_INTERFACE_H_ */
