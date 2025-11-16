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
				 char *host_ip,int *port,int *sampling_freq_ign,int *sampling_freq_standby);

int parse_command(const char* json_string, int* driver_id, int* direction);
char *read_file(const char *filename);
char *create_file(const char *filename);
int mount_sd(FATFS* FatFs);


#endif /* INC_UTILS_H_ */
