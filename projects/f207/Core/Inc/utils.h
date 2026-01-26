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
#include "spi.h"


int parse_config(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list, driver *ignition,
				 char *host_ip, char* password, int *port, int *sampling_freq_ign, int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count);

int parse_command(const char* json_string, int* driver_id, int* direction, driver *driver_list, int *ignition_flag,
				  int *shutdown_flag, int *cancel_ignition_flag);
int read_file(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size);
int create_file(FIL *target_file,const char *filename);
int open_file (FIL *target_file, const char *filename);
int append_file(FIL *target_file, char *data, UINT btw);
int mount_sd(FATFS* FatFs);
int close_file(FIL *target_file);
float get_sensorval(sensor *current_sensor);
uint16_t get_mcp3208_adcval(int channel, uint16_t cs, SPI_HandleTypeDef *spiHandle);
/*TODO: Need to implement filtering and decimation */
void filter_and_decimate(float *sensor_vals, int sensor_count);
/*TODO: Need to implement countdown sequence */
void ignition_sequence();
#endif /* INC_UTILS_H_ */
