/*
 * interface.h
 *
 *  Created on: Feb 9, 2026
 *      Author: Deepak
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include "main.h"
#include "fatfs.h"
#include "spi.h"


#define unmount_sd_interface() mount_sd_interface(0)

int parse_config_interface(const char *config_str, driver *driver_list,sensor *sensor_list,monitor *monitor_list, driver *ignition,
				 char *host_ip,char *password, int *port,int *sampling_freq_ign,int *sampling_freq_standby,
				 int *driver_count, int *sensor_count, int *monitor_count);
int read_file_interface(FIL *target_file, const char *filename, char *data_buffer, size_t buffer_size);
int mount_sd_interface(FATFS* FatFs);
int open_file_interface(FIL *target_file, char *filename);
int append_file_interface(FIL *target_file, char *data, UINT btw);
int create_file_interface(FIL *target_file, char *filename);
int close_file_interface(FIL *target_file);
float get_sensorval_interface(sensor *current_sensor);
int parse_command_interface(const char* json_string, int* driver_id, int* direction, driver *driver_list, int *ignition_flag,
  	  	  	int *shutdown_flag, int *cancel_ignition_flag, int *actuation_flag);
void filter_and_decimate_interface(float *sensor_vals, int sensor_count);
void sensor_message_interface(char *json_buf, int json_buf_size,float *sensor_vals, int *driver_states);
uint16_t get_mcp3208_adcval(int channel, uint16_t cs, SPI_HandleTypeDef *spiHandle);
int gen_rtc_start_params_interface(RTC_TimeTypeDef *time_field, RTC_DateTypeDef *date_field, char *config_filename);
int gen_filename_interface(char *target_filename,char *config_filename, char *target_type);
#endif /* INC_INTERFACE_H_ */
