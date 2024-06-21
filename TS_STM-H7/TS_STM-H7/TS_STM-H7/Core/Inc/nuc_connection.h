/*
 * nuc_connection.h
 *
 *  Created on: May 10, 2024
 *      Author: Andrea.Cioncolini
 */

#ifndef INC_NUC_CONNECTION_H_
#define INC_NUC_CONNECTION_H_
#include <stdint.h>
/* Definitions for NUC command */

/* Functions */
//DSTATUS SD_disk_initialize (BYTE pdrv); esempio preso dalla SD card

//void send_request_file_name( uint8_t rx_usb[]);
void get_file_name();
void get_nuc_time_stamp();
void send_complete_time_stamp (char* time_stamp_TS_buffer);

#endif /* INC_NUC_CONNECTION_H_ */
