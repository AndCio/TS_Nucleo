/*
 * total_station_interface.h
 *
 *  Created on: May 11, 2024
 *      Author: Andrea.Cioncolini
 */

#ifndef INC_TOTAL_STATION_INTERFACE_H_
#define INC_TOTAL_STATION_INTERFACE_H_

typedef struct
{
  uint32_t a;       				/*!< tempo di andata */
  uint32_t b;       				/*!< tempo di ritorno */
  uint32_t c;        				/*!< delta */
  uint32_t d;        	/*!< tempo di ritorno compensato */

} ForTime;

int 	search_prism(int search_prism_flag);
ForTime get_TS_time_stamp();
void 	start_measure();
void 	stop_measure();
void 	start_distance();
void 	saving_data();
#endif /* INC_TOTAL_STATION_INTERFACE_H_ */
