/*
 * Measurement_ECU.h
 *
 *  Created on: Dec 29, 2019
 *      Author: eng-m
 */

#ifndef APP_MEASURE_ECU_MEASUREMENT_ECU_H_
#define APP_MEASURE_ECU_MEASUREMENT_ECU_H_
/************************************************************************/
/*		                        INCLUDES                 		        */
/************************************************************************/
#include "std_types.h"
/************************************************************************/
/*		                        DEFINES                 		        */
/************************************************************************/

#define FETCH_BTN   BTN_0
#define PRINT_BTN   BTN_1

#define FETCH_PIN	PIN0
#define FETCH_PORT  GPIOA
/************************************************************************/
/*		                 FUNCTIONS' PROTOTYPES            		        */
/************************************************************************/

extern ERROR_STATUS MEASURE_ECU_Init(void);

extern ERROR_STATUS MEASURE_ECU_Update(void);


#endif /* APP_MEASURE_ECU_MEASUREMENT_ECU_H_ */
