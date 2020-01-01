/*
 * softwareDelay.c
 *
 *  Created on: Dec 3, 2019
 *      Author: eng-m
 */
#include "softwareDelay.h"


/**
 * Description: this delay function operate in a polling manner
 * 				don't use it with RTOSs
 * @param n: the milli-seconds
 */
void SwDelay_ms(uint32 n){
	/**
	 * each machine cycle took 12 clock cycle
	 * assume clock 16MHz ->  so machine cycle = 0.75 us
	 * so 1ms require almost 1333 machine cycle
	 * at each cycle the loop compare and then increment then jump
	 * and also load the register more than one time number of instruction in the loop
	 * takes about 8 machine cycle
	 */
	uint32 index = 0;
	for( index=0;index < n*165 ;index++)
	{

	}
}


