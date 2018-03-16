/*
 * basestationNRF24.h
 *
 *  Created on: Mar 16, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *   Basestation specific code for using the nRF24 wireless module.
 */

#ifndef BASESTATIONNRF24_H_
#define BASESTATIONNRF24_H_

#include "myNRF24.h"


/*
 * Pin setters
 */



//put the nss pin corresponding to the SPI used high
void nssHigh(SPI_HandleTypeDef* spiHandle);

//put the nss pin corresponding to the SPI used low
void nssLow(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used high
void ceHigh(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used low
void ceLow(SPI_HandleTypeDef* spiHandle);



#endif /* BASESTATIONNRF24_H_ */
