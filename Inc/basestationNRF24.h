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

//------------------------------ initTX ------------------------------//
//rest and init
//mask right interrupts
//set frequency
//enable ack pipe
//set RX buffer size
//set TX address + pipe 0 address
//powerUpTX
void initBase(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel);


//sends a packet from the basestation to a robot
uint8_t sendPacket(uint8_t packet[12]);



/*
 * Pin setters
 */

//put the nss pin corresponding to the SPI used high
void nrf24nssHigh();

//put the nss pin corresponding to the SPI used low
void nrf24nssLow();

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh();

//put the ce pin corresponding to the SPI used low
void nrf24ceLow();

uint8_t nrf24irqRead();

#endif /* BASESTATIONNRF24_H_ */
