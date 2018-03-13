/*
 * myNRF24basic.h
 *
 *  Created on: Mar 13, 2018
 *      Author: Ulf Stottmeister
 *
 *
 *  This is the interface for the low level functions on the nRF24L01 wireless module.
 */

#ifndef MYNRF24BASIC_H_
#define MYNRF24BASIC_H_

#include "bitops.h"
#include <inttypes.h>
#include "spi.h"
#include "myNRF24.h"




//read the interrupt pin
uint8_t irqRead(SPI_HandleTypeDef* spiHandle);

//returns 0 on success; -1 on error
int8_t clearInterrupts(SPI_HandleTypeDef* spiHandle);

//write to a register and output debug info to the terminal
void writeRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data);

//write to a register
//returns 0 on success; -1 on error
int8_t writeReg(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data);

//write to a multi-byte register and output debug info to the terminal
void writeRegMultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size);

/**
  * @brief  Write to a multi-byte register.
  * @param  pointer to spi handle
  * @param  register to write to
  * @param  Array of bytes with data to write
  * @param  Size of Data Array
  * @retval Error Status. No error: 0; on error: -1
  */
int8_t writeRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* pdata, uint8_t size);

//read a register and output debug info to the terminal
uint8_t readRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg);

//read a register
uint8_t readReg(SPI_HandleTypeDef* spiHandle, uint8_t reg);

//read a multi-byte register and output debug info to terminal
//output will be stored in the array dataBuffer
void readRegMultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size);

//read a multi-byte register
//output will be stored in the array dataBuffer
//returns 0 on success; -1 on error
int8_t readRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size);






//put the nss pin corresponding to the SPI used high
void nssHigh(SPI_HandleTypeDef* spiHandle);
//put the nss pin corresponding to the SPI used low
void nssLow(SPI_HandleTypeDef* spiHandle);
//put the ce pin corresponding to the SPI used high
void ceHigh(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used low
void ceLow(SPI_HandleTypeDef* spiHandle);






#endif /* MYNRF24BASIC_H_ */
