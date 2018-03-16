/*
 * basestationNRF.c
 *
 *  Created on: Mar 16, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *   Basestation specific code for using the nRF24 wireless module.
 */




#include "basestationNRF24.h"


/*
 * Pin setters
 */



//put the nss pin corresponding to the SPI used high
void nrf24nssHigh(){
	//NSS / CSN : chip select
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

//put the nss pin corresponding to the SPI used low
void nrf24nssLow(){
	HAL_GPIO_WritePin(GPIOD, CSN_SPI3_Pin, GPIO_PIN_RESET);
}

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh(SPI_HandleTypeDef* spiHandle){
	//CE: chip enable
	if(spiHandle->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOD, CE_SPI1_Pin, GPIO_PIN_SET);

	}
	else{
		HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_SET);
	}
}

//put the ce pin corresponding to the SPI used low
void nrf24ceLow(SPI_HandleTypeDef* spiHandle){
	if(spiHandle->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOD, CE_SPI1_Pin, GPIO_PIN_RESET);

	}
	else{
		HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_RESET);
	}
}
