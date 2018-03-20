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

void initBase(SPI_HandleTypeDef* spiHandle24, uint8_t freqChannel, uint8_t address[5]){

	NRFinit(spiHandle24, nrf24nssHigh, nrf24nssLow, nrf24ceHigh, nrf24ceLow, nrf24irqRead );

	//enable TX interrupts, disable RX interrupts
	//TXinterrupts(spiHandle);

	//set interrupts
	uint8_t config_reg = readReg(CONFIG);
	config_reg &= ~MASK_RX_DR;   //enable for RX_DR
	config_reg &= ~MASK_TX_DS;  //enable for TX_DS
	config_reg |= MASK_MAX_RT; //disable for MAX_RT
	writeReg( CONFIG, config_reg);


	//set the frequency channel
	setFreqChannel(freqChannel);

	//is this overwritten again whenever we transmit?
	setDataPipes(ERX_P0);

	//set the RX buffer size to x bytes
	setRXbufferSize(12);

	//set the TX address of
	setTXaddress(address);

	//set auto retransmit: disabled
	writeReg( SETUP_RETR, 0x00);

	//enable dynamic packet length, ack payload, dynamic acks
	writeReg( FEATURE, EN_DPL | EN_ACK_PAY | EN_DYN_ACK);

	setLowSpeed(spiHandle);

	//enableAutoRetransmitSlow(spiHandle);

	//go to TX mode and be ready to listen
	powerUpTX(spiHandle);
}


uint8_t sendPacket(uint8_t packet[12]){
	//---------------------------TX loop----------------------------//
	//get data from pc
	//read address -> test
	//change TX address to address from packet -> test
	//send data
	//wait for interrupt
	//note timeout and clear interrupt bit
	//clear interrupt bits in case of succesful transmission
	//set CE low
	//send ack data back to pc

	uint8_t addressLong[5] = {0x12, 0x34, 0x56, 0x78, 0x90 + (packet[0] >> 4)};

	setTXaddress(addressLong);
	sendData(packet, 12);


	//returning last byte of address, but no call to this function ever appears to use the return value
	return addressLong[4];
}


/*
 * Pin setters
 */



//put the nss pin corresponding to the SPI used high
void nrf24nssHigh(){
	//NSS / CSN : chip select
	HAL_GPIO_WritePin(GPIOD, CSN_SPI3_Pin, GPIO_PIN_SET);
}

//put the nss pin corresponding to the SPI used low
void nrf24nssLow(){
	HAL_GPIO_WritePin(GPIOD, CSN_SPI3_Pin, GPIO_PIN_RESET);
}

//put the ce pin corresponding to the SPI used high
void nrf24ceHigh(){
	//CE: chip enable
	HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_SET);
}

//put the ce pin corresponding to the SPI used low
void nrf24ceLow(){
	HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_RESET);
}

//reading if an interrupt happened
uint8_t nrf24irqRead(){
	return !HAL_GPIO_ReadPin(GPIOD, IRQ_SPI3_Pin);
}
