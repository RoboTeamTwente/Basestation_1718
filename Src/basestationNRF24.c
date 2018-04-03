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
#include "stm32f3xx_hal.h"

void initBase(SPI_HandleTypeDef* spiHandle24, uint8_t freqChannel){

	NRFinit(spiHandle24, nrf24nssHigh, nrf24nssLow, nrf24ceHigh, nrf24ceLow, nrf24irqRead );

	//set interrupts
	uint8_t config_reg = readReg(CONFIG);
	config_reg &= ~MASK_RX_DR;   //enable for RX_DR
	config_reg &= ~MASK_TX_DS;  //enable for TX_DS
	config_reg &= ~MASK_MAX_RT; //enable for MAX_RT
	writeReg(CONFIG, config_reg);


	setFreqChannel(freqChannel);
	//setLowSpeed();
	//the default value of RF_SETUP sets the module to 2 Mbps



	//setting up ACKs (with payload)
	setDataPipes(ERX_P0); //enable pipe(s). Pipe 0: ACK packets.

	//auto-ack settings
	uint8_t arc=0b1111; //auto-retransmit count
	uint8_t ard=0b0001; //auto-retransmit delay
	writeReg(SETUP_RETR, (ard<<4)|(arc&0b1111));

	//enable dynamic packet length, ack payload, dynamic acks
	/*
	 * EN_DYN_ACK enables to use the SPI command W_TX_PAYLOAD_NOACK.
	 * That means: sending a packet which does not need to be answered with an ACK
	 * when the module is otherwise configured to wait for ACKs.
	 */
	writeReg(FEATURE, EN_DPL | EN_ACK_PAY | EN_DYN_ACK);

	writeReg(DYNPD, DPL_P0); //enable dynamic packet length for data pipe x


	//go to TX mode and be ready to listen
	powerUpTX();
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

	/*
	if((readReg(FIFO_STATUS) & TX_EMPTY) != 0) {
		return 1;
	}
	*/
	uint8_t roboID = (packet[0] >> 4);

	//uint8_t addressLong[5] = {0b11010000 + (packet[0] >> 4), 0x12, 0x34, 0x56, 0x78};
	//uint8_t addressLong[5] = {0, 0, 0, 0, 0x90 + (packet[0] >> 4)};
	uint8_t addressLong[5] = {0x99, 0xB0 + roboID, 0x34, 0x56, 0x99};


	setTXaddress(addressLong);

	sendData(packet, 12);
	return 0;
}


/*
 * Pin setters
 */



//put the nss pin corresponding to the SPI used high
void nrf24nssHigh(){
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
