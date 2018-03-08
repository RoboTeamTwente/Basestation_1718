/*
' * myNRF24.c
 *
 *  Created on: 19 sep. 2016
 *      Author: Hans-van-der-Heide
 */


/*
 * see datasheet for info:
 * https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf
 * Important pages:
 * page 48 command words to chip
 * figure 23 on page 49: spi operations
 * page 54 register map
 * figure 3 on page 21: state diagram
 */

#include "myNRF24.h"
#include <string.h>

//*************************auxillery functions**********************************//
//*******************not actually for NRF24 control*****************************//

//blink leds for debugging purposes
//blink leds for debugging purposes
void fun(){
	  //HAL_GPIO_WritePin(GPIOE, LD3_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_RESET);
	//  HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	 // HAL_GPIO_WritePin(GPIOE, LD6_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_RESET);
	 // HAL_GPIO_WritePin(GPIOE, LD10_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	 // HAL_GPIO_WritePin(GPIOE, LD10_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_RESET);
	 // HAL_GPIO_WritePin(GPIOE, LD7_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	 // HAL_GPIO_WritePin(GPIOE, LD7_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, LD5_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOE, LD5_Pin, GPIO_PIN_RESET);
	 // HAL_GPIO_WritePin(GPIOE, LD3_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);
}

void fun2(){
	HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, LD5_Pin, GPIO_PIN_SET);
}

void fun2out(){
	HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, LD5_Pin, GPIO_PIN_RESET);
}


//set a specific bit in a byte to a 1 or a 0
uint8_t setBit(uint8_t byte, uint8_t position, uint8_t value){
	uint8_t result;

	if(value == 1){
		result = byte | (1 << position); //write a 1 on the right position
	}
	else if(value == 0){
		uint8_t temp = 0xFF;
		temp -= (1 << position); //have a byte with all 1's, except the position you write a 0 to
		result = byte & temp; // write a zero to the right position
	}
	else{
		TextOut("Error! value should be 1 or 0 \n");
		result = 0xFF;
	}

	return result;
}

//check if a specific bit in a byte is 1
uint8_t readBit(uint8_t byte, uint8_t position){
	uint8_t mask = 0;
	mask = mask | (1 << position);
	byte = byte & mask;
	if(byte == 0){
		return 0;
	}
	else{
		return 1;
	}
}

//*****************************low level library********************************//
//******************the user is not supposed to use these***********************//

//put the nss pin corresponding to the SPI used high
void nssHigh(SPI_HandleTypeDef* spiHandle){
	if(spiHandle->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	}
	else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	}
}

//put the nss pin corresponding to the SPI used low
void nssLow(SPI_HandleTypeDef* spiHandle){
	//HAL_Delay(1);
	if(spiHandle->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOD, CSN_SPI1_Pin, GPIO_PIN_RESET);

	}
	else{
		HAL_GPIO_WritePin(GPIOD, CSN_SPI3_Pin, GPIO_PIN_RESET);
	}
}

//put the ce pin corresponding to the SPI used high
void ceHigh(SPI_HandleTypeDef* spiHandle){
	if(spiHandle->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOD, CE_SPI1_Pin, GPIO_PIN_SET);

	}
	else{
		HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_SET);
	}
}

//put the ce pin corresponding to the SPI used low
void ceLow(SPI_HandleTypeDef* spiHandle){
	if(spiHandle->Instance == SPI1){
		HAL_GPIO_WritePin(GPIOD, CE_SPI1_Pin, GPIO_PIN_RESET);

	}
	else{
		HAL_GPIO_WritePin(GPIOD, CE_SPI3_Pin, GPIO_PIN_RESET);
	}
}

//read the interrupt pin
uint8_t irqRead(SPI_HandleTypeDef* spiHandle){
	if(spiHandle->Instance == SPI1)
		if(!HAL_GPIO_ReadPin(GPIOA, IRQ_SPI1_Pin)){
		//if (HAL_GPIO_ReadPin(IRQ_SPI1_GPIO_Port, IRQ_SPI1_Pin)){
			return 1;
		}
		else{
			return 0;
		}
	else{
		if(!HAL_GPIO_ReadPin(GPIOD, IRQ_SPI3_Pin)){
		//if (HAL_GPIO_ReadPin(IRQ_SPI1_GPIO_Port, IRQ_SPI1_Pin)){
			return 1;
		}
		else{
			return 0;
		}
	}
}

//write to a register and output debug info to the terminal
void writeRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data){
	if(reg == 	RX_PW_P0 || reg == RX_ADDR_P1 || reg == TX_ADDR){
		TextOut("Error, this is a multi-byte register. use writeRegMultiDebug instead\n");
	}
	else if(reg > FEATURE || reg == OBSERVE_TX || reg == RPD || (reg > FIFO_STATUS && reg < DYNPD)){
		TextOut("Error, invalid register. It is either read only or non-existing\n");
	}
	else{
		  //commands can only be given after a falling edge of the nss pin
		  //see figure 23 of datasheet

		nssLow(spiHandle);
		uint8_t sendData = setBit(reg, 5, 1); // W_REGISTER = 001A AAAA -> AAAAA = 5 bit register address
		uint8_t receiveData;
		uint8_t SPIstatus;

		//comand: write to register reg and get status register
		SPIstatus = HAL_SPI_TransmitReceive(spiHandle, &sendData, &receiveData, 1, 100);

		sprintf(smallStrBuffer, "writing to reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		sendData = data;
		//send data to the register
		SPIstatus = HAL_SPI_TransmitReceive(spiHandle, &sendData, &receiveData, 1, 100);

		sprintf(smallStrBuffer, "writing to reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		nssHigh(spiHandle);
		HAL_Delay(10);
	}
}

//write to a register
void writeReg(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data){
	if(reg == 0x0A || reg == 0x0B || reg == 0x10){
		TextOut("Error, this is a multi-byte register. use writeRegMulti instead\n");
	}
	else if(reg > 0x1D || reg == 0x08 || reg == 0x09 || (reg >= 0x17 &&reg <= 0x1B)){
		TextOut("Error, invalid register. It is either read only or non-existing\n");
	}
	else{
		  //commands can only be given after a falling edge of the nss pin
		  //see figure 23 of datasheet
		nssLow(spiHandle);

		uint8_t sendData = setBit(reg, 5, 1); // W_REGISTER = 001A AAAA -> AAAAA = 5 bit register address

		//comand: write to register reg
		HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);
		sendData = data;
		//send data to the register
		HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);

		nssHigh(spiHandle);
		//HAL_Delay(10);
	}
}

//write to a multi-byte register and output debug info to the terminal
void writeRegMultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size){
	if(!(reg == 0x0A || reg == 0x0B || reg == 0x10)){
		TextOut("Error, invalid register. It is either read only, single byte or non-existingd\n");
	}
	else if(size > 5){
		TextOut("Error, size can never be bigger than 5\n");
	}
	else{
		  //commands can only be given after a falling edge of the nss pin
		  //see figure 23 of datasheet
		nssLow(spiHandle);

		uint8_t command = setBit(reg, 5, 1); // W_REGISTER = 001A AAAA -> AAAAA = 5 bit register address
		uint8_t receiveData;
		uint8_t SPIstatus;

		//comand: write to register reg and get status register
		SPIstatus = HAL_SPI_TransmitReceive(spiHandle, &command, &receiveData, 1, 100);
		sprintf(smallStrBuffer, "writing to reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		//Do not remove the i
		//it invokes divine intervention
		int i = 0;

		//send data to the register
		SPIstatus = HAL_SPI_TransmitReceive(spiHandle, data, &receiveData, size, 100);

		sprintf(smallStrBuffer, "writing to reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		nssHigh(spiHandle);
		HAL_Delay(10);
	}
}

//write to a multi-byte register
void writeRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size){
	if(!(reg == 0x0A || reg == 0x0B || reg == 0x10)){
		TextOut("Error, invalid register. It is either read only, single byte or non-existingd\n");
	}
	else if(size > 5){
		TextOut("Error, size can never be bigger than 5\n");
	}
	else{
		//commands can only be given after a falling edge of the nss pin
		//see figure 23 of datasheet
		nssLow(spiHandle);

		uint8_t command = setBit(reg, 5, 1); // W_REGISTER = 001A AAAA -> AAAAA = 5 bit register address
		uint8_t receiveData;

		//comand: write to register reg and get status register
		HAL_SPI_TransmitReceive(spiHandle, &command, &receiveData, 1, 100);

		//Do not remove the i
		//it invokes divine intervention
		int i = 0;

		//send data to the register
		HAL_SPI_TransmitReceive(spiHandle, data, &receiveData, size, 100);

		nssHigh(spiHandle);
		//HAL_Delay(10);
	}
}

//read a register and output debug info to the terminal
uint8_t readRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg){
	if(reg > 0x1D){
		TextOut("Error, invalid register\n");
		return 0xF0;
	}
	else{
		//commands can only be given after a falling edge of the nss pin
		//see figure 23 of datasheet
		nssLow(spiHandle);

		//command: read reg 5
		uint8_t sendData = reg; //R_REGISTER = 000A AAAA -> AAAAA = 5 bit register address
		uint8_t SPIstatus;
		uint8_t receiveData;
		//command: read from register reg and get status register
		SPIstatus = HAL_SPI_TransmitReceive(spiHandle, &sendData, &receiveData, 1, 100);

		sprintf(smallStrBuffer, "read reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		//read data from the register
		SPIstatus = HAL_SPI_Receive(spiHandle, &receiveData, 1, 100);

		sprintf(smallStrBuffer, "reading reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		nssHigh(spiHandle);
		HAL_Delay(10);

		return receiveData;
	}
}

//read a register
uint8_t readReg(SPI_HandleTypeDef* spiHandle, uint8_t reg){
	if(reg > 0x1D){
		TextOut("Error, invalid register\n");
		return 0xF0;
	}
	else{
		//commands can only be given after a falling edge of the nss pin
		//see figure 23 of datasheet
		nssLow(spiHandle);

		uint8_t sendData = reg; //R_REGISTER = 000A AAAA -> AAAAA = 5 bit register address
		uint8_t receiveData;
		//command: read from register reg
		HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);

		//read data from the register
		HAL_SPI_Receive(spiHandle, &receiveData, 1, 100);

		nssHigh(spiHandle);
		//HAL_Delay(10);

		return receiveData;
	}
}

//read a multi-byte register and output debug info to terminal
//output will be stored in the array dataBuffer
void readRegMultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size){
	if(reg > 0x1D){
		TextOut("Error, invalid register\n");
	}
	else{
		//commands can only be given after a falling edge of the nss pin
		//see figure 23 of datasheet
		nssLow(spiHandle);

		//command: read reg 5
		uint8_t sendData = reg; //R_REGISTER = 000A AAAA -> AAAAA = 5 bit register address
		uint8_t SPIstatus;
		uint8_t receiveData;
		//command: read from register reg and get status register
		SPIstatus = HAL_SPI_TransmitReceive(spiHandle, &sendData, &receiveData, 1, 100);

		sprintf(smallStrBuffer, "writing to reg; spi status = %i", SPIstatus);
		TextOut(smallStrBuffer);
		sprintf(smallStrBuffer, "status reg = %x\n", receiveData);
		TextOut(smallStrBuffer);

		//read data from the register
		SPIstatus = HAL_SPI_Receive(spiHandle, dataBuffer, 5, 100);

		sprintf(smallStrBuffer, "reading reg; spi status = %i\n", SPIstatus);
		TextOut(smallStrBuffer);
		for(int i = 0; i < 5; i++){
			sprintf(smallStrBuffer, "reg = %x\n", dataBuffer[i]);
			TextOut(smallStrBuffer);
		}
		TextOut("\n");

		nssHigh(spiHandle);
		HAL_Delay(10);

	}
}

//read a multi-byte register
//output will be stored in the array dataBuffer
void readRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size){
	if(reg > 0x1D){
		TextOut("Error, invalid register\n");
	}
	else{
		//commands can only be given after a falling edge of the nss pin
		//see figure 23 of datasheet
		nssLow(spiHandle);

		//command: read reg 5
		uint8_t sendData = reg; //R_REGISTER = 000A AAAA -> AAAAA = 5 bit register address
		//command: read from register reg and get status register
		HAL_SPI_Transmit(spiHandle, &sendData,1, 100);

		//read data from the register
		HAL_SPI_Receive(spiHandle, dataBuffer, 5, 100);

		nssHigh(spiHandle);

		HAL_Delay(10);

	}
}



//****************************high level library**************************//
//********************the user may use these functions********************//

//--------------------initialization and configuration--------------------//

//reset to reset value on page 54
void reset(SPI_HandleTypeDef* spiHandle){
	uint8_t multRegData[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

	// see page 54 and further for reset values
	writeReg(spiHandle, 0x00, 0x08); // CONFIG
	writeReg(spiHandle, 0x01, 0x3F); // EN_AA
	writeReg(spiHandle, 0x02, 0x03); // EN_RXADDR
	writeReg(spiHandle, 0x03, 0x03); // SETUP_AW
	writeReg(spiHandle, 0x04, 0x03); // SETUP_RETR
	writeReg(spiHandle, 0x05, 0x02); // RF_CH
	writeReg(spiHandle, 0x06, 0x0F); // RF_SETUP
	writeReg(spiHandle, 0x07, 0x7E); // STATUS //weird things with clear interrups going on here. see datashet
	//register 0x08 and 0x09 are read only
	writeRegMulti(spiHandle, 0x0A, multRegData, 5); // RX_ADDR_P0

	for(int i = 0; i < 5; i++){multRegData[i] = 0xC2;}
	writeRegMulti(spiHandle, 0x0B, multRegData, 5); // RX_ADDR_P1
	writeReg(spiHandle, 0x0C, 0xC3); // RX_ADDR_P2
	writeReg(spiHandle, 0x0D, 0xC4); // RX_ADDR_P3
	writeReg(spiHandle, 0x0E, 0xC5); // RX_ADDR_P4
	writeReg(spiHandle, 0x0F, 0xC6); // RX_ADDR_P5
	for(int i = 0; i < 5; i++){multRegData[i] = 0xE7;}
	writeRegMulti(spiHandle, 0x10, multRegData, 5); // TX_ADDR
	writeReg(spiHandle, 0x11, 0x00); // RX_PW_P0
	writeReg(spiHandle, 0x12, 0x00); // RX_PW_P1
	writeReg(spiHandle, 0x13, 0x00); // RX_PW_P2
	writeReg(spiHandle, 0x14, 0x00); // RX_PW_P3
	writeReg(spiHandle, 0x15, 0x00); // RX_PW_P4
	writeReg(spiHandle, 0x16, 0x00); // RX_PW_P5
	//reg 17-1B are read only or non-existing
	writeReg(spiHandle, 0x15, 0x00); // DYNPD
	writeReg(spiHandle, 0x16, 0x00); // FEATURE

}

//initialize the system:
//reset it and enable pipe 1 and 0
//set pipeWith to 1
//flush TX and RX buffer
void NRFinit(SPI_HandleTypeDef* spiHandle){
	//reset system
	reset(spiHandle);

	//enable RX pipe 0 and 1, disable all other pipes
	writeReg(spiHandle, 0x02, 0x03);

	//set RX pipe with of pipe 0 to 1 byte.
	writeReg(spiHandle, 0x11, 0x01);

	//set RX pipe with of pipe 1 to 1 byte.
	writeReg(spiHandle, 0x12, 0x01);

	flushRX(spiHandle);
	flushTX(spiHandle);

}


//set the address you will send to
//pipe 0 is reserved for acks: it's adress always equals the TX address and is set with setTXaddress
void setRXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5], uint8_t pipeNumber){
	if(pipeNumber == 0){
		TextOut("Error: pipe 0 is reserved for acks\n");
	}
	else if(pipeNumber > 5){
		TextOut("Error: max pipe number = 5\n");
	}
	else{
		writeRegMulti(spiHandle, 0x0A + pipeNumber, address, 5);
	}
}

//set own address note: only data pipe 0 is used in this implementation
void setTXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5]){
	writeRegMulti(spiHandle, 0x0A, address, 5); // set RX address pipe 0 for auto acks
	writeRegMulti(spiHandle, 0x10, address, 5); // set TX address
}

void setFreqChannel(SPI_HandleTypeDef* spiHandle, uint8_t channelNumber){
	if(channelNumber > 127){
		TextOut("Error, max channelNumber = 127\n");
	}
	else{
		writeReg(spiHandle, 0x05, channelNumber);
	}
}

//enable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
void enableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber){
	if(pipeNumber > 5){
		TextOut("Error, max pipe number = 5 \n");
	}
	else{
		uint8_t reg02 = readReg(spiHandle, 0x02);
		reg02 = setBit(reg02, pipeNumber, 1);
		writeReg(spiHandle, 0x02, reg02);
	}
}

//disable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
void disableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber){
	if(pipeNumber == 0){
		TextOut("Error, pipe 0 reserved for acks\n");
	}
	else if(pipeNumber > 5){
		TextOut("Error, max pipe number = 5 \n");
	}
	else{
		uint8_t reg02 = readReg(spiHandle, 0x02);
		reg02 = setBit(reg02, pipeNumber, 0);
		writeReg(spiHandle, 0x02, reg02);// disable pipe
		writeReg(spiHandle, 0x11 + pipeNumber, 0); //set buffer size to 0;
	}
}

//choose which datapipes to use
//note that pipeNumber[0] should always be 1, because this pipe is used for acks
void setDataPipeArray(SPI_HandleTypeDef* spiHandle, uint8_t pipeEnable[6]){
	if(pipeEnable[0] == 0){
		TextOut("Error, pipe 0 reserved for acks\n");
	}
	else{
		uint8_t reg02 = 0; //what is this purpose of this variable? -- the value which will be written to Register 0x02...
		for(int i = 0; i < 6; i++){
			if(pipeEnable[i] == 1){
				reg02 = setBit(reg02, i, 1);
			}
		}
		writeReg(spiHandle, 0x02, reg02); //what is the hardcoded number? -- apparently the EN_RXADDR register (datasheet page 54)
	}
}

//set the size of the RX buffer in bytes
void setRXbufferSize(SPI_HandleTypeDef* spiHandle, uint8_t size){
	if(size > 32){
		TextOut("Error: size can not be bigger than 32 bytes\n");
	}
	else{
		uint8_t reg02 = readReg(spiHandle, 0x02);
		for(int i = 0; i < 6; i++){
			if(readBit(reg02, i)){
				writeReg(spiHandle, 0x11 + i, size);
			}
			else{
				writeReg(spiHandle, 0x11 + i, 0);
			}
		}
	}
}

//make sure interrupts for the TX functions are enabled
//and those for the RX functions not
void TXinterrupts(SPI_HandleTypeDef* spiHandle){
	uint8_t reg00 = readReg(spiHandle, 0x00);
	reg00 = setBit(reg00, 6, 1);
	reg00 = setBit(reg00, 5, 0);
	reg00 = setBit(reg00, 4, 0);
	writeReg(spiHandle, 0x00, reg00);
}

//make sure interrupts for the RX functions are enabled
//and those for the TX functions not
void RXinterrupts(SPI_HandleTypeDef* spiHandle){
	uint8_t reg00 = readReg(spiHandle, 0x00);
	reg00 = setBit(reg00, 6, 0);
	reg00 = setBit(reg00, 5, 1);
	reg00 = setBit(reg00, 4, 1);
	writeReg(spiHandle, 0x00, reg00);
}

//---------------------------------modes----------------------------------//

//power down the device. SPI stays active.
void powerDown(SPI_HandleTypeDef* spiHandle){
	ceLow(spiHandle); //go to standby mode
	uint8_t reg00 = readReg(spiHandle, 0x00);

	//set power bit: bit 1 of reg 0
	reg00 = setBit(reg00, 0x01, 0);

	writeReg(spiHandle, 0x00, reg00);
}

//go to standby. SPI stays active. consumes more power, but can go to TX or RX quickly
void powerUp(SPI_HandleTypeDef* spiHandle){
	ceLow(spiHandle);

	uint8_t reg00 = readReg(spiHandle, 0x00);

	//set power up bit: bit 2 of reg 0.
	reg00 = reg00 | 0x02;

	writeReg(spiHandle, 0x00, reg00);

}

//device power up and start listening
void powerUpTX(SPI_HandleTypeDef* spiHandle){
	ceLow(spiHandle); //stay in standby mode, until there is data to send.
	uint8_t reg00 = readReg(spiHandle, 0x00);

	//flush TX buffer
	flushTX(spiHandle);

	//set power up bit: bit 1 of reg 0
	reg00 = setBit(reg00, 1, 1);
	//set RX/TX to TX: bit 0 to 1
	reg00 = setBit(reg00, 0, 0);


	writeReg(spiHandle, 0x00, reg00);

}

//device power up, and be ready to receive bytes.
void powerUpRX(SPI_HandleTypeDef* spiHandle){
	uint8_t reg00 = readReg(spiHandle, 0x00);

	flushRX(spiHandle);

	//set power up bit: bit 1 of reg 0
	reg00 = setBit(reg00, 1, 1);
	//set RX/TX to RX: bit 0 to 1
	reg00 = setBit(reg00, 0, 1);

	writeReg(spiHandle, 0x00, reg00);

	//put CE pin high ->  start listening
	ceHigh(spiHandle);
}


//--------------------------sending and receiving-------------------------//

//flush the TX buffer
void flushTX(SPI_HandleTypeDef* spiHandle){
	nssLow(spiHandle);
	uint8_t sendData = 0xE1; //FLUSH_TX
	HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);
	nssHigh(spiHandle);

}

//flush the RX buffer
void flushRX(SPI_HandleTypeDef* spiHandle){
	nssLow(spiHandle);
	uint8_t sendData = 0xE2; //FLUSH_RX
	HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);
	nssHigh(spiHandle);
}

//send a byte. only used in TX mode
//warning: after sending, the CE pin stays high.
//it should be put down manually when either MAX_RT or TX_DS is high
//this can be done using the powerUpTX function
//not doing this will cause the wireless module to stay on, which is a wast of energy.
void sendData(SPI_HandleTypeDef* spiHandle, uint8_t data[], uint8_t length){
	//TextOut("before sending\n");

	uint8_t troep[length];
	ceLow(spiHandle);

	flushTX(spiHandle);

	nssLow(spiHandle);
	uint8_t command = 0xA0; // W_TX_PAYLOAD
	HAL_SPI_Transmit(spiHandle, &command, 1, 100); //send the write to TX FIFO command

	//old method sending unidirectional
	//for(int i = 0; i < length; i++){
	//	HAL_SPI_Transmit(spiHandle, data + i, 1, 100); //send the data to NRF
	//}
	HAL_SPI_TransmitReceive(spiHandle, data, troep, length, 100); //new method with return data
	nssHigh(spiHandle);

	//send over air

	//is it save to set CE high here (disables transmitter module)? Can we be sure that all the data is already transmitted?
	ceHigh(spiHandle);


}

//read a byte from the buffer. only used in RX mode
void readData(SPI_HandleTypeDef* spiHandle, uint8_t* receiveBuffer, uint8_t length){
	uint8_t receivedData[length];

	nssLow(spiHandle);

	uint8_t command = 0x61; //R_RX_PAYLOAD
	HAL_SPI_Transmit(spiHandle, &command, 1, 100);

	HAL_SPI_Receive(spiHandle, receivedData, length, 100);

	nssHigh(spiHandle);

	memcpy(receiveBuffer, receivedData, length);

}

void setLowSpeed(SPI_HandleTypeDef* spiHandle){
	uint8_t reg06 = readReg(spiHandle, 0x06);
	reg06 = setBit(reg06, 5, 1);
	reg06 = setBit(reg06, 3, 0);
	writeReg(spiHandle, 0x06, reg06);
}

void enableAutoRetransmitSlow(SPI_HandleTypeDef* spiHandle){
	//uint8_t reg04 = 0xf3;
	writeReg(spiHandle, 0x04, 0x11);
}


//---------------------------------debug----------------------------------//

void printAllRegisters(SPI_HandleTypeDef* spiHandle){
	uint8_t regMulti[5];
	uint8_t reg;
	for(int i = 0x00; i <= 0x09; i++){
		reg = readReg(spiHandle, i);
		sprintf(smallStrBuffer, "reg %x = %x\n", i, reg);
		TextOut(smallStrBuffer);

	}

	for(int i = 0x0A; i <= 0x10; i++){
		readRegMulti(spiHandle, i, regMulti, 5);
		for(int j = 0; j < 5; j++){
			sprintf(smallStrBuffer, "reg %x; field %x = %x\n", i, j, regMulti[j]);
			TextOut(smallStrBuffer);

		}
	}

	for(int i = 0x11; i <= 0x17; i++){

		readReg(spiHandle, i);
		sprintf(smallStrBuffer, "reg %x = %x\n", i, reg);
		TextOut(smallStrBuffer);
	}

	for(int i = 0x1C; i <= 0x1D; i++){
		readReg(spiHandle, i);
		sprintf(smallStrBuffer, "reg %x = %x\n", i, reg);
		TextOut(smallStrBuffer);
	}
}

//**********************application specific code*********************//

void initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address){
	//reset and flush buffer
	NRFinit(spiHandle);

	//enable RX interrupts, disable TX interrupts
	RXinterrupts(spiHandle);

	//set the frequency channel
	setFreqChannel(spiHandle, freqChannel);

	//enable pipe 0 and 1, diabable all other pipes
	uint8_t dataPipeArray[6] = {1, 1, 0, 0, 0, 0};
	setDataPipeArray(spiHandle, dataPipeArray);

	//set the RX buffer size to 8 bytes
	setRXbufferSize(spiHandle, 8);

	uint8_t addressLong[5] = {0x12, 0x34, 0x56, 0x78, 0x90 + address};
	//set the RX address of channel 1
	setRXaddress(spiHandle, addressLong, 1);

	setLowSpeed(spiHandle);

	//go to RX mode and start listening
	powerUpRX(spiHandle);

}

void initBase(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address[5]){
	//reset and flush buffer
	NRFinit(spiHandle);

	//enable RX interrupts, disable TX interrupts
	TXinterrupts(spiHandle);

	//set the frequency channel
	setFreqChannel(spiHandle, freqChannel);

	//enable pipe 0, diabable all other pipes
	uint8_t dataPipeArray[6] = {1, 0, 0, 0, 0, 0};

	//is this overwritten again whenever we transmit?
	setDataPipeArray(spiHandle, dataPipeArray);

	//set the RX buffer size to 8 bytes
	setRXbufferSize(spiHandle, 12);

	//set the TX address of
	setTXaddress(spiHandle, address);

	//set auto retransmit to 1 time with 250 us
	writeReg(spiHandle, 0x04, 0x00);

	setLowSpeed(spiHandle);

	//enableAutoRetransmitSlow(spiHandle);

	//go to TX mode and be ready to listen
	powerUpTX(spiHandle);
}

uint8_t sendPacketPart1(SPI_HandleTypeDef* spiHandle, uint8_t packet[12]){

	uint8_t addressLong[5] = {0x12, 0x34, 0x56, 0x78, 0x90 + (packet[0] >> 4)};

	setTXaddress(spiHandle, addressLong);
	sendData(spiHandle, packet, 12);


	//returning Robot ID, but no call to this function ever appears to use the return value
	return addressLong[4];
}

void waitAck(SPI_HandleTypeDef* spiHandle, uint8_t roboID){
	if(irqRead(spiHandle)){
		//check if transmistion was succesful
		//TextOut("in waitAck!\n");
		uint8_t succesful;
		uint8_t reg07 = readReg(spiHandle, 0x07);
		ceLow(spiHandle);
		if(readBit(reg07, 4)){
			succesful = 0;
		}
		else if(readBit(reg07, 5)){
			succesful = 1;
		}
		else{
			succesful = 0xFF;
			TextOut("Error: interupt pin high, but no transmission\n");
		}
		writeReg(spiHandle, 0x07, 0x3E);
		uint8_t ack[2] = {roboID, succesful};
		sprintf(smallStrBuffer, "%x%x", ack[0], ack[1]);
		TextOut(smallStrBuffer);
		//HexOut(ack, 2);
		//sprintf(smallStrBuffer, "id: %i; succesfull: %i\n", roboID, succesful);
		//TextOut(smallStrBuffer);

	}
}

dataPacket dataStruct;
void roboCallback(SPI_HandleTypeDef* spiHandle){
		uint8_t dataArray[8];


		ceLow(spiHandle);
		readData(spiHandle, dataArray, 12);
		dataStruct.robotID = dataArray[0] >> 4;
		dataStruct.robotVelocity = ((dataArray[0] & 0xF) << 8) + dataArray[1];
		dataStruct.movingDirection = (dataArray[2] << 1) + (dataArray[3] >> 4);
		dataStruct.rotationDirection = dataArray[3] & 0x8;
		dataStruct.angularVelocity = ((dataArray[3] & 0x7) << 8) + dataArray[4];
		dataStruct.kickForce = dataArray[5];
		dataStruct.kick = dataArray[6] & 0x40;
		dataStruct.chipper = dataArray[6] & 0x20;
		dataStruct.forced = dataArray[6] & 0x10;
		dataStruct.driblerDirection = dataArray[6] & 0x8;
		dataStruct.driblerSpeed = dataArray[6] & 0x7;
		//clear RX interrupt
		writeReg(spiHandle, 0x07, 0x4E);

		ceHigh(spiHandle);

	}

