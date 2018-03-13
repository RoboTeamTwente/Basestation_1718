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
#include "myNRF24basic.h"

#include <string.h>


//****************************high level library**************************//
//********************the user may use these functions********************//

//--------------------initialization and configuration--------------------//

//reset all register values to reset values on page 54, datasheet
void softResetRegisters(SPI_HandleTypeDef* spiHandle){
	uint8_t multRegData[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

	// see page 54 and further for reset values
	writeReg(spiHandle, CONFIG, 0x08);
	writeReg(spiHandle, EN_AA, 0x3F);
	writeReg(spiHandle, EN_RXADDR, 0x03);
	writeReg(spiHandle, SETUP_AW, 0x03);
	writeReg(spiHandle, SETUP_RETR, 0x03);
	writeReg(spiHandle, RF_CH, 0x02);
	writeReg(spiHandle, RF_SETUP, 0x0E);
	writeReg(spiHandle, STATUS, 0x00);
	//register 0x08 and 0x09 are read only
	writeRegMulti(spiHandle, RX_ADDR_P0, multRegData, 5);

	for(int i = 0; i < 5; i++){multRegData[i] = 0xC2;}
	writeRegMulti(spiHandle, RX_ADDR_P1, multRegData, 5);
	writeReg(spiHandle, RX_ADDR_P2, 0xC3);
	writeReg(spiHandle, RX_ADDR_P3, 0xC4);
	writeReg(spiHandle, RX_ADDR_P4, 0xC5);
	writeReg(spiHandle, RX_ADDR_P5, 0xC6);
	for(int i = 0; i < 5; i++){multRegData[i] = 0xE7;}
	writeRegMulti(spiHandle, TX_ADDR, multRegData, 5);
	writeReg(spiHandle, RX_PW_P0, 0x00);
	writeReg(spiHandle, RX_PW_P1, 0x00);
	writeReg(spiHandle, RX_PW_P2, 0x00);
	writeReg(spiHandle, RX_PW_P3, 0x00);
	writeReg(spiHandle, RX_PW_P4, 0x00);
	writeReg(spiHandle, RX_PW_P5, 0x00);
	writeReg(spiHandle, FIFO_STATUS, 0x11);
	//reg 0x18 to 0x1B are undocumented test registers. Don't write to them!
	writeReg(spiHandle, DYNPD, 0x00);
	writeReg(spiHandle, FEATURE, 0x00);

}

//initialize the system:
//reset it and enable pipe 1 and 0
//set pipeWith to 1
//flush TX and RX buffer
void NRFinit(SPI_HandleTypeDef* spiHandle){
	//reset system

	/*
	 * I don't see a need for resetting all register values to their default values
	 * when the system has just booted up. The datasheet already promises those values during boot-up.
	 * A call to the softResetRegisters() function is only needed when we want to simulate a reboot of the nRF module
	 * without actually turning it on and off.
	 */
	//softResetRegisters(spiHandle);
	clearInterrupts(spiHandle);

	//enable RX pipe 0 and 1, disable all other pipes
	writeReg(spiHandle, EN_RXADDR, ERX_P0|ERX_P1);

	//set RX pipe with of pipe 0 to 1 byte.
	writeReg(spiHandle, RX_PW_P0, 0x01);

	//set RX pipe with of pipe 1 to 1 byte.
	writeReg(spiHandle, RX_PW_P1, 0x01);

	flushRX(spiHandle);
	flushTX(spiHandle);

}


//set the address you will send to
//pipe 0 is reserved for acks: it's adress always equals the TX address and is set with setTXaddress
//returns 0 on success; -1 on error
int8_t setRXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5], uint8_t pipeNumber){
	if(pipeNumber == 0){
		//TextOut("Error: pipe 0 is reserved for acks\n");
		return -1; //error
	}
	else if(pipeNumber > 5){
		TextOut("Error: max pipe number = 5\n");
		return -1; //error
	}

	writeRegMulti(spiHandle, RX_ADDR_P0 + pipeNumber, address, 5);

	return 0;
}

//set own address note: only data pipe 0 is used in this implementation
//returns 0 on success; -1 on error
int8_t setTXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5]){
	if(writeRegMulti(spiHandle, RX_ADDR_P0, address, 5) != 0) // set RX address pipe 0 for auto acks
		return -1; //error
	if(writeRegMulti(spiHandle, TX_ADDR, address, 5) != 0) // set TX address
		return -1; //error

	return 0; //success
}

//returns 0 on success; -1 on error
int8_t setFreqChannel(SPI_HandleTypeDef* spiHandle, uint8_t channelNumber){
	if(channelNumber > 127)
		//TextOut("Error, max channelNumber = 127\n");
		return -1; //error: invalid channel Number

	//forward the return value of writeReg() to caller
	return writeReg(spiHandle, RF_CH, channelNumber);
}

//enable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
//returns 0 on success; -1 on error
int8_t enableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber){
	if(pipeNumber > 5)
		return -1; //error: invalid pipeNumber

	uint8_t en_rxaddr_val = readReg(spiHandle, EN_RXADDR);
	en_rxaddr_val |= (1 << pipeNumber);

	return writeReg(spiHandle, EN_RXADDR, en_rxaddr_val);
}

//disable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
int8_t disableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber){
	if(pipeNumber == 0){
		//TextOut("Error, pipe 0 reserved for acks\n");
		return -1; //error: invalid pipeNumber
	}
	else if(pipeNumber > 5){
		//TextOut("Error, max pipe number = 5 \n");
		return -1; //error: invalid pipeNumber
	}
	uint8_t reg02 = readReg(spiHandle, EN_RXADDR);
	reg02 = setBit(reg02, pipeNumber, 0);
	writeReg(spiHandle, EN_RXADDR, reg02);// disable pipe
	writeReg(spiHandle, RX_PW_P0 + pipeNumber, 0); //set buffer size to 0;
	return 0;
}

//choose which datapipes to use
//Check out page 54 (Register Map) in the datasheet.
//Set the Bits according to the datapipes in pipeEnable.
void setDataPipes(SPI_HandleTypeDef* spiHandle, uint8_t pipeEnable){
	writeReg(spiHandle, EN_RXADDR, pipeEnable);
}

//set the size of the RX buffer in bytes
int8_t setRXbufferSize(SPI_HandleTypeDef* spiHandle, uint8_t size){
	if(size > 32){
		//TextOut("Error: size can not be bigger than 32 bytes\n");
		return -1; //error: size too big
	}

	uint8_t rx_addr_reg = readReg(spiHandle, EN_RXADDR);

	//for every activated data pipe in EN_RXADDR, set the buffer size in RX_PW_PX to "size" amount of bytes.
	for(uint8_t i = 0; i < 6; i++){
		if(readBit(rx_addr_reg, i)){
			writeReg(spiHandle, RX_PW_P0 + i, size);
		}
		else{
			writeReg(spiHandle, RX_PW_P0 + i, 0);
		}
	}

	return 0; //success
}

//make sure interrupts for the TX functions are enabled
//and those for the RX functions not
void TXinterrupts(SPI_HandleTypeDef* spiHandle){
	uint8_t config_reg = readReg(spiHandle, CONFIG);
	/* Register 0x00 (CONFIG)
	 * Bit 6: MASK_RX_DR
	 * Bit 5: MASK_TX_DS
	 * Bit 4: MASK_MAX_RT
	 *
	 * For those Bits:
	 * 1 means: disabled; Interrupt not reflected on IRQ Pin
	 * 0 means: enabled; Interrupt on IRQ Pin as active low
	 *
	 */
	config_reg = setBit(config_reg, 6, 1); //diable for RX_DR
	config_reg = setBit(config_reg, 5, 0); //enable for TX_DS
	config_reg = setBit(config_reg, 4, 0); //enable for MAX_RT

	//another way of writing that:
	//config_reg |= MASK_RX_DR;   //1
	//config_reg &= ~MASK_TX_DS;  //0
	//config_reg &= ~MASK_MAX_RT; //0


	writeReg(spiHandle, CONFIG, config_reg);
}

//make sure interrupts for the RX functions are enabled
//and those for the TX functions not
void RXinterrupts(SPI_HandleTypeDef* spiHandle){
	uint8_t reg_config = readReg(spiHandle, CONFIG);
	reg_config = setBit(reg_config, 6, 0);
	reg_config = setBit(reg_config, 5, 1);
	reg_config = setBit(reg_config, 4, 1);
	writeReg(spiHandle, CONFIG, reg_config);
}

//---------------------------------modes----------------------------------//

//power down the device. SPI stays active.
void powerDown(SPI_HandleTypeDef* spiHandle){
	ceLow(spiHandle); //go to standby mode
	uint8_t reg_config = readReg(spiHandle, CONFIG);

	//clear power bit: bit 2 to 0
	reg_config = setBit(reg_config, 2, 0);

	writeReg(spiHandle, CONFIG, reg_config);
}

//go to standby. SPI stays active. consumes more power, but can go to TX or RX quickly
void powerUp(SPI_HandleTypeDef* spiHandle){
	ceLow(spiHandle);

	uint8_t reg_config = readReg(spiHandle, CONFIG);

	//set power up bit: bit 2 of reg 0.
	reg_config = reg_config | PWR_UP;

	writeReg(spiHandle, CONFIG, reg_config);

}

//device power up and start listening
void powerUpTX(SPI_HandleTypeDef* spiHandle){
	ceLow(spiHandle); //stay in standby mode, until there is data to send.
	uint8_t reg_config = readReg(spiHandle, CONFIG);

	//flush TX buffer
	flushTX(spiHandle);

	//set power up bit: bit 1 of reg 0
	reg_config = setBit(reg_config, 1, 1);
	//set is Primary Transmitter (PRIM_RX Bit to 0)
	reg_config = setBit(reg_config, 0, 0);


	writeReg(spiHandle, CONFIG, reg_config);

}

//device power up, and be ready to receive bytes.
void powerUpRX(SPI_HandleTypeDef* spiHandle){
	flushRX(spiHandle);
	writeReg(spiHandle, CONFIG, PRIM_RX|PWR_UP);
	//put CE pin high ->  start listening
	ceHigh(spiHandle);
}


//--------------------------sending and receiving-------------------------//

//flush the TX buffer
void flushTX(SPI_HandleTypeDef* spiHandle){
	nssLow(spiHandle);
	uint8_t sendData = NRF_FLUSH_TX; //FLUSH_TX
	HAL_SPI_Transmit(spiHandle, &sendData, 1, 100);
	nssHigh(spiHandle);

}

//flush the RX buffer
void flushRX(SPI_HandleTypeDef* spiHandle){
	nssLow(spiHandle);
	uint8_t sendData = NRF_FLUSH_RX; //FLUSH_RX
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

	ceLow(spiHandle);

	flushTX(spiHandle);

	nssLow(spiHandle);
	uint8_t spi_command = NRF_W_TX_PAYLOAD; // W_TX_PAYLOAD
	uint8_t spi_timeout = 100;
	HAL_SPI_Transmit(spiHandle, &spi_command, 1, spi_timeout);


	HAL_SPI_Transmit(spiHandle, data, length, spi_timeout);
	nssHigh(spiHandle);


	//send over air

	ceHigh(spiHandle);
}

//read a byte from the buffer. only used in RX mode
//Edit: the datasheet say it is used in RX mode. It doesn't say it is only(!) used in RX mode.
//     Afaik, you also need to use it in TX mode when you want to read an ACK payload
void readData(SPI_HandleTypeDef* spiHandle, uint8_t* receiveBuffer, uint8_t length){
	uint8_t receivedData[length];

	nssLow(spiHandle);

	uint8_t command = NRF_R_RX_PAYLOAD; //R_RX_PAYLOAD
	HAL_SPI_Transmit(spiHandle, &command, 1, 100);

	HAL_SPI_Receive(spiHandle, receivedData, length, 100);

	nssHigh(spiHandle);

	memcpy(receiveBuffer, receivedData, length);

}


//read data from the RX FIFO until it's empty or until max_length has been reached
//this function hasn't been tested yet.
int8_t readAllData(SPI_HandleTypeDef* spiHandle, uint8_t* receiveBuffer, uint8_t max_length){
	//while there is data in the RX FIFO, read byte per byte
	uint8_t bytes_read = 0;
	while((readReg(FIFO_STATUS) & RX_EMPTY) != 0) {
		nssLow(spiHandle);

		uint8_t command = NRF_R_RX_PAYLOAD; //R_RX_PAYLOAD
		HAL_SPI_Transmit(spiHandle, &command, 1, 100);

		HAL_SPI_Receive(spiHandle, receiveBuffer++, 1, 100);

		nssHigh(spiHandle);
		bytes_read++;

		if(bytes_read >= max_length)
			return -1; //stop here and give an error. Only the amount of bytes specified by max_length will be available.
	}
	return 0; //success
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

//write ACK payload to module
//this payload will be included in the payload of ACK packets when automatic acknowledgments are activated
int8_t writeACKpayload(SPI_HandleTypeDef* spiHandle, uint8_t* payloadBytes, uint8_t payload_length) {
	//This function should be called as often as a packet was received (either before or after reception),
	//because the module can only hold up to 3 ACK packets.
	//It will use up one of the packets as a response when it receives a packet.
	//See: https://shantamraj.wordpress.com/2014/11/30/auto-ack-completely-fixed/ (visited 13th March, 2018)

	//you may want to call writeACKpayload() in the procedure which reads a packet
	nssLow(spiHandle);

	uint8_t spi_command = NRF_W_ACK_PAYLOAD;
	//activate spi command
	if(HAL_SPI_Transmit(spiHandle, &spi_command,1, 100) != HAL_OK)
		return -1; //HAL/SPI error

	//transmit values for spi command (send payload to nRF module)
	if(HAL_SPI_Transmit(spiHandle, payloadBytes, payload_length, 100) != HAL_OK)
		return -1; //HAL/SPI error

	nssHigh(spiHandle);

	return 0; //success
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
		reg = readReg(spiHandle, i);
		sprintf(smallStrBuffer, "reg %x = %x\n", i, reg);
		TextOut(smallStrBuffer);
	}

	for(int i = 0x1C; i <= 0x1D; i++){
		reg = readReg(spiHandle, i);
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
	uint8_t dataPipes = 0b0000011; //the Bits from right to left define which data pipes to activate, starting from pipe 0 on the rightmost bit.
	setDataPipes(spiHandle, dataPipes);

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
	//uint8_t dataPipeArray[6] = {1, 0, 0, 0, 0, 0};

	//is this overwritten again whenever we transmit?
	setDataPipes(spiHandle, 0x01);

	//set the RX buffer size to x bytes
	setRXbufferSize(spiHandle, 12);

	//set the TX address of
	setTXaddress(spiHandle, address);

	//set auto retransmit: disabled
	writeReg(spiHandle, SETUP_RETR, 0x00);

	setLowSpeed(spiHandle);

	//enableAutoRetransmitSlow(spiHandle);

	//go to TX mode and be ready to listen
	powerUpTX(spiHandle);
}

uint8_t sendPacketPart1(SPI_HandleTypeDef* spiHandle, uint8_t packet[12]){

	uint8_t addressLong[5] = {0x12, 0x34, 0x56, 0x78, 0x90 + (packet[0] >> 4)};

	setTXaddress(spiHandle, addressLong);
	sendData(spiHandle, packet, 12);


	//returning last byte of address, but no call to this function ever appears to use the return value
	return addressLong[4];
}

int8_t waitAck(SPI_HandleTypeDef* spiHandle, uint8_t roboID){
	if(irqRead(spiHandle)){
		//uint8_t succesful = 0;
		uint8_t status_reg = readReg(spiHandle, STATUS);
		ceLow(spiHandle);
		if(status_reg & MAX_RT){
			//maximum retransmissions reached without receiving an ACK
		}
		else if(status_reg & TX_DS & RX_DR){
			//packet (successfully) transmitted
			//an ACK with payload was received and put in the RX FIFO of the nRF module
			//succesful = 1;

			//read payload here!
			uint8_t ack_payload[12]; //I don't really like to set it to a fixed length here. I would rather like to read a global macro (#define PAYLOAD_LENGTH ?)
			readData(spiHandle, ack_payload, 12);

			clearInterrupts(spiHandle);

			//copy all bytes as hex values to a string
			for(uint8_t i=0; i<12; i++) {
				sprintf(smallStrBuffer, "%x", ack_payload[i]);
			}

			//print string to serial interface (USB)
			TextOut(smallStrBuffer);

			return 1; //success
		}
		else if((status_reg & TX_DS) && !(status_reg & RX_DR)){
			//packet transmitted, but we received no ack payload in return.
			//either we aren't using auto-acknowledgements or the receiver sent back an empty ack (ack with no payload)
		}
		else {
			//This case would be reached when RX_DR is high (indicating that we received a packet),
			//but TX_DS is low (indicating that we didn't send anything successfully prior to this reception).
			//That means: a packet was addressed to us, but we weren't waiting for it.
			//This packet is not an ACK payload, but a regular packet.
		}

	}

	return 0; //no ack payload received
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
		writeReg(spiHandle, STATUS, RX_DR);

		ceHigh(spiHandle);

	}

