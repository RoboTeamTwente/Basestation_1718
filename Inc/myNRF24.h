/*
 * myNRF24.h
 *
 *  Created on: 19 sep. 2016
 *      Author: Hans-van-der-Heide
 */

//TODO SPI timeout

#ifndef MYNRF24_H_
#define MYNRF24_H_

#define PI 3.14159

#include "TextOut.h"
#include "gpio.h"
#include "stm32f3xx_hal.h"
#include "spi.h"

//defining registers
//see datasheet page 54 and following
enum nrfRegister {
	//feel free to complete
	//the values in the comments.
	//they are just increments unless they are overwritten (DYNPD)
	CONFIG, //0x00
	EN_AA, //0x01
	EN_RXADDR,
	SETUP_AW,
	SETUP_RETR,
	RF_CH, //0x05
	RF_SETUP,
	STATUS,
	OBSERVE_TX,
	RPD,
	RX_ADDR_P0, //0x0A
	RX_ADDR_P1,
	RX_ADDR_P2,
	RX_ADDR_P3,
	RX_ADDR_P4,
	RX_ADDR_P5, //0x0F
	TX_ADDR, //0x10
	RX_PW_P0,
	RX_PW_P1,
	RX_PW_P2,
	RX_PW_P3,
	RX_PW_P4,
	RX_PW_P5,
	FIFO_STATUS, //0x17
	DYNPD=0x1C, //0x1C
	FEATURE //0x1D

};

//defining bit flags for each register with single-bit fields
//multi-bit fields are not included -- when you use them, you should
//definitely look it up in the datasheet

enum CONFIG_FLAG {
	MASK_RX_DR = 1<<6,
	MASK_TX_DS = 1<<5,
	MASK_MAX_RT = 1<<4,
	EN_CRC = 1<<3,
	CRC0 = 1<<2,
	PWR_UP = 1<<1,
	PRIM_RX = 1<<0
};

enum EN_AA_FLAG {
	ENAA_P5 = 1<<5,
	ENAA_P4 = 1<<4,
	ENAA_P3 = 1<<3,
	ENAA_P2 = 1<<2,
	ENAA_P1 = 1<<1,
	ENAA_P0 = 1<<0
};

enum EN_RXADDR_FLAG {
	ERX_P5 = 1<<5,
	ERX_P4 = 1<<4,
	ERX_P3 = 1<<3,
	ERX_P2 = 1<<2,
	ERX_P1 = 1<<1,
	ERX_P0 = 1<<0
};

//enum SETUP_AW_FLAG {
	//AW is Bit 1 down to 0
	//AW 1:0
//};

//enum SETUP_RETR_FLAG {
	//ARD is Bit 7 down to 4
	//ARD 7:4
	//ARC 3:0
//};

//enum RF_CH_FLAG {
	//RF_CH 6:0
//};

enum RF_SETUP_FLAG {
	CONT_WAVE = 1<<7,
	//6 reserved
	RF_DR_LOW = 1<<5,
	PLL_LOCK = 1<<4,
	RF_DR_HIGH = 1<<3,
	//RF_PWR 2:1
	//0 Obsolete
};

enum STATUS_FLAG {
	RX_DR = 1<<6,
	TX_DS = 1<<5,
	MAX_RT = 1<<4,
	//RX_P_NO 3:1
	STATUS_TX_FULL = 1<<0,
};

//enum OBSERVE_TX_FLAG {
	//PLOS_CNT 7:4
	//ARC_CNT 3:0
//};

//enum RPD_FLAG {
//	RPD = 1<<0,
//};


//skipping a few registers here, since they are multi-bit registers anyway
//RX_ADDR_P0
//RX_ADDR_P1
//RX_ADDR_P2
//RX_ADDR_P3
//RX_ADDR_P4
//RX_ADDR_P5
//TX_ADDR


//enum RX_PW_PX_FLAG {
	//RX_PW_PX 5:0
//};

enum FIFO_STATUS_FLAG {
	TX_REUSE = 1<<6,
	FIFO_STATUS_TX_FULL = 1<<5,
	TX_EMPTY = 1<<4,
	//3:2 reserved
	RX_FULL = 1<<1,
	RX_EMPTY = 1<<0,
};

enum DYNPD_FLAG {
	DPL_P5 = 1<<5,
	DPL_P4 = 1<<4,
	DPL_P3 = 1<<3,
	DPL_P2 = 1<<2,
	DPL_P1 = 1<<1,
	DPL_P0 = 1<<0,
};

enum FEATURE_FLAG {
	EN_DPL = 1<<2,
	EN_ACK_PAY = 1<<1,
	EN_DYN_ACK = 1<<0,
};


/*
 *
 *
 */

typedef struct dataPacket {
  uint8_t robotID; // 0 to 15
  uint16_t robotVelocity; //between 0 and 4095mm/s
  uint16_t movingDirection; // resolution: 2pi/512 radians
  uint8_t rotationDirection; //0 = cw; 1 = ccw;
  uint16_t angularVelocity; //0 to 2047 deg/s
  uint8_t kickForce; // 0 to 255
  uint8_t forced; // 0 = normal kick; 1 = forced kick
  uint8_t chipper; // 0 = kicker; 1 = chipper
  uint8_t kick; // 0 = do not kick; 1 = kick according to forced and chipper
  uint8_t driblerDirection; // 0 = cw; 1 = ccw;
  uint8_t driblerSpeed; // between 0 and 7
} dataPacket;

struct ackPacket {
  uint8_t robotID;
  uint8_t succes;
};

extern dataPacket dataStruct;


//*************************auxillery functions**********************************//
//*******************not actually for NRF24 control*****************************//

//blink leds for debugging purposes
void fun();

void fun2();
void fun2out();

//set a specific bit in a byte to a 1 or a 0
uint8_t setBit(uint8_t byte, uint8_t position, uint8_t value);

//check if a specific bit in a byte is 1
uint8_t readBit(uint8_t byte, uint8_t position);


//*****************************low level library********************************//
//******************the user is not supposed to use these***********************//

//put the csn pin corresponding to the SPI used high
void nssHigh(SPI_HandleTypeDef* spiHandle);

//put the csn pin corresponding to the SPI used low
void nssLow(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used high
void ceHigh(SPI_HandleTypeDef* spiHandle);

//put the ce pin corresponding to the SPI used low
void ceLow(SPI_HandleTypeDef* spiHandle);

uint8_t irqRead(SPI_HandleTypeDef* spihandle);

//write to a register and output debug info to the terminal
void writeRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data);

//write to a register
void writeReg(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t data);

//write to a multi-byte register and output debug info to the terminal
void writeRegMultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size);

//write to a multi-byte register
int8_t writeRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* data, uint8_t size);

//read a register and output debug info to the terminal
uint8_t readRegDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg);

//read a register
uint8_t readReg(SPI_HandleTypeDef* spiHandle, uint8_t reg);

//read a multi-byte register and output debug info to terminal
//output will be stored in the array dataBuffer
void readRegmultiDebug(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size);

//read a multi-byte register
//output will be stored in the array dataBuffer
void readRegMulti(SPI_HandleTypeDef* spiHandle, uint8_t reg, uint8_t* dataBuffer, uint8_t size);




//****************************high level library**************************//
//********************the user may use these functions********************//

//--------------------initialization and configuration--------------------//

//reset to reset value on page 54
void softResetRegisters(SPI_HandleTypeDef* spiHandle);

//initialize the system:
//reset it and enable pipe 1 and 0
//set pipeWith to 1
//flush TX and RX buffer
void NRFinit(SPI_HandleTypeDef* spiHandle);

//set the address you will send to
void setTXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5]);

//set own address note: only data pipe 0 is used in this implementation
void setRXaddress(SPI_HandleTypeDef* spiHandle, uint8_t address[5], uint8_t pipeNumber);

//set a frequency channel
void setFreqChannel(SPI_HandleTypeDef* spiHandle, uint8_t channelNumber);

//enable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
//note: RX buffer size should be set
void enableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber);

//disable a RX data pipe
//note: pipe 0 is invalid, as it is used for acks
void disableDataPipe(SPI_HandleTypeDef* spiHandle, uint8_t pipeNumber);

//choose which datapipes to use
//note: that pipeNumber[0] should always be 1, because this pipe is used for acks
//note: RX buffer size should be set
void setDataPipeArray(SPI_HandleTypeDef* spiHandle, uint8_t pipeEnable);

//set the size of the RX buffer in bytes
void setRXbufferSize(SPI_HandleTypeDef* spiHandle, uint8_t size);

//make sure interrupts for the TX functions are enabled
//and those for the RX functions not
void TXinterrupts(SPI_HandleTypeDef* spiHandle);;

//make sure interrupts for the RX functions are enabled
//and those for the TX functions not
void RXinterrupts(SPI_HandleTypeDef* spiHandle);

//---------------------------------modes----------------------------------//

//power down the device. SPI stays active.
void powerDown(SPI_HandleTypeDef* spiHandle);

//go to standby. SPI stays active. consumes more power, but can go to TX or RX quickly
void powerUp(SPI_HandleTypeDef* spiHandle);

//device power up and start listening
void powerUpRX(SPI_HandleTypeDef* spiHandle);

//device power up, and be ready to receive bytes.
void powerUpTX(SPI_HandleTypeDef* spiHandle);


//--------------------------sending and receiving-------------------------//

//flush the TX buffer
void flushTX(SPI_HandleTypeDef* spiHandle);

//flush the RX buffer
void flushRX(SPI_HandleTypeDef* spiHandle);

//send a byte. only used in TX mode
//warning: after sending, the CE pin stays high.
//it should be put down manually when either MAX_RT or TX_DS is high
//this can be done using the powerUpTX function
//not doing this will cause the wireless module to stay on, which is a waste of energy.
void sendData(SPI_HandleTypeDef* spiHandle, uint8_t data[], uint8_t length);

//read a byte from the buffer. only used in RX mode
void readData(SPI_HandleTypeDef* spiHandle, uint8_t* receiveBuffer, uint8_t length);

void setLowSpeed(SPI_HandleTypeDef* spiHandle);



//---------------------------------debug----------------------------------//
//print all the registers
void printAllRegisters(SPI_HandleTypeDef* spiHandle);

//**********************application specific code*********************//
//

//------------------------------ initRX ------------------------------//
//reset and init
//mask right interrupts
//set frequency
//enable right data pipe
//set RX buffer size
//set RX address
//powerUpRX
void initRobo(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address);

//------------------------------ initTX ------------------------------//
//rest and init
//mask right interrupts
//set frequency
//enable ack pipe
//set RX buffer size
//set TX address + pipe 0 address
//powerUpTX
void initBase(SPI_HandleTypeDef* spiHandle, uint8_t freqChannel, uint8_t address[5]);

//---------------------------TODO TX loop----------------------------//
//get data from pc
//read address -> test
//change TX address to address from packet -> test
//send data
//wait for interrupt
//note timeout and clear interrupt bit
//clear interrupt bits in case of succesful transmission
//set CE low
//send ack data back to pc

//sendpacket is split in 2 parts, so it can be done on 1 microcontroller
//waiting for send done interrupt would halt the receiving part of the program
uint8_t sendPacketPart1(SPI_HandleTypeDef* spiHandle, uint8_t packet[8]);
void waitAck(SPI_HandleTypeDef* spiHandle, uint8_t roboID);

//------------------------TODO RX callback---------------------------//
//interrupt driven, should be executed when IRQ pin is low
//read data from buffer
//TODO decode data and put in struct
//TODO return struct
//TODO put interrupt low

void roboCallback(SPI_HandleTypeDef* spiHandle);

//------------------------TODO RX get info---------------------------//
//TODO get struct from robot
//TODO encode struct in uint8_t array
//TODO put array as acknowledgement

#endif /* MYNRF24_H_ */
