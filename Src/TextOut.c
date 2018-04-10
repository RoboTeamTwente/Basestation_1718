/*
 * TextOut.c
 *
 *  Created on: 14 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "TextOut.h"
char smallStrBuffer[1024];

//adding a carriage-return (\r) at the first line-feed (\n) we find; then ensure the string ends with null (\0) right after that
//the result is written to smallStrBuffer (this makes it an ugly and not obvious implementation)
void backslashNfixer(char* str){
	int length = strlen(str);
	char* backslashNpointer = strchr(str, '\n'); //point to the first \n
	memcpy(smallStrBuffer, str, length);
	if(backslashNpointer != NULL){
		//before call: "somestring\n\0andmaybemoretextoverhere", after call: "somestring\n\r\0" -- I believe that's what we're doing here
		smallStrBuffer[length] = '\r';
		smallStrBuffer[length+1] = '\0';
	}
}

void TextOut(char *str){

	backslashNfixer(str); //fixes encoding and copies to smallStrBuffer...
	//memcpy(smallStrBuffer, str, strlen(str)); //already copied with backslashNfixer
	int length = strlen(smallStrBuffer);

	/*
	while(CDC_Transmit_FS(0, 0) & USBD_BUSY) {
		//wait until the interface is no longer busy
		if(CDC_Transmit_FS(0, 0) & USBD_FAIL) //or break on fail
			return;
	}
	*/
	//while(CDC_Transmit_FS((uint8_t*)smallStrBuffer, length) == USBD_BUSY);
	CDC_Transmit_FS((uint8_t*)smallStrBuffer, length);
	HAL_Delay(1);

	//clear buffer
	for(uint16_t i = 0; i <= 1023; i++) {
		smallStrBuffer[i] = '\0';
	}

}

void HexOut(uint8_t data[], uint8_t length){
	CDC_Transmit_FS(data, length);
}
