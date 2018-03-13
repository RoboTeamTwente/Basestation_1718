/*
 * discoveryboard.c
 *
 *  Created on: Mar 13, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *    Functions which relate to the hardware of the STM32F3DISCOVERY board.
 */

#include "discoveryboard.h"


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


