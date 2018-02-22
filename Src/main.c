/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "myNRF24.h"
#include "TextOut.h"
#include "packing.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	 * Why "madeUpPacket"? Is this an "invented" and totally made-up packet?
	 * As I understand it, it is structured data which we are about to send as payload with the NRF module.
	 * I believe there could be a better name for this. "madeUp" sounds like it's fake and untrue.
	 *
	 * Edit: Oh, I get it. It fakes a USB packet.
	 */
	uint8_t madeUpPacket[12];

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  fun2();
  nssHigh(&hspi3);
  ceLow(&hspi3);
  uint8_t address[5] = {0x12, 0x34, 0x56, 0x78, 0x97};
  initBase(&hspi3, 78  , address);
  GPIO_PinState button6;
  GPIO_PinState button5;
  GPIO_PinState button4;
  /* never used:
  GPIO_PinState buttom3;
  GPIO_PinState buttom2;
  GPIO_PinState prevButtom6;
  GPIO_PinState prevButtom5;
  */
  uint8_t remote = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);

  int id = 2;
  int robot_vel = 0;
  int ang = 0;
  uint8_t rot_cclockwise = 0;
  int w_vel = 0; //w=omega velocity: rotational velocity.
  uint8_t kick_force = 0;
  uint8_t do_kick = 0;
  uint8_t chip = 0;
  uint8_t forced = 0;
  uint8_t dribble_cclockwise = 0;
  uint8_t dribble_vel = 0xff;
  // never used: uint8_t* byteArr = 0;
  uint8_t prevBlue = 0;
  uint8_t blue = 0;
  // never used: int cnt = 0;

  uint8_t debug_transmit_repeatedly = 1;

  while (1)
  {
	  button6 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
	  button5 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	  button4 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	  //never used: buttom3 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	  //never used: buttom2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
	  blue = HAL_GPIO_ReadPin(GPIOA, Blue_Pin);

	  if(debug_transmit_repeatedly == 1) {
		  TextOut("sending a packet");
		  remote = 1;
		  robot_vel = 1000;
		  ang = 0;
		  w_vel = 0;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);

		  for(int i = 0; i < 12; i++){
			  usbData[i] = madeUpPacket[i];

		  }
		  usbLength = 12;
		  sendPacketPart1(&hspi3, usbData);
		  usbLength = 0;
		  HAL_Delay(10);



		  sendPacketPart1(&hspi3, madeUpPacket);
		  HAL_Delay(300);
		  fun(); //delay with a LED animation

	  }
	  if(blue == 1 && blue != prevBlue){
		  //initBase(&hspi3, 0x2A, address);
		  HAL_Delay(100);
		  //printAllRegisters(&hspi3);
	  }

	  if(!button6){
		  TextOut("forward");
		  remote = 1;
		  robot_vel = 1000;
		  ang = 0;
		  w_vel = 0;
		  //dribble_vel = 7;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);


	  }
	  else if(!button5){
		  TextOut("sidewards");
		  /*remote = 1;
		  robot_vel = 250;
		  ang = 256;
		  w_vel = 0;
		  dribble_vel = 0;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);
	  */
		  remote = 1;
		  kick_force = 250;
		  chip = 0;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);
	  }
	  else if(!button4){
		  TextOut("turning");
		  remote = 1;
		  robot_vel = 0;
		  w_vel = 360;
		  rot_cclockwise = 1;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);

	  }
	  else{
		  //TextOut("turning");
		  //remote = 1;
		  /*
		   * Apparently this section should reset the Robot to an idle state after it was controlled with the buttons on the board.
		   * However, it would actually be executed whenever no button was pressed with every round of the main loop.
		   * That means it is also executed when the board is controlled over USB.
		   * This does not seem to be a big issue, since the varables set here are all related to creating a packet which is meant to
		   * overwrite the usbData packet, which, however, is never overwritten unless remote=1.
		   *
		   * Therefore: in such a case we are doing useless memory writes here which are never read.
		   *
		   */
		  w_vel = 000;
		  rot_cclockwise = 1;
		  robot_vel = 0000;
		  ang = 0;
		  kick_force = 0;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);

	  }


	  if(remote == 1){

		  /*
		   * I see that remote is set to 1 whenever a button on the board is pressed.
		   * But it is never reset to 0.
		   * Does that mean, that the board will stay in manual mode (button controlled) until reset?
		   * If that is the case, then we could ignore any USB data anyway and circumvent the below mentioned possible
		   * flaw for a race condition (which would be caused when the button control is used while USB data is received at the same time).
		   */
		  for(int i = 0; i < 12; i++){
			  /*
			   * why are you overwriting the usbData when it appears you could just
			   * call sendPacketPart1() with madeUpPacket instead?
			   *
			   * usbData seems to be an external array, possibly overwritten by an interrupt service routine,
			   * so it doesn't seem to be thread safe to overwrite this data. It might cause a race condition:
			   * it might be overwritten by an interrupt while you attempt to overwrite it or after you
			   * were overwriting it and before the call of sendPacketPart1().
			   *
			   */
			  usbData[i] = madeUpPacket[i];

		  }
		  //usbData[7] = 0;
		  usbLength = 12;
		  /*for(int i = 0; i < 8; i++){
			  sprintf(smallStrBuffer, "byte %i: %x\n", i, usbData[i]);
			  TextOut(smallStrBuffer);
		  }*/
		  sendPacketPart1(&hspi3, usbData);
		  usbLength = 0;
		  HAL_Delay(10);
	  }


	  if(usbLength == 12){

		  sendPacketPart1(&hspi3, usbData);
		  usbLength = 0;
	  }



	  if(usbLength != 0){
		  //fun();
	  }


	  //was never read: prevButtom6 = buttom6;
	  //was never read: prevButtom5 = buttom5;
	  prevBlue = blue;


	  //waitAck(&hspi3, usbData[0] >> 4); //note: this prints ACK messages with TextOut()


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
