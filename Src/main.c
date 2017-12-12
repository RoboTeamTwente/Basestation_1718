/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */



int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t madeUpPacket[12];

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  //fun();
  nssHigh(&hspi3);
  ceLow(&hspi3);
  uint8_t address[5] = {0x12, 0x34, 0x56, 0x78, 0x97};
  initBase(&hspi3, 78  , address);
  GPIO_PinState buttom6;
  GPIO_PinState buttom5;
  GPIO_PinState buttom4;
  GPIO_PinState buttom3;
  GPIO_PinState buttom2;
  GPIO_PinState prevButtom6;
  GPIO_PinState prevButtom5;
  uint8_t remote = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);

  int id = 2;
  int robot_vel = 0;
  int ang = 0;
  uint8_t rot_cclockwise = 0;
  int w_vel = 0;
  uint8_t kick_force = 0;
  uint8_t do_kick = 0;
  uint8_t chip = 0;
  uint8_t forced = 0;
  uint8_t dribble_cclockwise = 0;
  uint8_t dribble_vel = 0xff;
  uint8_t* byteArr = 0;
  uint8_t prevBlue = 0;
  uint8_t blue = 0;
  int cnt = 0;

  while (1)
  {
	  buttom6 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
	  buttom5 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
	  buttom4 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	  buttom3 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	  buttom2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
	  blue = HAL_GPIO_ReadPin(GPIOA, Blue_Pin);

	  if(blue == 1 && blue != prevBlue){
		  //initBase(&hspi3, 0x2A, address);
		  HAL_Delay(100);
		  printAllRegisters(&hspi3);
	  }

	  if(!buttom6){
		  //TextOut("forward");
		  remote = 1;
		  robot_vel = 1000;
		  ang = 0;
		  w_vel = 0;
		  //dribble_vel = 7;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);


	  }
	  else if(!buttom5){
		  //TextOut("sidewards");
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
	  else if(!buttom4){
		  //TextOut("turning");
		  remote = 1;
		  robot_vel = 0;
		  w_vel = 360;
		  rot_cclockwise = 1;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);

	  }
	  else{
		  //TextOut("turning");
		  //remote = 1;
		  w_vel = 000;
		  rot_cclockwise = 1;
		  robot_vel = 0000;
		  ang = 0;
		  kick_force = 0;
		  createRobotPacket(id, robot_vel, ang, rot_cclockwise, w_vel, kick_force, do_kick, chip, forced, dribble_cclockwise, dribble_vel, madeUpPacket);

	  }


	  if(remote == 1){

		  for(int i = 0; i < 12; i++){
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


	  prevButtom6 = buttom6;
	  prevButtom5 = buttom5;
	  prevBlue = blue;
	  waitAck(&hspi3, usbData[0] >> 4);


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
    Error_Handler();
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
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
