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
#include "basestationNRF24.h"
#include "TextOut.h"
#include "packing.h"
#include "discoveryboard.h"


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



	//fun2();
  	//ensure that LED4 is on during start-up
	HAL_GPIO_WritePin(GPIOE, LD4_Pin, GPIO_PIN_SET);

	//TextOut("Initializing Basestation.\n");
	//initializing address with a pseudo value ("BAD FOOD").
	//the address will be overwritten as soon as we have decided to which robot we talk
	//uint8_t address[5] = {0xBA, 0xAA, 0xAD, 0xF0, 0x0D};
	initBase(&hspi3, 78);
	/*
	GPIO_PinState button6;
	GPIO_PinState button5;
	GPIO_PinState button4;

	uint8_t remote = 0;
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//Apparently we need to wait about 20 seconds after connecting the bastation
	//Only then we would receive data without dropping anything.
	//This only applies to some systems (Ubuntu Linux) but not all (works on Windows without warm-up).

	uint8_t useWarmup = 0;
	for(uint8_t i=40; i>0; i--) {
		//sprintf(smallStrBuffer, "Warming Up Serial Connection. Seconds left:  %i   ", i);
		if(!useWarmup) break;
		sprintf(smallStrBuffer, "%i  ", i);
		TextOut(smallStrBuffer);
		HAL_Delay(500);
	}
	TextOut("Starting Execution \n\n\n");


	int id = 10;
	/*
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
	uint8_t blue = 0;
	*/

	uint8_t debug_transmit_repeatedly = 1;

	uint8_t pktNum = 0;
	//uint8_t toggleMe = 1;
	HAL_Delay(1000);








	//new testing section
	/*
	 * Test case:
	 * create a roboData Struct
	 * fill it with pseudo-random data
	 * convert it to a packet
	 * convert the packet back to roboData
	 * compare the original roboData with the output.
	 */
	roboData robInput;
	roboData robOutput;

	uint8_t roboPkt[40]; //larger than it needs to be..


	while(1) {

		//filling struct with pseudo-random data
		robInput.id = (uint8_t) HAL_GetTick()&0x1f;
		robInput.rho = (uint16_t) HAL_GetTick()&0x7ff;
		robInput.theta = (uint16_t) HAL_GetTick()&0x7ff;
		robInput.driving_reference = (uint8_t) HAL_GetTick()&1;
		robInput.use_cam_info = (uint8_t) HAL_GetTick()&1;
		robInput.velocity_angular = (int16_t) HAL_GetTick()&0x1ff;
		robInput.debug_info = (uint8_t) HAL_GetTick()&1;
		robInput.do_kick = (uint8_t) HAL_GetTick()&1;
		robInput.do_chip = (uint8_t) HAL_GetTick()&1;
		robInput.kick_chip_forced = (uint8_t) HAL_GetTick()&1;
		robInput.kick_chip_power = (uint8_t) HAL_GetTick()&0xff;
		robInput.velocity_dribbler = (uint8_t) HAL_GetTick()&0xff;
		robInput.geneva_drive_state = (uint8_t) HAL_GetTick()&7;
		robInput.cam_position_x = (uint16_t) HAL_GetTick()&0x1fff;
		robInput.cam_position_y = (uint16_t) HAL_GetTick()&0x1fff;
		robInput.cam_rotation = (uint16_t) HAL_GetTick()&0x7ff;



		//converting struct to packet
		robotDataToPacket(&robInput, roboPkt);

		//converting back to struct
		packetToRoboData(roboPkt, &robOutput);

		//comparing input with output

		//hacky!! D:
		if(strcmp(((const char*) &robInput), ((const char*) &robOutput)) == 0) {
			//both are equal
			TextOut("\nAlright! Input == Output.\n");
		} else {
			//something went wrong
			TextOut("\nERROR!!!!! One or more fields have a mismatch!\n");

			//printing Data
			TextOut("Inputs were:\n");
			sprintf(smallStrBuffer, "ID: %i\n", robInput.id);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "Rho: %i\n", robInput.rho);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "Theta: %i\n", robInput.theta);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "driving_reference: %i\n", robInput.driving_reference);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "use_cam_info: %i\n", robInput.use_cam_info);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "velocity_angular: %i\n", robInput.velocity_angular);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "debug_info: %i\n", robInput.debug_info);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "do_kick: %i\n", robInput.do_kick);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "do_chip: %i\n", robInput.do_chip);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "kick_chip_forced: %i\n", robInput.kick_chip_forced);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "kick_chip_power: %i\n", robInput.kick_chip_power);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "velocity_dribbler: %i\n", robInput.velocity_dribbler);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "geneva_drive_state: %i\n", robInput.geneva_drive_state);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "cam_position_x: %i\n", robInput.cam_position_x);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "cam_position_y: %i\n", robInput.cam_position_y);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "cam_rotation: %i\n", robInput.cam_rotation);
			TextOut(smallStrBuffer);
			TextOut("------------------------------------------------------------------------\n");
			TextOut("Printing Outputs: \n");
			sprintf(smallStrBuffer, "ID: %i\n", robOutput.id);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "Rho: %i\n", robOutput.rho);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "Theta: %i\n", robOutput.theta);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "driving_reference: %i\n", robOutput.driving_reference);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "use_cam_info: %i\n", robOutput.use_cam_info);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "velocity_angular: %i\n", robOutput.velocity_angular);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "debug_info: %i\n", robOutput.debug_info);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "do_kick: %i\n", robOutput.do_kick);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "do_chip: %i\n", robOutput.do_chip);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "kick_chip_forced: %i\n", robOutput.kick_chip_forced);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "kick_chip_power: %i\n", robOutput.kick_chip_power);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "velocity_dribbler: %i\n", robOutput.velocity_dribbler);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "geneva_drive_state: %i\n", robOutput.geneva_drive_state);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "cam_position_x: %i\n", robOutput.cam_position_x);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "cam_position_y: %i\n", robOutput.cam_position_y);
			TextOut(smallStrBuffer);
			sprintf(smallStrBuffer, "cam_rotation: %i\n", robOutput.cam_rotation);
			TextOut(smallStrBuffer);
		}

		TextOut("\n\n\n");

		HAL_Delay(1000);
	}






	/*
	 * The code below is currently not in use
	 */

	while (1)
	{
		//button6 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
		//button5 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
		//button4 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
		//blue = HAL_GPIO_ReadPin(GPIOA, Blue_Pin); //button state of the blue user button of the Discovery board

		if(debug_transmit_repeatedly == 1) {
			createRobotPacket(id, 0, pktNum++, 0, 0, 0, 0, 0, 0, 0, 0, madeUpPacket);

			unsigned int retransmissionSum = 0;
			uint16_t lostpackets = 0;
			uint16_t emptyack = 0;
			uint16_t packetsToTransmit = 100;
			uint8_t verbose = 0;
			uint8_t interpacketdelay = 16; //delay in milliseconds between packets

			for(uint16_t i = 0; i<packetsToTransmit; i++) {
				sendPacket(madeUpPacket);

				uint8_t ack_payload[32];
				uint8_t payload_length;
				int8_t returncode;

				do {
					returncode = getAck(ack_payload, &payload_length);
				} while(returncode == -1);
				uint8_t retr = readReg(OBSERVE_TX)&0x0f;

				if(returncode == -2) {
					lostpackets++;
					if(verbose) {
						sprintf(smallStrBuffer, "%i. Packet lost.\n", (i+1));
						TextOut(smallStrBuffer);
					}

				} else if(returncode == 1) {
					if(verbose) {
						sprintf(smallStrBuffer, "%i. Packet delivered with %i retransmissions.\n", (i+1), retr);
						TextOut(smallStrBuffer);
					}
					retransmissionSum += retr;
				} else if(returncode == 0) {
					emptyack++;
					if(verbose) {
						sprintf(smallStrBuffer, "%i. Packet delivered with empty ACK!\n", (i+1));
						TextOut(smallStrBuffer);
					}
				}
				clearInterrupts();
				HAL_Delay(interpacketdelay);

			}
			uint8_t packetloss = (int)(lostpackets*100.0)/(packetsToTransmit*1.0);
			uint8_t emptyackprocent = (int) (emptyack*100.0)/(packetsToTransmit*1.0);
			sprintf(smallStrBuffer, "Packets TX'd: %i with a delay of %i ms, delivered: %i with retransmissions: %u, packet loss: %i %% (empty ack: %i %%) \n\n", packetsToTransmit, interpacketdelay,(packetsToTransmit-lostpackets), retransmissionSum, packetloss, emptyackprocent);
			TextOut(smallStrBuffer);
			continue; //skip to the next loop iteration
		}

		if(usbLength == 12){
			sendPacket(usbData);
			usbLength = 0;
		}


		if(usbLength != 0){
			//fun();
		}


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
