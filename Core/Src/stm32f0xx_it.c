/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint8_t us_delay_flag;

volatile uint8_t spi2State = SPI2_FREE;
volatile uint8_t spi2Buff[SPI2BUFFSIZE] = {0};
volatile uint8_t spi2BuffIndx = 0;
volatile uint8_t firstMsg = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    /* USER CODE BEGIN LL_EXTI_LINE_12 */
    if(LL_GPIO_IsInputPinSet(SPI2_CS_GPIO_Port, SPI2_CS_Pin))//CS pin went high -> start of transfer
    {
    	spi2State = SPI2_BUSY_RECEPTION;
    	LL_SPI_Enable(SPI2);
    	LL_SPI_EnableIT_RXNE(SPI2);
    	LL_SPI_EnableIT_TXE(SPI2);
    	LL_GPIO_SetOutputPin(SPI2_EXT_CTR_BSY_GPIO_Port, SPI2_EXT_CTR_BSY_Pin);
    }
    else///CS pin went low -> end of transfer
    {
    	LL_SPI_DisableIT_RXNE(SPI2);
    	LL_SPI_DisableIT_TXE(SPI2);
    	LL_SPI_Disable(SPI2);

    	spi2State = SPI2_END_OF_TRANSFER;
    	volatile uint8_t rxcmd = spi2Buff[0];
    	spi2Buff[spi2BuffIndx] = 0;//zero terminating the string, because it might not be. spi2BuffIndx increment was done in the receive interrupt, so this points after the last received byte
    	//uint8_t rxlen = spi2Buff[1];

    	if(firstMsg==0)//if first msg arrives we clear the bitmap from the screen
    	{
    		delete_disp_mat();
    		print_disp_mat();
    		firstMsg=1;
    	}


    	switch(rxcmd)
    	{
    		case PrintFilenameTXT_cmd:	fill_rectangle_x1y1_x2y2(0, 0, 8, 127, Pixel_off);
    									write_text_H(0, 55, (char*)&spi2Buff[2], Pixel_on, size_5x8);
    									print_disp_mat();
    									break;

    		case PrintCNCcmd_cmd:	fill_rectangle_x1y1_x2y2(10, 0, 18, 127, Pixel_off);
    								write_text_H(0, 45, (char*)&spi2Buff[2], Pixel_on, size_5x8);
    								print_disp_mat();
    								break;

    		case PrintInfo_cmd:		fill_rectangle_x1y1_x2y2(20, 0, 28, 127, Pixel_off);
									write_text_H(0, 35, (char*)&spi2Buff[2], Pixel_on, size_5x8);
									print_disp_mat();
									break;

    		case PrintCNCcmdAndLineNum_cmd:	char* splitpos = strchr((char*)&spi2Buff[2], '_');//search for the '_'character that is between the 2 strings to print
    										if(splitpos != NULL)
    										{
    											char printcmdstr[SPI2BUFFSIZE] = {0};//too long, can contain much more text than we are able to print to the screen in one line, but it is fine
    											uint8_t str1len = (splitpos - (char*)&spi2Buff[2]);

    											char linenumstr [SPI2BUFFSIZE] = {0};
    											uint8_t str2len = ( strlen((char*)&spi2Buff[2]) - (str1len+1));

    											memcpy(printcmdstr, (char*)&spi2Buff[2], str1len);
    											memcpy(linenumstr, (char*)&spi2Buff[2+str1len+1], str2len);

    											fill_rectangle_x1y1_x2y2(10, 0, 18, 127, Pixel_off);
												write_text_H(0, 45, printcmdstr, Pixel_on, size_5x8);

												fill_rectangle_x1y1_x2y2(30, 0, 38, 127, Pixel_off);
												write_text_H(0, 25, linenumstr, Pixel_on, size_5x8);

												print_disp_mat();
    										}

											break;

    		default:
					 break;
    	}


    	spi2BuffIndx = 0;
    	spi2State = SPI2_FREE;
    	LL_GPIO_ResetOutputPin(SPI2_EXT_CTR_BSY_GPIO_Port, SPI2_EXT_CTR_BSY_Pin);
    }
    /* USER CODE END LL_EXTI_LINE_12 */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM15 global interrupt.
  */
void TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_IRQn 0 */
	if(LL_TIM_IsActiveFlag_CC1(TIM15))
	{
		LL_TIM_ClearFlag_CC1(TIM15);
		LL_TIM_DisableCounter(TIM15);
		LL_TIM_CC_DisableChannel(TIM15, LL_TIM_CHANNEL_CH1);
		us_delay_flag=1;
	}
  /* USER CODE END TIM15_IRQn 0 */
  /* USER CODE BEGIN TIM15_IRQn 1 */

  /* USER CODE END TIM15_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */
	if(LL_SPI_IsActiveFlag_RXNE(SPI2))
	{
		if(spi2BuffIndx<SPI2BUFFSIZE)
		{
			spi2Buff[spi2BuffIndx] = LL_SPI_ReceiveData8(SPI2);
			spi2BuffIndx++;
		}

	}

	if(LL_SPI_IsActiveFlag_OVR(SPI2))
	{
		LL_SPI_ClearFlag_OVR(SPI2);
	}

	if(LL_SPI_IsActiveFlag_TXE(SPI2))
	{
		LL_SPI_TransmitData8(SPI2, 0x55);//dummy transfer
	}
  /* USER CODE END SPI2_IRQn 0 */
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
