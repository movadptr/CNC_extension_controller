/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "PCD8544_48x84_LCD.h"
#include "SSD1315_128x64_Oled.h"
#include "Fonts_and_bitmaps_FLASH.h"
#include "disp_fgv.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void msDelay(uint32_t val);
void usDelay(uint32_t val);
void usDelay_noblock(uint32_t val);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_EXT_CTR_BSY_Pin LL_GPIO_PIN_14
#define SPI2_EXT_CTR_BSY_GPIO_Port GPIOC
#define SPI2_CS_Pin LL_GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_CS_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

#define USE_INTERRUPT_US_DELAY
//#define USE_BLOCKING_US_DELAY


#define SPI2BUFFSIZE	100U

#define SPI2_FREE				0x01
#define SPI2_BUSY_RECEPTION		0x02
#define SPI2_BUSY_TRANSMIT		0x03
#define SPI2_END_OF_TRANSFER	0x04
#define SPI2_ERROR				0xff


//command byte defines for extension shield
#define	PrintFilenameTXT_cmd		0x01
#define	PrintCNCcmd_cmd				0x02
#define PrintInfo_cmd				0x03
#define PrintCNCcmdAndLineNum_cmd	0x04

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
