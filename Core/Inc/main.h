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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STM_TX_EN_Pin GPIO_PIN_13
#define STM_TX_EN_GPIO_Port GPIOC
#define RF_LOG_EN_Pin GPIO_PIN_14
#define RF_LOG_EN_GPIO_Port GPIOC
#define STM_RX_MIX_EN_Pin GPIO_PIN_0
#define STM_RX_MIX_EN_GPIO_Port GPIOA
#define MODULUJACY_Pin GPIO_PIN_6
#define MODULUJACY_GPIO_Port GPIOA
#define AM1_Pin GPIO_PIN_7
#define AM1_GPIO_Port GPIOA
#define MIC_SECONDARY_Pin GPIO_PIN_0
#define MIC_SECONDARY_GPIO_Port GPIOB
#define AUDIO_OUT_Pin GPIO_PIN_1
#define AUDIO_OUT_GPIO_Port GPIOB
#define MIC_PRIMARY_Pin GPIO_PIN_15
#define MIC_PRIMARY_GPIO_Port GPIOB
#define VBAT_Pin GPIO_PIN_8
#define VBAT_GPIO_Port GPIOA
#define STM_5V_DET_Pin GPIO_PIN_9
#define STM_5V_DET_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_10
#define BTN2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_5
#define BTN1_GPIO_Port GPIOB
#define STM_RX_EN_Pin GPIO_PIN_7
#define STM_RX_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
