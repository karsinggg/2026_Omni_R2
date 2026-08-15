/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PNEU_TOP_Pin GPIO_PIN_3
#define PNEU_TOP_GPIO_Port GPIOE
#define CASCADE_MIN_Pin GPIO_PIN_4
#define CASCADE_MIN_GPIO_Port GPIOE
#define PS4_SDA_Pin GPIO_PIN_0
#define PS4_SDA_GPIO_Port GPIOF
#define PS4_SCL_Pin GPIO_PIN_1
#define PS4_SCL_GPIO_Port GPIOF
#define IR_1_Pin GPIO_PIN_8
#define IR_1_GPIO_Port GPIOF
#define IR_4_Pin GPIO_PIN_10
#define IR_4_GPIO_Port GPIOF
#define CASCADE_MAX_Pin GPIO_PIN_0
#define CASCADE_MAX_GPIO_Port GPIOC
#define CS_MECH1_Pin GPIO_PIN_0
#define CS_MECH1_GPIO_Port GPIOA
#define CS_MECH2_Pin GPIO_PIN_4
#define CS_MECH2_GPIO_Port GPIOA
#define IR_3_Pin GPIO_PIN_14
#define IR_3_GPIO_Port GPIOD
#define IR_2_Pin GPIO_PIN_15
#define IR_2_GPIO_Port GPIOD
#define BNO_RST_Pin GPIO_PIN_5
#define BNO_RST_GPIO_Port GPIOG
#define BNO_HINT_Pin GPIO_PIN_6
#define BNO_HINT_GPIO_Port GPIOG
#define BNO_BOOT_Pin GPIO_PIN_8
#define BNO_BOOT_GPIO_Port GPIOG
#define CS_STEP_UP_Pin GPIO_PIN_9
#define CS_STEP_UP_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PNEU_BOT_Pin GPIO_PIN_4
#define PNEU_BOT_GPIO_Port GPIOD
#define PNEU_EXTEND_Pin GPIO_PIN_5
#define PNEU_EXTEND_GPIO_Port GPIOD
#define CS_SWERVE_Pin GPIO_PIN_9
#define CS_SWERVE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
