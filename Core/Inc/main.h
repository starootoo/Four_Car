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
#include "stm32f1xx_hal.h"

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
#define SCL_Pin GPIO_PIN_13
#define SCL_GPIO_Port GPIOC
#define SDL_Pin GPIO_PIN_14
#define SDL_GPIO_Port GPIOC
#define Encoderl_A_Pin GPIO_PIN_0
#define Encoderl_A_GPIO_Port GPIOA
#define Encoderl_B_Pin GPIO_PIN_1
#define Encoderl_B_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_6
#define BIN1_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_0
#define PWMA_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDL_Pin GPIO_PIN_11
#define OLED_SDL_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_12
#define Echo_GPIO_Port GPIOB
#define Huidu10_Pin GPIO_PIN_13
#define Huidu10_GPIO_Port GPIOB
#define Huidu9_Pin GPIO_PIN_14
#define Huidu9_GPIO_Port GPIOB
#define Huidu8_Pin GPIO_PIN_15
#define Huidu8_GPIO_Port GPIOB
#define Huidu7_Pin GPIO_PIN_8
#define Huidu7_GPIO_Port GPIOA
#define Huidu6_Pin GPIO_PIN_9
#define Huidu6_GPIO_Port GPIOA
#define Huidu5_Pin GPIO_PIN_10
#define Huidu5_GPIO_Port GPIOA
#define Huidu4_Pin GPIO_PIN_11
#define Huidu4_GPIO_Port GPIOA
#define Huidu3_Pin GPIO_PIN_12
#define Huidu3_GPIO_Port GPIOA
#define Trig_Pin GPIO_PIN_15
#define Trig_GPIO_Port GPIOA
#define key3_Pin GPIO_PIN_3
#define key3_GPIO_Port GPIOB
#define key3_EXTI_IRQn EXTI3_IRQn
#define key2_Pin GPIO_PIN_4
#define key2_GPIO_Port GPIOB
#define key1_Pin GPIO_PIN_5
#define key1_GPIO_Port GPIOB
#define key1_EXTI_IRQn EXTI9_5_IRQn
#define Encoderr_A_Pin GPIO_PIN_6
#define Encoderr_A_GPIO_Port GPIOB
#define Encoderr_B_Pin GPIO_PIN_7
#define Encoderr_B_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDL_Pin GPIO_PIN_9
#define MPU_SDL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
