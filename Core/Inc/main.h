/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define P1_Pin GPIO_PIN_3
#define P1_GPIO_Port GPIOA
#define V1_Pin GPIO_PIN_4
#define V1_GPIO_Port GPIOA
#define P2_Pin GPIO_PIN_5
#define P2_GPIO_Port GPIOA
#define V2_Pin GPIO_PIN_6
#define V2_GPIO_Port GPIOA
#define P3_Pin GPIO_PIN_7
#define P3_GPIO_Port GPIOA
#define V3_Pin GPIO_PIN_8
#define V3_GPIO_Port GPIOA
#define V6_Pin GPIO_PIN_15
#define V6_GPIO_Port GPIOA
#define P6_Pin GPIO_PIN_3
#define P6_GPIO_Port GPIOB
#define V5_Pin GPIO_PIN_4
#define V5_GPIO_Port GPIOB
#define P5_Pin GPIO_PIN_5
#define P5_GPIO_Port GPIOB
#define V4_Pin GPIO_PIN_6
#define V4_GPIO_Port GPIOB
#define P4_Pin GPIO_PIN_7
#define P4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
