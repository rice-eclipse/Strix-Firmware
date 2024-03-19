/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define BMP388_Int_Pin GPIO_PIN_2
#define BMP388_Int_GPIO_Port GPIOE
#define BMP388_NCS_Pin GPIO_PIN_3
#define BMP388_NCS_GPIO_Port GPIOE
#define Pyro_A_Trigger_Pin GPIO_PIN_2
#define Pyro_A_Trigger_GPIO_Port GPIOB
#define Pyro_B_Trigger_Pin GPIO_PIN_7
#define Pyro_B_Trigger_GPIO_Port GPIOE
#define Pyro_C_Trigger_Pin GPIO_PIN_8
#define Pyro_C_Trigger_GPIO_Port GPIOE
#define Pyro_D_Trigger_Pin GPIO_PIN_9
#define Pyro_D_Trigger_GPIO_Port GPIOE
#define Pyro_E_Trigger_Pin GPIO_PIN_12
#define Pyro_E_Trigger_GPIO_Port GPIOE
#define Pyro_F_Trigger_Pin GPIO_PIN_13
#define Pyro_F_Trigger_GPIO_Port GPIOE
#define Continuity_LED_D_Pin GPIO_PIN_14
#define Continuity_LED_D_GPIO_Port GPIOB
#define Continuity_LED_C_Pin GPIO_PIN_15
#define Continuity_LED_C_GPIO_Port GPIOB
#define Continuity_LED_B_Pin GPIO_PIN_10
#define Continuity_LED_B_GPIO_Port GPIOD
#define Continuity_LED_A_Pin GPIO_PIN_11
#define Continuity_LED_A_GPIO_Port GPIOD
#define Continuity_LED_E_Pin GPIO_PIN_12
#define Continuity_LED_E_GPIO_Port GPIOD
#define Continuity_LED_F_Pin GPIO_PIN_13
#define Continuity_LED_F_GPIO_Port GPIOD
#define Status_LED_Pin GPIO_PIN_14
#define Status_LED_GPIO_Port GPIOD
#define LIS3MDL_NCS_Pin GPIO_PIN_15
#define LIS3MDL_NCS_GPIO_Port GPIOA
#define LIS3MDL_Int_Pin GPIO_PIN_0
#define LIS3MDL_Int_GPIO_Port GPIOD
#define KX134_Int_Pin GPIO_PIN_1
#define KX134_Int_GPIO_Port GPIOD
#define KX134_NCS_Pin GPIO_PIN_2
#define KX134_NCS_GPIO_Port GPIOD
#define ADXL375_NCS_Pin GPIO_PIN_6
#define ADXL375_NCS_GPIO_Port GPIOB
#define ADXL375_Int_Pin GPIO_PIN_7
#define ADXL375_Int_GPIO_Port GPIOB
#define BMI088_Gyro_NCS_Pin GPIO_PIN_8
#define BMI088_Gyro_NCS_GPIO_Port GPIOB
#define BMI088_Gyro_Int_Pin GPIO_PIN_9
#define BMI088_Gyro_Int_GPIO_Port GPIOB
#define BMI088_Accel_NCS_Pin GPIO_PIN_0
#define BMI088_Accel_NCS_GPIO_Port GPIOE
#define BMI088_Accel_Int_Pin GPIO_PIN_1
#define BMI088_Accel_Int_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
