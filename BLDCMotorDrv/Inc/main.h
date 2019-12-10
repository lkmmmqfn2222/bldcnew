/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define V_BUS_Pin GPIO_PIN_1
#define V_BUS_GPIO_Port GPIOC
#define M_IA_Pin GPIO_PIN_6
#define M_IA_GPIO_Port GPIOA
#define M_IB_Pin GPIO_PIN_7
#define M_IB_GPIO_Port GPIOA
#define M_IC_Pin GPIO_PIN_4
#define M_IC_GPIO_Port GPIOC
#define RUN_Pin GPIO_PIN_5
#define RUN_GPIO_Port GPIOC
#define POT_Pin GPIO_PIN_0
#define POT_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_1
#define STOP_GPIO_Port GPIOB
#define UP_Pin GPIO_PIN_10
#define UP_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_11
#define DOWN_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_12
#define DIR_GPIO_Port GPIOB
#define PWM1_L_Pin GPIO_PIN_13
#define PWM1_L_GPIO_Port GPIOB
#define PWM2_L_Pin GPIO_PIN_14
#define PWM2_L_GPIO_Port GPIOB
#define PWM3_L_Pin GPIO_PIN_15
#define PWM3_L_GPIO_Port GPIOB
#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOC
#define HALL_B_Pin GPIO_PIN_7
#define HALL_B_GPIO_Port GPIOC
#define HALL_C_Pin GPIO_PIN_8
#define HALL_C_GPIO_Port GPIOC
#define PWM1_H_Pin GPIO_PIN_8
#define PWM1_H_GPIO_Port GPIOA
#define PWM2_H_Pin GPIO_PIN_9
#define PWM2_H_GPIO_Port GPIOA
#define PWM3_H_Pin GPIO_PIN_10
#define PWM3_H_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
