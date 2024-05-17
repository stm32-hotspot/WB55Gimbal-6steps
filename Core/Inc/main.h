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
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motorcontrol.h"
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
#define M1_BEMF_U_Pin GPIO_PIN_0
#define M1_BEMF_U_GPIO_Port GPIOC
#define M1_BEMF_W_Pin GPIO_PIN_1
#define M1_BEMF_W_GPIO_Port GPIOC
#define M1_TEMPERATURE_Pin GPIO_PIN_2
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define M1_BEMF_V_Pin GPIO_PIN_3
#define M1_BEMF_V_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_0
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_4
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_0
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define M1_PWM_EN_U_Pin GPIO_PIN_13
#define M1_PWM_EN_U_GPIO_Port GPIOB
#define M1_PWM_EN_V_Pin GPIO_PIN_14
#define M1_PWM_EN_V_GPIO_Port GPIOB
#define M1_PWM_EN_W_Pin GPIO_PIN_15
#define M1_PWM_EN_W_GPIO_Port GPIOB
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define M1_DP_Pin GPIO_PIN_11
#define M1_DP_GPIO_Port GPIOA
#define M1_BEMF_DIVIDER_Pin GPIO_PIN_10
#define M1_BEMF_DIVIDER_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_0
#define B2_GPIO_Port GPIOD
#define B2_EXTI_IRQn EXTI0_IRQn
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOD
#define B3_EXTI_IRQn EXTI1_IRQn
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
