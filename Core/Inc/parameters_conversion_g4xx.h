
/**
  ******************************************************************************
  * @file    parameters_conversion_g4xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32G4 Family.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PARAMETERS_CONVERSION_G4XX_H
#define PARAMETERS_CONVERSION_G4XX_H
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      64000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz  64 /* Actual TIM clk including Timer clock divider*/
#define ADC_CLK_MHz     16
#define HALL_TIM_CLK    64000000uL
#define APB1TIM_FREQ 64000000uL

/*************************  IRQ Handler Mapping  *********************/
//#define TIMx_UP_M1_IRQHandler TIM1_UP_TIM16_IRQHandler

//#define TIMx_BRK_M1_IRQHandler TIM1_BRK_TIM15_IRQHandler
//#define BEMF_READING_IRQHandler          ADC1_2_IRQHandler
//#define PERIOD_COMM_IRQHandler              TIM2_IRQHandler

#define ADC_TRIG_CONV_LATENCY_CYCLES 3.5
#define ADC_SAR_CYCLES 12.5

#define M1_VBUS_SW_FILTER_BW_FACTOR     6u

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

#endif /*PARAMETERS_CONVERSION_G4XX_H*/

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
