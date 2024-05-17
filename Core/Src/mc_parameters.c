
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
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

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"

#include "pwmc_3pwm.h"
#include "g4xx_bemf_ADC_fdbk.h"
/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
const ThreePwm_Params_t ThreePwm_ParamsM1 =
{
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = 1,
  .TIMx              = TIM1,

/* PWM Driving signals initialization ----------------------------------------*/
  .OCPolarity        = LL_TIM_OCPOLARITY_HIGH,
  .pwm_en_u_port     = M1_PWM_EN_U_GPIO_Port,
  .pwm_en_u_pin      = M1_PWM_EN_U_Pin,
  .pwm_en_v_port     = M1_PWM_EN_V_GPIO_Port,
  .pwm_en_v_pin      = M1_PWM_EN_V_Pin,
  .pwm_en_w_port     = M1_PWM_EN_W_GPIO_Port,
  .pwm_en_w_pin      = M1_PWM_EN_W_Pin,
};

/**
  * @brief  Current sensor parameters Motor 1 - single shunt phase shift
  */
const Bemf_ADC_Params_t Bemf_ADC_ParamsM1 =
{
  .LfTim                  = TIM2,
  .LfTimerChannel         = LL_TIM_CHANNEL_CH1,        /*!< Channel of the LF timer used for speed measurement */
  .gpio_divider_available = true,               /*!< Availability of the GPIO port enabling the bemf resistor divider */
  .bemf_divider_port      = M1_BEMF_DIVIDER_GPIO_Port, /*!< GPIO port of OnSensing divider enabler */
  .bemf_divider_pin       = M1_BEMF_DIVIDER_Pin,
  /*!< Pointer to the ADC */
  .pAdc                   = {ADC1, ADC1, ADC1},
  .AdcChannel             = {MC_ADC_CHANNEL_1, MC_ADC_CHANNEL_4, MC_ADC_CHANNEL_2},
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/

