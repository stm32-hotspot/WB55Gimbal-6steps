
/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team,ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044,the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

/**
  * @brief  PI / PID Speed loop parameters Motor 1.
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)(PERIODMAX * SP_KIDIV),
  .wLowerIntegralLimit = 0,
  .hUpperOutputLimit   = (int16_t)PERIODMAX,
  .hLowerOutputLimit   = 0,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain          = 0x0000U,
  .hKdDivisor          = 0x0000U,
  .hKdDivisorPOW2      = 0x0000U,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1.
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz             = MEDIUM_FREQUENCY_TASK_RATE,
  .MaxAppPositiveMecSpeedUnit = (uint16_t)(MAX_APPLICATION_SPEED_UNIT),
  .MinAppPositiveMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
  .MaxAppNegativeMecSpeedUnit = (int16_t)(-MIN_APPLICATION_SPEED_UNIT),
  .MinAppNegativeMecSpeedUnit = (int16_t)(-MAX_APPLICATION_SPEED_UNIT),
  .MaxPositiveDutyCycle       = (uint16_t)PERIODMAX / 2U,
  .ModeDefault                = DEFAULT_CONTROL_MODE,
  .MecSpeedRefUnitDefault     = (int16_t)(DEFAULT_TARGET_SPEED_UNIT),
  .DutyCycleRefDefault        = 0,
};

RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = (ENABLE_SL_ALGO_FROM_PHASE-1u),
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED_UNIT,
  .ParamsData  =
  {
    {(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_UNIT),
    (uint16_t)PHASE1_VOLTAGE_DPP,&RevUpControlM1.ParamsData[1]},
    {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_UNIT),
    (uint16_t)PHASE2_VOLTAGE_DPP,&RevUpControlM1.ParamsData[2]},
    {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_UNIT),
    (uint16_t)PHASE3_VOLTAGE_DPP,&RevUpControlM1.ParamsData[3]},
    {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_UNIT),
    (uint16_t)PHASE4_VOLTAGE_DPP,&RevUpControlM1.ParamsData[4]},
    {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_UNIT),
    (uint16_t)PHASE5_VOLTAGE_DPP,(void*)MC_NULL},
  },
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class.
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{

  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },

  .hSpeedSamplingFreqHz        = MEDIUM_FREQUENCY_TASK_RATE,
  .hTransitionSteps            = (int16_t)((TF_REGULATION_RATE * TRANSITION_DURATION) / 1000.0),
};

Bemf_ADC_Handle_t Bemf_ADC_M1 =
{
  ._Super =
  {
    .bElToMecRatio             = POLE_PAIR_NUM,
    .hMaxReliableMecSpeedUnit  = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
    .hMinReliableMecSpeedUnit  = (uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS,
    .hMaxReliableMecAccelUnitP = 65535,
    .hMeasurementFrequency     = TF_REGULATION_RATE_SCALED,
    .DPPConvFactor             = DPP_CONV_FACTOR,
  },
  .Pwm_OFF = {
    .AdcThresholdDown = BEMF_THRESHOLD_DOWN,
    .AdcThresholdUp = BEMF_THRESHOLD_UP,
    .SamplingPoint = BEMF_ADC_TRIG_TIME,
  },
  .Pwm_ON = {
    .AdcThresholdDown = BEMF_THRESHOLD_DOWN_ON,
    .AdcThresholdUp = BEMF_THRESHOLD_UP_ON,
    .SamplingPoint = BEMF_ADC_TRIG_TIME_ON,
  },

  .OnSensingEnThres            = BEMF_PWM_ON_ENABLE_THRES,
  .OnSensingDisThres           = BEMF_PWM_ON_DISABLE_THRES,
  .Zc2CommDelay                = ZCD_TO_COMM,
  .pParams_str                 = &Bemf_ADC_ParamsM1,
  .TIMClockFreq                = SYSCLK_FREQ,
  .PWMFreqScaling              = PWM_FREQ_SCALING,
  .SpeedSamplingFreqHz         = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize             = BEMF_AVERAGING_FIFO_DEPTH,
  .LowFreqTimerPsc             = LF_TIMER_PSC,
  .MinStartUpValidSpeed        = OBS_MINIMUM_SPEED_UNIT,
  .StartUpConsistThreshold     = NB_CONSECUTIVE_TESTS,

  .DemagParams =
  {
    .DemagMinimumSpeedUnit     = DEMAG_MINIMUM_SPEED,
    .RevUpDemagSpeedConv       = DEMAG_REVUP_CONV_FACTOR,
    .RunDemagSpeedConv         = DEMAG_RUN_CONV_FACTOR,
    .DemagMinimumThreshold     = MIN_DEMAG_TIME,
  },

  .DriveMode                   = DEFAULT_DRIVE_MODE,
};

PWMC_ThreePwm_Handle_t PWM_Handle_M1 =
{
  {
    .pFctSwitchOffPwm          = &ThreePwm_SwitchOffPWM,
    .pFctSwitchOnPwm           = &ThreePwm_SwitchOnPWM,
    .pFctSetADCTriggerChannel  = &ThreePwm_SetADCTriggerChannel,
    .pFctTurnOnLowSides        = &ThreePwm_TurnOnLowSides,
    .pGetFastDemagFlag         = &ThreePwm_FastDemagFlag,
    .pGetQuasiSynchFlag        = MC_NULL,
    .StartCntPh                = PWM_PERIOD_CYCLES,
    .SWerror                   = 0,
    .TurnOnLowSidesAction      = false,
    .Motor                     = 0,
    .PWMperiod                 = PWM_PERIOD_CYCLES,
    .DemagCounterThreshold     = MIN_DEMAG_TIME,
    .OverCurrentFlag  = 0,
    .OverVoltageFlag  = 0,
    .BrakeActionLock  = 0,
    .driverProtectionFlag = false,
  },
  .FastDemag  = false,
  .Oversampling  = false,
  .pParams_str = &ThreePwm_ParamsM1,
};

/**
  * temperature sensor parameters Motor 1.
  */
RegConv_t TempRegConv_M1 =
{
    .regADC                = ADC1,
    .channel               = MC_ADC_CHANNEL_3,
  .samplingTime          = M1_TEMP_SAMPLING_TIME,
};

NTC_Handle_t TempSensor_M1 =
{
  .bSensorType             = REAL_SENSOR,

  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (int16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)((V0_V * 65536) / ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,
};

/* Bus voltage sensor value filter buffer */
static uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1.
  */
RegConv_t VbusRegConv_M1 =
{
    .regADC                   = ADC1,
    .channel                  = MC_ADC_CHANNEL_5,
    .samplingTime             = M1_VBUS_SAMPLING_TIME,
};

RDivider_Handle_t BusVoltageSensor_M1 =
{
  ._Super =
  {
    .SensorType               = REAL_SENSOR,
    .ConversionFactor         = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
  },

  .LowPassFilterBW            =  M1_VBUS_SW_FILTER_BW_FACTOR,
  .OverVoltageThreshold       = OVERVOLTAGE_THRESHOLD_d,
  .OverVoltageThresholdLow    = OVERVOLTAGE_THRESHOLD_d,
  .OverVoltageHysteresisUpDir = true,
  .UnderVoltageThreshold      =  UNDERVOLTAGE_THRESHOLD_d,
  .aBuffer                    = RealBusVoltageSensorFilterBufferM1,
};

MCI_Handle_t Mci[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS]    = {&SpeednTorqCtrlM1};
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS] = {&TempSensor_M1};
/* USER CODE BEGIN Additional configuration */
/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/

