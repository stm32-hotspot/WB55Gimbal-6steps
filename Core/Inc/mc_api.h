
/**
  ******************************************************************************
  * @file    mc_api.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file defines the high level interface of the Motor Control SDK.
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
  * @ingroup MCIAPI
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_API_H
#define MC_API_H

#include "mc_type.h"
#include "mc_interface.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup CAI
  * @{
  */

/** @addtogroup MCIAPI
  * @{
  */

/* Starts Motor 1 */
bool MC_StartMotor1(void);

/* Stops Motor 1 */
bool MC_StopMotor1(void);

/* Programs a Speed ramp for Motor 1 */
void MC_ProgramSpeedRampMotor1(int16_t hFinalSpeed, uint16_t hDurationms);

/* Returns the current PWM duty cycle reference for Motor 1 */
int16_t MCI_GetDutyCycleRefMotor1(void);

/* Returns the state of the last submited command for Motor 1 */
MCI_CommandState_t MC_GetCommandStateMotor1(void);

/* Stops the execution of the current speed ramp for Motor 1 if any */
bool MC_StopSpeedRampMotor1(void);

/* Stops the execution of the on going ramp for Motor 1 if any.
   Note: this function is deprecated and should not be used anymore. It will be removed in a future version. */
void MC_StopRampMotor1(void);

/* Returns true if the last submited ramp for Motor 1 has completed, false otherwise */
bool MC_HasRampCompletedMotor1(void);

/* Returns the current mechanical rotor speed reference set for Motor 1, expressed in the unit defined by #SPEED_UNIT */
int16_t MC_GetMecSpeedReferenceMotor1(void);

/* Returns the current mechanical rotor speed reference set for Motor 1, expressed in rpm */
float_t MC_GetMecSpeedReferenceMotor1_F(void);

/* Returns the last computed average mechanical rotor speed for Motor 1, expressed in the unit defined by #SPEED_UNIT */
int16_t MC_GetMecSpeedAverageMotor1(void);

/* Returns the last computed average mechanical rotor speed for Motor 1, expressed in rpm */
float_t MC_GetAverageMecSpeedMotor1_F(void);

/* Returns the final speed of the last ramp programmed for Motor 1, if this ramp was a speed ramp */
int16_t MC_GetLastRampFinalSpeedMotor1(void);

/* Returns the final speed of the last ramp programmed for Motor 1, if this ramp was a speed ramp */
float_t MC_GetLastRampFinalSpeedM1_F(void);

/* Returns the current Control Mode for Motor 1 (either Speed or Torque) */
MC_ControlMode_t MC_GetControlModeMotor1(void);

/* Returns the direction imposed by the last command on Motor 1 */
int16_t MC_GetImposedDirectionMotor1(void);

/* Returns the current reliability of the speed sensor used for Motor 1 */
bool MC_GetSpeedSensorReliabilityMotor1(void);

/* Acknowledge a Motor Control fault on Motor 1 */
bool MC_AcknowledgeFaultMotor1(void);

/* Returns a bitfiled showing faults that occured since the State Machine of Motor 1 was moved to FAULT_NOW state */
uint16_t MC_GetOccurredFaultsMotor1(void);

/* Returns a bitfield showing all current faults on Motor 1 */
uint16_t MC_GetCurrentFaultsMotor1(void);

/* returns the current state of Motor 1 state machine */
MCI_State_t  MC_GetSTMStateMotor1(void);

/* Call the Profiler command */
uint8_t MC_ProfilerCommand (uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MC_API_H */
/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
