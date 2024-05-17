
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common_sixstep.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

#define STOPPERMANENCY_MS              ((uint16_t)400)
#define STOPPERMANENCY_MS2             ((uint16_t)400)
#define STOPPERMANENCY_TICKS           (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)  / ((uint16_t)1000))
#define STOPPERMANENCY_TICKS2          (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2) / ((uint16_t)1000))
/* USER CODE END Private define */

#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)
/* Private variables----------------------------------------------------------*/

static SixStepVars_t SixStepVars[NBR_OF_MOTORS];

static PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];

static uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

static volatile uint8_t bMCBootCompleted = ((uint8_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS          (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0.000\
                                      * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS         (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES ((uint32_t)0\
                                      * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))
#define S16_90_PHASE_SHIFT             (int16_t)(65536/4)

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
void SixStep_Clear(uint8_t bMotor);
void SixStep_InitAdditionalMethods(uint8_t bMotor);
void SixStep_CalcSpeedRef(uint8_t bMotor);
static uint16_t SixStep_StatorController(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  if (MC_NULL == pMCIList)
  {
    /* Nothing to do */
  }
  else
  {

    bMCBootCompleted = (uint8_t )0;

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    ThreePwm_Init(&PWM_Handle_M1);
    ASPEP_start(&aspepOverUartA);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    BADC_Init (&Bemf_ADC_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &Bemf_ADC_M1._Super);

    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/
    VSS_Init(&VirtualSpeedSensorM1);

    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    RUC_Init(&RevUpControlM1,pSTC[M1],&VirtualSpeedSensorM1);

    /********************************************************/
    /*   Bus voltage sensor component initialization        */
    /********************************************************/
    (void)RCM_RegisterRegConv(&VbusRegConv_M1);
    RVBS_Init(&BusVoltageSensor_M1);

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    (void)RCM_RegisterRegConv(&TempRegConv_M1);
    NTC_Init(&TempSensor_M1);

    SixStep_Clear(M1);
    SixStepVars[M1].bDriveInput = EXTERNAL;
    MCI_Init(&Mci[M1], pSTC[M1], &SixStepVars[M1], pwmcHandle[M1] );
    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */
    pMCIList[M1] = &Mci[M1];

    /* Applicative hook in MCBoot() */
    MC_APP_BootHook();

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */

    bMCBootCompleted = 1U;
  }
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors.
 * - Safety Task.
 * - Power Factor Correction Task (if enabled).
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if (0U == bMCBootCompleted)
  {
    /* Nothing to do */
  }
  else
  {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that
     * it can overcome actions they initiated if needed */
    TSK_SafetyTask();

  }
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
    ThreePwm_SwitchOffPWM(pwmcHandle[motor]);

  SixStep_Clear(motor);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (((uint8_t)1) == bMCBootCompleted)
  {
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();

      /* Applicative hook at end of Medium Frequency for Motor 1 */
      MC_APP_PostMediumFrequencyHook_M1();

      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess(MCP_Over_UartA.pTransportLayer,
                                                                                &MCP_Over_UartA.rxLength);
      if ( 0U == MCP_Over_UartA.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_UartA.pTransportLayer->fGetBuffer(MCP_Over_UartA.pTransportLayer,
                                                     (void **) &MCP_Over_UartA.txBuffer, //cstat !MISRAC2012-Rule-11.3
                                                     MCTL_SYNC))
        {
          /* No buffer available to build the answer ... should not occur */
        }
        else
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket(MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer,
                                                      MCP_Over_UartA.txLength, MCTL_SYNC);
          /* No buffer available to build the answer ... should not occur */
        }
      }

      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0U)
    {
      hBootCapDelayCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
    if(hStopPermanencyCounterM1 > 0U)
    {
      hStopPermanencyCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  (void)BADC_CalcAvrgMecSpeedUnit(&Bemf_ADC_M1, &wAux);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
              RUC_UpdatePulse(&RevUpControlM1, &BusVoltageSensor_M1._Super);
              ThreePwm_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              ThreePwm_SwitchOffPWM(pwmcHandle[M1]);
              SixStepVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
              BADC_Clear(&Bemf_ADC_M1);
              SixStep_Clear( M1 );
              BADC_SetDirection(&Bemf_ADC_M1, MCI_GetImposedMotorDirection( &Mci[M1]));
              BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1._Super, pSTC[M1] );

                Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            bool ObserverConverged;

            /* Execute the Rev Up procedure */
            if(! RUC_Exec(&RevUpControlM1))
            {
            /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
            (void) BADC_CalcRevUpDemagTime (&Bemf_ADC_M1);
            PWMC_ForceFastDemagTime (pwmcHandle[M1], Bemf_ADC_M1.DemagCounterThreshold);
            SixStepVars[M1].DutyCycleRef = STC_CalcSpeedReference( pSTC[M1] );
           }

            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);

            /* Check that startup stage where the observer has to be used has been reached */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
            {
             ObserverConverged = BADC_IsObserverConverged( &Bemf_ADC_M1);

              (void)VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
            }
            else
            {
              ObserverConverged = false;
            }
            if (ObserverConverged)
            {
              Mci[M1].State = SWITCH_OVER;
            }
          }
          break;
        }

        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool LoopClosed;
            int16_t hForcedMecSpeedUnit;

            if (! RUC_Exec(&RevUpControlM1))
            {
              /* The time allowed for the startup sequence has expired */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /* Compute the virtual speed and positions of the rotor.
                 The function returns true if the virtual speed is in the reliability range */
              LoopClosed = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
              /* Check if the transition ramp has completed. */
              bool tempBool;
              tempBool = VSS_TransitionEnded(&VirtualSpeedSensorM1);
              LoopClosed = LoopClosed || tempBool;

              /* If any of the above conditions is true, the loop is considered closed.
                 The state machine transitions to the RUN state */
              if (true ==  LoopClosed)
              {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
                PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
                PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                    (((int32_t)SixStepVars[M1].DutyCycleRef * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                    / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
                /* USER CODE BEGIN MediumFrequencyTask M1 1 */

                /* USER CODE END MediumFrequencyTask M1 1 */
                STC_SetSpeedSensor(pSTC[M1], &Bemf_ADC_M1._Super); /* Observer has converged */
                SixStep_InitAdditionalMethods(M1);
                SixStep_CalcSpeedRef(M1);
                BADC_SetLoopClosed(&Bemf_ADC_M1);
                BADC_SpeedMeasureOn(&Bemf_ADC_M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
                MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
                Mci[M1].State = RUN;
              }
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);

            SixStep_CalcSpeedRef( M1 );
            BADC_SetSamplingPoint(&Bemf_ADC_M1, &PWM_Handle_M1._Super, pSTC[M1] );
            (void) BADC_CalcRunDemagTime (&Bemf_ADC_M1);
            PWMC_ForceFastDemagTime (pwmcHandle[M1], Bemf_ADC_M1.DemagCounterThreshold);

          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
            BADC_Clear(&Bemf_ADC_M1);
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void SixStep_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN SixStep_Clear 0 */

  /* USER CODE END SixStep_Clear 0 */

  STC_Clear(pSTC[bMotor]);
  SixStepVars[bMotor].DutyCycleRef = STC_GetDutyCycleRef(pSTC[bMotor]);
  BADC_Stop( &Bemf_ADC_M1 );
  BADC_Clear( &Bemf_ADC_M1 );
  BADC_SpeedMeasureOff(&Bemf_ADC_M1);
  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN SixStep_Clear 1 */

  /* USER CODE END SixStep_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void SixStep_InitAdditionalMethods(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void SixStep_CalcSpeedRef(uint8_t bMotor)
{

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if(SixStepVars[bMotor].bDriveInput == INTERNAL)
  {
    SixStepVars[bMotor].DutyCycleRef = STC_CalcSpeedReference(pSTC[bMotor]);
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed.
  * @param  none
  * @retval bool true if time has elapsed, false otherwise.
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hBootCapDelayCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1.
  * @param  hTickCount number of ticks to be counted.
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed.
  * @param  none
  * @retval bool true if time is elapsed, false otherwise.
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (((uint16_t)0) == hStopPermanencyCounterM1)
  {
    retVal = true;
  }
  return (retVal);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t TSK_HighFrequencyTask(void)
{

 /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  uint8_t bMotorNbr = 0;
  uint16_t hSixStepReturn;
  if ((true == BADC_ClearStepUpdate(&Bemf_ADC_M1)) || (false == ThreePwm_IsFastDemagUpdated(&PWM_Handle_M1)))
  {
    if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State )) /*  only for sensor-less*/
    {
      if (START == Mci[M1].State)
      {
        if (0U == RUC_IsAlignStageNow(&RevUpControlM1))
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
        }
        else
        {
          PWMC_SetAlignFlag(&PWM_Handle_M1._Super, RUC_GetDirection(&RevUpControlM1));
        }
      }
      else
      {
        PWMC_SetAlignFlag(&PWM_Handle_M1._Super, 0);
      }
      int16_t hObsAngle = SPD_GetElAngle(&VirtualSpeedSensorM1._Super);
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    (void)BADC_CalcElAngle (&Bemf_ADC_M1);
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
      hSixStepReturn = SixStep_StatorController();
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
    if(MC_DURATION == hSixStepReturn)
    {
      MCI_FaultProcessing(&Mci[bMotorNbr], MC_DURATION, 0);
    }
    else
    {
      /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

      /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
    }
    ThreePwm_UpdatePwmDemagCounter( &PWM_Handle_M1 );
  }
  else
  {
    ThreePwm_DisablePwmDemagCounter( &PWM_Handle_M1 );
  }
  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */
  GLOBAL_TIMESTAMP++;
  if (0U == MCPA_UART_A.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_A);
  }
  return (bMotorNbr);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

inline uint16_t SixStep_StatorController(void)
{
  uint16_t hCodeError = MC_NO_ERROR;
  int16_t hElAngle, hSpeed, hDirection;
  SpeednPosFdbk_Handle_t *speedHandle;
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  if(false == BADC_IsObserverConverged(&Bemf_ADC_M1))
  {
    hElAngle = SPD_GetElAngle(speedHandle);
  }
  else
  {
    hElAngle = SPD_GetElAngle(&Bemf_ADC_M1._Super);
  }
  hSpeed = SPD_GetElSpeedDpp(speedHandle);
  hDirection = RUC_GetDirection(&RevUpControlM1);
  PWMC_SetPhaseVoltage( pwmcHandle[M1], SixStepVars[M1].DutyCycleRef );
  if (hDirection > 0)
  {
    SixStepVars[M1].qElAngle = hElAngle + S16_90_PHASE_SHIFT;
  }
  else
  {
    SixStepVars[M1].qElAngle = hElAngle - S16_90_PHASE_SHIFT;
  }
  PWM_Handle_M1._Super.hElAngle = SixStepVars[M1].qElAngle;
  ThreePwm_LoadNextStep( &PWM_Handle_M1, hDirection );
  if (true == ThreePwm_ApplyNextStep(&PWM_Handle_M1))
  {
    if (false == Bemf_ADC_M1.IsLoopClosed)
    {
      BADC_StepChangeEvent(&Bemf_ADC_M1, hSpeed, &PWM_Handle_M1._Super);
    }
  }
  BADC_Start(&Bemf_ADC_M1, PWM_Handle_M1._Super.Step );
  return(hCodeError);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (1U == bMCBootCompleted)
  {
    TSK_SafetyTask_PWMOFF(M1);
    /* User conversion execution */
    RCM_ExecUserConv();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_OFF_PWM.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink.
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  const uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  /* Check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[bMotor]);     /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                    (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Process faults */

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
    if (MCPA_UART_A.Mark != 0U)
    {
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    else
    {
      /* Nothing to do */
    }
    SixStep_Clear(bMotor);
    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  else
  {
    /* No errors */
  }
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  ThreePwm_SwitchOffPWM(pwmcHandle[M1]);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}

__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  GPIO_PinState state;
  if (IDLE == MC_GetSTMStateMotor1())
  {
    /* Ramp parameters should be tuned for the actual motor */
    (void)MC_StartMotor1();
    state = GPIO_PIN_SET;
  }
  else
  {
    (void)MC_StopMotor1();
    state = GPIO_PIN_RESET;
  }
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, state);
/* USER CODE END START_STOP_BTN */
}

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration.
  */
__weak void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_DP_GPIO_Port, M1_DP_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_PWM_EN_V_GPIO_Port, M1_PWM_EN_V_Pin);
LL_GPIO_LockPin(M1_PWM_EN_U_GPIO_Port, M1_PWM_EN_U_Pin);
LL_GPIO_LockPin(M1_PWM_EN_W_GPIO_Port, M1_PWM_EN_W_Pin);
LL_GPIO_LockPin(M1_BEMF_DIVIDER_GPIO_Port, M1_BEMF_DIVIDER_Pin);
LL_GPIO_LockPin(M1_BEMF_V_GPIO_Port, M1_BEMF_V_Pin);
LL_GPIO_LockPin(M1_BEMF_W_GPIO_Port, M1_BEMF_W_Pin);
LL_GPIO_LockPin(M1_BEMF_U_GPIO_Port, M1_BEMF_U_Pin);
LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
}
/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
