
/**
  ******************************************************************************
  * @file    mc_math.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
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
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

/* Private macro -------------------------------------------------------------*/

#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315 */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function returns cosine and sine functions of the angle fed in input.
  * @param  hAngle: angle in q1.15 format.
  * @retval Sin(angle) and Cos(angle) in Trig_Components format.
  */

__weak Trig_Components MCM_Trig_Functions(int16_t hAngle)
{
  /* MISRAC2012-violation Rule 19.2. The union keyword should not be used.
   * If this rule is not followed, the kinds of behavior that need to be determined
   * are:
   * Padding — how much padding is inserted at the end of the union;
   * Alignment — how are members of any structures within the union aligned;
   * Endianness — is the most significant byte of a word stored at the lowest or
   *              highest memory address;
   * Bit-order — how are bits numbered within bytes and how are bits allocated to
   *             bit fields.
   * Low. Use of union (u32toi16x2). */
  //cstat -MISRAC2012-Rule-19.2
//  union u32toi16x2 {
//    uint32_t CordicRdata;
//    Trig_Components Components;
//  } CosSin;
//  //cstat +MISRAC2012-Rule-19.2
//  /* Configure CORDIC */
//  /* Misra  violation Rule 11.4 A�Conversion�should�not�be�performed�between�a�
//   * pointer�to�object and an integer type */
//  WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
//  /* Misra  violation Rule�11.4 A�Conversion�should�not�be�performed�between�a
//   * pointer�to�object and an integer type */
//  LL_CORDIC_WriteData(CORDIC, ((uint32_t)0x7FFF0000) + ((uint32_t)hAngle));
//  /* Read angle */
//  /* Misra  violation Rule�11.4 A�Conversion�should�not�be�performed between�a
//   * pointer�to object and an integer type */
//  CosSin.CordicRdata = LL_CORDIC_ReadData(CORDIC);
//  return (CosSin.Components); //cstat !UNION-type-punning
  Trig_Components temp;
  temp.hCos = temp.hSin = 0;
  return temp;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It calculates the square root of a non-negative int32_t. It returns 0 for negative int32_t.
  * @param  Input int32_t number.
  * @retval int32_t Square root of Input (0 if Input<0).
  */
__weak int32_t MCM_Sqrt(int32_t wInput)
{
//  int32_t wtemprootnew;
//
//  if (wInput > 0)
//  {
//    uint32_t retVal;
//    /* Disable Irq as sqrt is used in MF and HF task */
//    __disable_irq();
//    /* Configure CORDIC */
//    WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SQRT);
//    LL_CORDIC_WriteData(CORDIC, ((uint32_t)wInput));
//    /* Read sqrt and return */
//#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
//    retVal = (LL_CORDIC_ReadData(CORDIC)) >> 15; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
//#else
//    retVal = (LL_CORDIC_ReadData(CORDIC)) / 32768U;
//#endif
//    wtemprootnew = (int32_t)retVal;
//    __enable_irq();
//
//  }
//  else
//  {
//    wtemprootnew = (int32_t)0;
//  }
//
//  return (wtemprootnew);
  return 0;
}

/**
  * @brief  This function codify a floating point number into the relative 32bit integer.
  * @param  float Floating point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
__weak uint32_t MCM_floatToIntBit( float_t x ) //cstat !MISRAC2012-Dir-4.6_a
{
  const uint32_t *pInt;
  pInt = (uint32_t *)(&x); //cstat !MISRAC2012-Rule-11.3
  return (*pInt);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
