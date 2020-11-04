/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the Encoder component of the Motor Control SDK:
  *
  *           - computes & stores average mechanical speed
  *           - computes & stores average mechanical acceleration
  *           - computes & stores  the instantaneous electrical speed
  *           - calculates the rotor electrical and mechanical angle
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "encoder_speed_pos_fdbk.h"
#include "trajectory_ctrl.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "mc_config.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup Encoder Encoder Speed & Position Feedback
  * @brief Quadrature Encoder based Speed & Position Feedback implementation
  *
  * This component is used in applications controlling a motor equipped with a quadrature encoder.
  *
  * This component uses the output of a quadrature encoder to provide a measure of the speed and
  * the position of the rotor of the motor.
  *
  * @todo Document the Encoder Speed & Position Feedback "module".
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/


/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC)
            required for the speed position sensor management using ENCODER
            sensors.
  * @param  pHandle: handler of the current instance of the encoder component
  * @retval none
  */
__weak void ENC_Init( ENCODER_Handle_t * pHandle )
{
  uint8_t BufferSize;
  uint8_t Index;

  /* Reset counter */

  pHandle->SpeedSamplingFreqUnit = pHandle->SpeedSamplingFreqHz * SPEED_UNIT;

  /* Erase speed buffer */
  BufferSize = pHandle->SpeedBufferSize;
  
  for ( Index = 0u; Index < BufferSize; Index++ )
  {
    pHandle->DeltaCapturesBuffer[Index] = 0;
  }
}

/**
* @brief  Clear software FIFO where are "pushed" rotor angle variations captured
*         This function must be called before starting the motor to initialize
*         the speed measurement process.
* @param  pHandle: handler of the current instance of the encoder component
* @retval none
*/
__weak void ENC_Clear( ENCODER_Handle_t * pHandle )
{
  uint8_t Index;
  for ( Index = 0u; Index < pHandle->SpeedBufferSize; Index++ )
  {
    pHandle->DeltaCapturesBuffer[Index] = 0;
  }
  
  pHandle->CurrentCapture = pHandle->CurrentCapture + pHandle->Offset;
  
  if(pHandle->CurrentCapture > pHandle->PulseNumber)
  {
    pHandle->CurrentCapture %= pHandle->PulseNumber;
  }
  pHandle->Offset = pHandle->CurrentCapture;
  
  pHandle->CurrentCapture = 0;
  pHandle->PreviousCapture = 0;    
  pHandle->_Super.wMecAngle = 0;
  pHandle->SensorIsReliable = true;
}

/**
* @brief  It calculates the rotor electrical and mechanical angle, on the basis
*         of the instantaneous value of the timer counter.
* @param  pHandle: handler of the current instance of the encoder component
* @retval int16_t Measured electrical angle in s16degree format.
*/
__weak int16_t ENC_CalcAngle( ENCODER_Handle_t * pHandle )
{
  static int32_t wtemp1;
  int16_t elAngle;  /* s16degree format */
  int16_t mecAngle; /* s16degree format */
  /* PR 52926 We need to keep only the 16 LSB, bit 31 could be at 1 
   if the overflow occurs just after the entry in the High frequency task */
  //read old unhandled data  

  SPI1->DR = 0xFFFF;
  wtemp1 = SPI1->DR & 0x1FFF; 

  if(pHandle->Offset <= wtemp1)
  {
    wtemp1 -= pHandle->Offset;
  }
  else
  {
    wtemp1 = (pHandle->PulseNumber + wtemp1) - pHandle->Offset;
  }
    
  wtemp1 = ( int32_t )( ( wtemp1 ) * 0xffff );

  /*Computes and stores the rotor mechanical angle*/
  mecAngle = ( int16_t )( wtemp1 / pHandle->PulseNumber );

  int16_t hMecAnglePrev = pHandle->_Super.hMecAngle;

  pHandle->_Super.hMecAngle = mecAngle;
  
  /*Computes and stores the rotor electrical angle*/
  elAngle = mecAngle * pHandle->_Super.bElToMecRatio;

  pHandle->_Super.hElAngle = elAngle;
  
  int16_t hMecSpeedDpp = mecAngle - hMecAnglePrev;
  pHandle->_Super.wMecAngle += (int32_t)(hMecSpeedDpp & 0xFFFF);
  /*Returns rotor electrical angle*/
  return ( elAngle );
}

/**
  * @brief  This method must be called with the periodicity defined by parameter
  *         hSpeedSamplingFreqUnit. The method generates a capture event on
  *         a channel, computes & stores average mechanical speed [expressed in the unit
  *         defined by #SPEED_UNIT] (on the basis of the buffer filled by CCx IRQ),
  *         computes & stores average mechanical acceleration [#SPEED_UNIT/SpeedSamplingFreq],
  *         computes & stores the instantaneous electrical speed [dpp], updates the index of the
  *         speed buffer, then checks & stores & returns the reliability state
  *         of the sensor.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  pMecSpeedUnit pointer used to return the rotor average mechanical speed
  *         (expressed in the unit defined by #SPEED_UNIT)
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */


__weak bool ENC_CalcAvrgMecSpeedUnit( ENCODER_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{  
  static bool isSpeedReliable = false;
  static uint8_t dumyCupturesNb = 10;
  static uint16_t absDeltaAngle;
  static int32_t wOverallAngleVariation = 0;
  static int32_t wtemp1;
  static int32_t wtemp2;
  static uint8_t bBufferIndex = 0u;
  static uint32_t angleBuf;
 
//read old unhandled data  
  if((SPI1->SR & SPI_SR_RXNE_Msk) == SPI_SR_RXNE_Msk)  
  {
    pHandle->CurrentCapture = SPI1->DR & 0x1FFF;
  }
//read current angle  
  angleBuf = 0;
  for(uint8_t i = 0; i < 6; i++)
  {
    while((SPI1->SR & SPI_SR_RXNE_Msk) != SPI_SR_RXNE_Msk)
    {
      SPI1->DR = 0xFFFF;
      while((SPI1->SR & SPI_SR_BSY_Msk) != 0){};
    }
    
    angleBuf += SPI1->DR & 0x1FFF;
  }
 
  angleBuf /= 6;
  pHandle->CurrentCapture = pHandle->Angle = (uint16_t)angleBuf;
  
  if(dumyCupturesNb > 0)
  {        
    pHandle->MaxDeltaAngle   = (pHandle->MaxRPM * 2 * pHandle->PulseNumber)/(_RPM * 1000),
    pHandle->Offset = pHandle->CurrentCapture;
    pHandle->PreviousCapture = pHandle->CurrentCapture = 0;
    dumyCupturesNb--;
    pHandle->initDone = true;
  }
  else
  {//calc an angle through the offset
    if(pHandle->Offset <= pHandle->CurrentCapture)
    {
      pHandle->CurrentCapture -= pHandle->Offset;
    }
    else
    {
      pHandle->CurrentCapture = (pHandle->PulseNumber + pHandle->CurrentCapture) - pHandle->Offset;
    }
    
     if(pHandle->CurrentCapture == 0)
     {
       TC_EncoderReset(&pPosCtrlM1); 
     }
    //calc absolute delta angle    
    if(pHandle->CurrentCapture >= pHandle->PreviousCapture)
    {
      absDeltaAngle = pHandle->CurrentCapture - pHandle->PreviousCapture;
    }
    else
    {
      absDeltaAngle = pHandle->PreviousCapture - pHandle->CurrentCapture;
    }
    
    if(absDeltaAngle > pHandle->MaxDeltaAngle)
    {//overflow
      if(pHandle->CurrentCapture >= pHandle->PreviousCapture)
      {//dir DOWN
        pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] = absDeltaAngle - pHandle->PulseNumber;
      }
      else
      {//dir UP
        pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] = pHandle->PulseNumber - absDeltaAngle;
      }
    }
    else
    {
      pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] = pHandle->CurrentCapture - pHandle->PreviousCapture;
    }      

    pHandle->PreviousCapture = pHandle->CurrentCapture;
   /*Computes & stores the instantaneous electrical speed [dpp], var wtemp1*/
    wtemp1 = pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] *
             ( int32_t )( pHandle->SpeedSamplingFreqHz ) *
             ( int32_t )pHandle->_Super.bElToMecRatio;
    wtemp1 /= ( int32_t )( pHandle->PulseNumber );
    wtemp1 *= ( int32_t )( pHandle->_Super.DPPConvFactor);
    wtemp1 /= ( int32_t )( pHandle->_Super.hMeasurementFrequency );

    pHandle->_Super.hElSpeedDpp = ( int16_t )wtemp1;
    
    pHandle->DeltaCapturesIndex++;
    
    if ( pHandle->DeltaCapturesIndex >= pHandle->SpeedBufferSize )
    {
      pHandle->DeltaCapturesIndex = 0u;
      isSpeedReliable = true;      
    }
    
    if(isSpeedReliable)
    {   
      wOverallAngleVariation = 0;
      /*Computes & returns average mechanical speed */
      for ( bBufferIndex = 0u; bBufferIndex < pHandle->SpeedBufferSize; bBufferIndex++ )
      {
        wOverallAngleVariation += pHandle->DeltaCapturesBuffer[bBufferIndex];
      }
      wtemp1 = wOverallAngleVariation * ( int32_t )( pHandle->SpeedSamplingFreqUnit );
      wtemp2 = ( int32_t )( pHandle->PulseNumber ) *
               ( int32_t )( pHandle->SpeedBufferSize );
      wtemp1 /= wtemp2;
      /*Computes & stores average mechanical acceleration */
      pHandle->_Super.hMecAccelUnitP = ( int16_t )( wtemp1 - pHandle->_Super.hAvrMecSpeedUnit );

      /*Stores average mechanical speed */
      pHandle->_Super.hAvrMecSpeedUnit = ( int16_t )wtemp1;    
    }   
  }  
  
  *pMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
  
  return SPD_IsMecSpeedReliable( &pHandle->_Super, pMecSpeedUnit );   
}

/**
  * @brief  It set instantaneous rotor mechanical angle.
  *         As a consequence, timer counted is computed and updated.
  * @param  pHandle: handler of the current instance of the encoder component
  * @param  hMecAngle new value of rotor mechanical angle (s16degrees)
  * @retval none
  */
__weak void ENC_SetMecAngle( ENCODER_Handle_t * pHandle, int16_t hMecAngle )
{
  uint16_t hAngleCounts;
  uint16_t hMecAngleuint;

  
  pHandle->_Super.hMecAngle = hMecAngle;
  pHandle->_Super.hElAngle = hMecAngle * pHandle->_Super.bElToMecRatio;
  if ( hMecAngle < 0 )
  {
    hMecAngle *= -1;
    hMecAngleuint = 65535u - ( uint16_t )hMecAngle;
  }
  else
  {
    hMecAngleuint = ( uint16_t )hMecAngle;
  }

  hAngleCounts = ( uint16_t )( ( ( uint32_t )hMecAngleuint *
                                 ( uint32_t )pHandle->PulseNumber ) / 65535u );

  
  pHandle->CurrentCapture = pHandle->CurrentCapture + pHandle->Offset;
  
  if(pHandle->CurrentCapture > pHandle->PulseNumber)
  {
    pHandle->CurrentCapture %= pHandle->PulseNumber;
  }
  
  if(pHandle->CurrentCapture > hAngleCounts)
  {
    pHandle->Offset = pHandle->CurrentCapture - hAngleCounts;
  }
  else
  {
    pHandle->Offset = (pHandle->PulseNumber + pHandle->CurrentCapture) - hAngleCounts;
  }
  pHandle->CurrentCapture = hAngleCounts;
  pHandle->PreviousCapture = hAngleCounts;  
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */


/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
