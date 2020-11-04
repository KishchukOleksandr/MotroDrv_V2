/**
  ******************************************************************************
  * @file    encoder_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Encoder Speed & Position Feedback component of the Motor Control SDK.
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
  * @ingroup SpeednPosFdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_SPEEDNPOSFDBK_H
#define __ENCODER_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
   * @{
   */

/** @addtogroup Encoder
 * @{
 */

/* Exported constants --------------------------------------------------------*/

#define ENC_SPEED_ARRAY_SIZE  ((uint8_t)16)    /* 2^4 */

/**
  * @brief  ENCODER class parameters definition
  */
typedef struct
{
  SpeednPosFdbk_Handle_t _Super;

  int32_t DeltaCapturesBuffer[ENC_SPEED_ARRAY_SIZE]; /*!< Buffer used to store
                                        captured variations of timer counter*/            
                                        
  /* SW Settings */
  uint32_t SpeedSamplingFreqUnit; /*!< Frequency at which motor speed is to be
                                 computed. It must be equal to the frequency
                                 at which function SPD_CalcAvrgMecSpeedUnit
                                 is called.*/
  uint16_t SpeedSamplingFreqHz;        /*! <Frequency (Hz) at which motor speed
                                        is to be computed. */   
  uint16_t MaxDeltaAngle;
  uint16_t PulseNumber; 
  uint16_t Angle; 
  uint16_t PreviousCapture;           
  uint16_t CurrentCapture;     
  uint16_t Offset;     
  uint16_t MaxRPM;
  uint8_t SpeedBufferSize;       
  uint8_t TimerOverflowNb;  
  bool SensorIsReliable;                                
  bool initDone;
  volatile uint8_t DeltaCapturesIndex; /*! <Buffer index*/

} ENCODER_Handle_t;


void * ENC_IRQHandler( void * pHandleVoid );
void ENC_Init( ENCODER_Handle_t * pHandle );
void ENC_Clear( ENCODER_Handle_t * pHandle );
int16_t ENC_CalcAngle( ENCODER_Handle_t * pHandle );
bool ENC_CalcAvrgMecSpeedUnit( ENCODER_Handle_t * pHandle, int16_t * pMecSpeedUnit );
void ENC_SetMecAngle( ENCODER_Handle_t * pHandle, int16_t hMecAngle );

/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__ENCODER_SPEEDNPOSFDBK_H*/

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
