/**
******************************************************************************
* @file    ics_f4xx_pwm_curr_fdbk.h
* @author  Motor Control SDK Team, ST Microelectronics
* @brief   This file contains all definitions and functions prototypes for the
*          ICS PWM current feedback component for F4XX of the Motor Control SDK.
******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
******************************************************************************
  * @ingroup ICS_F4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ICS_F4XX_PWMNCURRFDBK_H
#define __ICS_F4XX_PWMNCURRFDBK_H

#include "pwm_curr_fdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */



/** @addtogroup ConvFOC
  * @{
  */

/** @addtogroup MCLLAPI
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup ICS_pwm_curr_fdbk
  * @{
  */

#define SOFOC 0x0008u /* This flag is reset at the beginning of FOC
                          and it is set in the TIM UP IRQ. If at the end of
                          FOC this flag is set, it means that FOC rate is too 
                          high and thus an error is generated */



#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)


/*
  * @brief  Current feedback component parameters structure definition for ICS configuration.
  */
typedef struct
{
  /* HW IPs --------------------------------------------------------------------- */
  ADC_TypeDef * ADCx_1;                  /* First ADC peripheral to be used. */
  ADC_TypeDef * ADCx_2;                  /* Second ADC peripheral to be used. */
  TIM_TypeDef * TIMx;                    /* Timer used for PWM generation. */
  
  /* PWM generation parameters -------------------------------------------------- */
  uint8_t  RepetitionCounter;            /* Expresses the number of PWM
                                             periods to be elapsed before compare
                                             registers are updated again. In
                                             particular:
                                             RepetitionCounter = (2 * PWM Period) - 1 */

  /* Currents sampling parameters ----------------------------------------------- */
  uint8_t IaChannel;                     /* ADC channel used for conversion of
                                             current Ia. It must be equal to
                                             ADC_CHANNEL_x x= 0, ..., 15*/
  uint8_t IbChannel;                     /* ADC channel used for conversion of
                                             current Ib. It must be equal to
                                             ADC_CHANNEL_x x= 0, ..., 15*/

  /* Dual MC parameters -------------------------------------------------------- */
  uint8_t  FreqRatio;                    /* Used in case of dual MC to
                                             synchronize TIM1 and TIM8. It has
                                             effect only on the second instanced
                                             object and must be equal to the
                                             ratio between the two PWM frequencies
                                             (higher/lower). Supported values are
                                             1, 2 or 3. */
  uint16_t Tw;                          /*!< Used for switching the context
                                              in dual MC. It contains the biggest delay
                                              (expressed in counter ticks) between
                                              the counter crest and ADC latest
                                              trigger. Specific to F4XX. */
  uint8_t  IsHigherFreqTim;              /* When FreqRatio is greater than 1
                                             this param is used to indicate if this
                                             instance is the one with the highest
                                             frequency. Allowed values are: HIGHER_FREQ
                                             or LOWER_FREQ. */

} ICS_Params_t;



/*
  * @brief  PWM and Current Feedback ICS handle.
  */
typedef struct
{
  PWMC_Handle_t _Super;                 /* Base component handler. */
  uint32_t PhaseAOffset;                /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;                /* Offset of Phase B current sensing network. */
  uint32_t ADC_ExternalTriggerInjected; /*!< Stores the value for ADC CR2 to properly configure
                                          current sampling during the context switching. Specific to F4XX and F7XX.*/

  uint16_t Half_PWMPeriod;              /* Half PWM Period in timer clock counts. */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase. */
  volatile uint8_t PolarizationOnGoing; /* status of the polarization execution. */
 
  ICS_Params_t const * pParams_str;
} PWMC_ICS_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO and NVIC for current reading
  * with ICS configurations.
  */
void ICS_Init( PWMC_ICS_Handle_t * pHandle );

/*
  * Sums up injected conversion data into wPhaseXOffset.
  */
void ICS_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void ICS_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * pStator_Currents );

/*
  * Turns on low sides switches.
  */
void ICS_TurnOnLowSides( PWMC_Handle_t * pHandle, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void ICS_SwitchOnPWM( PWMC_Handle_t * pHandle );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void ICS_SwitchOffPWM( PWMC_Handle_t * pHandle );

/*
  * Writes into peripheral registers the new duty cycle and sampling point.
  */
uint16_t ICS_WriteTIMRegisters( PWMC_Handle_t * pHandle );

/*
  * Contains the TIMx Update event interrupt.
  */
void * ICS_TIMx_UP_IRQHandler( PWMC_ICS_Handle_t * pHandle );

/*
  * Contains the TIMx Break1 event interrupt.
  */
void * ICS_BRK_IRQHandler( PWMC_ICS_Handle_t * pHdl );

/*
  * Stores in the handler the calibrated offsets.
  */
void ICS_SetOffsetCalib( PWMC_Handle_t * pHdl, PolarizationOffsets_t * offsets );

/*
  * Reads the calibrated offsets stored in the handler.
  */
void ICS_GetOffsetCalib( PWMC_Handle_t * pHdl, PolarizationOffsets_t * offsets );

/*
  * Sets the PWM mode for R/L detection.
  */
void ICS_RLDetectionModeEnable( PWMC_Handle_t * pHdl );

/*
  * Disables the PWM mode for R/L detection.
  */
void ICS_RLDetectionModeDisable( PWMC_Handle_t * pHdl );

/*
  * Sets the PWM dutycycle for R/L detection.
  */
uint16_t ICS_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty );

/*
  * Turns on low sides switches and start ADC triggering.
  */
void ICS_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl );

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

#endif /*__ICS_F4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
