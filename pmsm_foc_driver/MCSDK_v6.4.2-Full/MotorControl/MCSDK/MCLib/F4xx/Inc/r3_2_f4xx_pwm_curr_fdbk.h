/**
  ******************************************************************************
  * @file    r3_2_f4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_2_F4XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R3_2_F4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_2_PWM_CURR_FDBK_H
#define __R3_2_PWM_CURR_FDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_2_pwm_curr_fdbk
  * @{
  */

/*
  * @brief  PWM and current feedback component parameters definition for dual ADCs configurations.
  */
typedef const struct
{
  /* HW IP involved ------------------------------------------------------------- */
  TIM_TypeDef * TIMx;                       /* Timer used for PWM generation. Must be equal to TIM1 if M1, to TIM8 otherwise */
  uint32_t ADCConfig1[6];                   /* First ADC's channels sequence across the 6 sectors */
  uint32_t ADCConfig2[6];                   /* Second ADC's channels sequence across the 6 sectors */

  /* Currents sampling parameters ----------------------------------------------- */
  uint16_t Tafter;                          /* Sum of dead time plus max value between rise time and noise time expressed in number of TIM clocks */
  uint16_t Tbefore;                         /* Total time of the sampling sequence expressed in number of TIM clocks */
  uint16_t Tcase2;                          /* Sum of dead time, noise time and sampling time divided by 2 ; expressed in number of TIM clocks */
  uint16_t Tcase3;                          /* Sum of dead time, rise time and sampling time ; expressed in number of TIM clocks */
  ADC_TypeDef * ADCDataReg1[6];             /* Contains the Address of ADC read value for one phase and all the 6 sectors */ 
  ADC_TypeDef * ADCDataReg2[6];             /* Contains the Address of ADC read value for one phase and all the 6 sectors */ 

  /* PWM Driving signals initialization ----------------------------------------- */
  uint8_t  RepetitionCounter;               /* Number of elapsed PWM periods before Compare Registers are updated again.
                                                In particular : RepetitionCounter = (2 * PWM periods) - 1 */

  /* Dual MC parameters --------------------------------------------------------- */
  uint8_t  FreqRatio;                       /* Used in case of dual MC to
                                                synchronize TIM1 and TIM8. It has
                                                effect only on the second instanced
                                                object and must be equal to the
                                                ratio between the two PWM frequencies
                                                (higher/lower). Supported values are
                                                1, 2 or 3 */
  uint16_t Tw;                            /*!< Used for switching the context
                                                in dual MC. It contains biggest delay
                                                (expressed in counter ticks) between
                                                the counter crest and ADC latest trigger. Specific to F4XX, F7XX and H5XX. */
  uint8_t  IsHigherFreqTim;                 /* When FreqRatio is greater than 1
                                                this param is used to indicate if this
                                                instance is the one with the highest
                                                frequency. Allowed values are: HIGHER_FREQ
                                                or LOWER_FREQ */ 

} R3_2_Params_t;

/*
  * @brief  PWM and current feedback component for dual ADCs configurations.
  */
typedef struct
{
  PWMC_Handle_t _Super;                 /* Base component handler */
  uint32_t PhaseAOffset;                /* Offset of Phase A current sensing network */
  uint32_t PhaseBOffset;                /* Offset of Phase B current sensing network */
  uint32_t PhaseCOffset;                /* Offset of Phase C current sensing network */
  uint16_t Half_PWMPeriod;              /* Half PWM Period in timer clock counts */
  uint32_t ADC_ExternalTriggerInjected; /* External trigger selection for ADC peripheral */
  uint32_t ADCTriggerEdge;              /* Polarity of the ADC triggering, can be either on rising or falling edge */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase */
  uint8_t PolarizationSector;           /* Space vector sector number during calibration */
  volatile uint8_t PolarizationOnGoing; /* status of the polarization execution. */
  R3_2_Params_t const *pParams_str;
} PWMC_R3_2_Handle_t;


/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in three shunt topology using STM32F4XX and two ADCs.
  */
void R3_2_Init( PWMC_R3_2_Handle_t * pHandle );

/*
  * Stores into the handler the voltage present on Ia and Ib current 
  * feedback analog channels when no current is flowing into the motor.
  */
void R3_2_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R3_2_GetPhaseCurrents( PWMC_Handle_t * pHandle,ab_t* Iab);

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format. Specific to overmodulation.
  */
void R3_2_GetPhaseCurrents_OVM( PWMC_Handle_t * pHandle,ab_t* Iab);

/*
  * Turns on low side switches.
  */
void R3_2_TurnOnLowSides( PWMC_Handle_t * pHandle, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOnPWM( PWMC_Handle_t * pHandle );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* 
  * Configures the ADC for the current sampling during calibration.
  */
uint16_t R3_2_SetADCSampPointCalibration( PWMC_Handle_t * pHandle );

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_2_SetADCSampPointSectX( PWMC_Handle_t * pHandle );

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ) in case of overmodulation.
  */
uint16_t R3_2_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHandle );

/*
  *  Contains the TIMx Update event interrupt.
  */
void *R3_2_TIMx_UP_IRQHandler( PWMC_R3_2_Handle_t * pHandle);

/*
  *  Contains the TIMx Break event interrupt.
  */
void *R3_2_BRK_IRQHandler(PWMC_R3_2_Handle_t *pHdl);

/*
  * Sets the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeEnable( PWMC_Handle_t * pHandle  );

/*
  * Disables the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeDisable( PWMC_Handle_t * pHandle  );

/*
  * Sets the PWM dutycycle for R/L detection.
  */
uint16_t R3_2_RLDetectionModeSetDuty( PWMC_Handle_t * pHandle , uint16_t hDuty );

/*
 *  Turns on low sides switches and start ADC triggering.
 */
void R3_2_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHandle  );

/*
  * Stores in the handler the calibrated offsets.
  */
void R3_2_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets stored in the handler.
  */
void R3_2_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

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

#endif /*__R3_2_PWM_CURR_FDBK_H*/

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
