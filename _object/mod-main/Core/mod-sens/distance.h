/**
  ******************************************************************************
  * @file           : distance.c
  * @brief          : Distance sensor driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Krzysztof Siejka.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISTANCE_H
#define __DISTANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

uint32_t IC_Value1;
uint32_t IC_Value2;
uint32_t Difference;
uint16_t Distance;
volatile uint16_t distance_tab[5];

void RightDS_task( void );

void LeftDS_task( void );

#ifdef __cplusplus
}
#endif

#endif /* __DISTANCE_H */

/*****************************END OF FILE**************************************/

