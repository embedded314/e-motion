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

#define UART_MM_DEBUG

enum {
	FREE = 0,
	FWD,
	BACK,
	TURN
} MM_drive_dire;

struct {
	short int	state;
	int			RS_distance;
	int			LS_distance;
	long		total_distance;
	signed long	logic_distance;
} MM_drive;

#define MM_DIST_ARR_SIZE	5
int MM_L_dist_arr[MM_DIST_ARR_SIZE];
int MM_R_dist_arr[MM_DIST_ARR_SIZE];

void MM_task_init ( void );

void MM_R_dist_t( void );

void MM_L_dist_t( void );

void clear_array(int *buffer, size_t buffer_size);

int update_array (int *buffer, size_t buffer_size, int new_value);

#ifdef UART_MM_DEBUG
void UART_DS_debug (int *val_L, int *val_R);
#endif


#ifdef __cplusplus
}
#endif

#endif /* __DISTANCE_H */

/*****************************END OF FILE**************************************/

