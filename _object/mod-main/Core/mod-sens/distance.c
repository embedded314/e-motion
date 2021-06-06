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

#include <main.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "distance.h"

TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/**
  * @brief Right distance sensor readings task.
  * @param None
  * @retval None
  */
void RightDS_task() {
	if (distance_tab[0] != 0) {
		for (uint8_t i = 0; i < 5; i++) {
			distance_tab[i] = 0;
		}
	}
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

	/* Infinite loop */
	for (;;) {
		/*	Set pin R trigger to high state for 10us. */
		HAL_GPIO_WritePin(GPIOA, Trig_R_DS_Pin, GPIO_PIN_SET);
		usDelay(7);
		/*	Set pin R trigger back to low state.*/
		HAL_GPIO_WritePin(GPIOA, Trig_R_DS_Pin, GPIO_PIN_RESET);

		/*	Wait for the first echo pin, rising edge. */
		while (!(TIM2->SR & TIM_SR_CC1IF)) {
			//	TO DO: infinite while loop protection
		}
		IC_Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // capture first timer value
		usDelay(1);

		/*	Wait for the second echo pin, falling edge. */
		while (!(TIM2->SR & TIM_SR_CC1IF)) {
			//	TO DO: infinite while loop protection
		};
		IC_Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // capture second timer value

		if (IC_Value2 > IC_Value1) {
			Difference = IC_Value2 - IC_Value1;  // calculate the difference
		} else if (IC_Value2 < IC_Value1) {
			Difference = ((0xffff - IC_Value1) + IC_Value2) + 1;
		}
		//Frequency = HAL_RCC_GetPCLK1Freq()/Difference;  // calculate frequency
		Distance = (Difference / 2) * 0.343;	//	calculate distance

		TIM2->CNT = 0x00;
		TIM2->CCMR1 = 0x00;

		if (distance_tab[0] == 0) {
			for (uint8_t i = 0; i < 5; i++) {
				distance_tab[i] = Distance;
			}
		}

		/*	move value from [i] to [i-1] */
		for (uint8_t i = 4; i > 0; i--) {
			distance_tab[i] = distance_tab[i - 1];
		}
		/*	Update distance array with new value. */
		distance_tab[0] = Distance;

		volatile uint16_t buff_tab[5];

		/*	Copy current distance array to the buffer. */
		for (uint8_t i = 0; i < 5; i++) {
			buff_tab[i] = distance_tab[i];
		}

		/*	Sort buffer array in ascending order. */
		uint8_t n = 5;
		uint8_t c, d;
		uint16_t swap;
		for (c = 0; c < (n - 1); c++) {
			for (d = 0; d < n - c - 1; d++) {
				if (buff_tab[d] > buff_tab[d + 1]) /* For decreasing order use < */
				{
					swap = buff_tab[d];
					buff_tab[d] = buff_tab[d + 1];
					buff_tab[d + 1] = swap;
				}
			}
		}

		//	qsort (buff_tab, 5, sizeof(uint16_t), compare_function);

		uart_buff_len = sprintf(uart_buff, "%1u;\r\n", buff_tab[2]);
		HAL_UART_Transmit(&huart2, uart_buff, uart_buff_len, 100);

		osDelay(500);
	}
}
/**
  * @brief Left side distance sensor readings task.
  * @param None
  * @retval None
  */
void LeftDS_task( void ){
}
/*****************************END OF FILE**************************************/
