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

#include "main.h"
#include <cmsis_os.h>
#include <stdio.h>
#include <string.h>
#include "distance.h"

#define WD_DELAY	30000

const float speed_of_sound = 0.343;

long IC_Value1;
long IC_Value2;
long difference;
int	 distance;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart2;

osSemaphoreId MM_bin_semHandle;

/**
  * @brief Semaphore initialization.
  * @param None
  * @retval None
  */
void MM_task_init ( void ) {
	osSemaphoreDef(MM_bin_sem);
	MM_bin_semHandle = osSemaphoreCreate(osSemaphore(MM_bin_sem), 1);

	clear_array (MM_R_dist_arr, sizeof (MM_R_dist_arr)/sizeof (MM_R_dist_arr[0]));
	clear_array (MM_L_dist_arr, sizeof (MM_L_dist_arr)/sizeof (MM_L_dist_arr[0]));
}
/**
  * @brief Right distance sensor readings task.
  * @param None
  * @retval None
  */
void MM_R_dist_t() {
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

	/* Infinite loop */
	for (;;) {
		osSemaphoreWait(MM_bin_semHandle, osWaitForever);

		/*	Set pin R trigger to high state for no less then 10us. */
		HAL_GPIO_WritePin(MM_R_dist_trig_GPIO_Port, MM_R_dist_trig_Pin, GPIO_PIN_SET);
		usDelay(7);
		/*	Set pin R trigger back to low state.*/
		HAL_GPIO_WritePin(MM_R_dist_trig_GPIO_Port, MM_R_dist_trig_Pin, GPIO_PIN_RESET);

		/*	Reset capture flag.	*/
		TIM2->SR = 0;


		int WD_cnt = 0;
		/*	Wait for the first echo pin, rising edge. */
		while  (!(TIM2->SR & TIM_SR_CC1IF) && (WD_cnt < WD_DELAY)) {
			WD_cnt++;
		}

		/*	Read first timer data.	*/
		IC_Value1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		usDelay(1);

		WD_cnt = 0;
		/*	Wait for the second echo pin, falling edge. */
		while (!(TIM2->SR & TIM_SR_CC1IF) && (WD_cnt < WD_DELAY)) {
			WD_cnt++;
		}

		/*	Read second timer data.	*/
		IC_Value2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		/*	Calculate the difference between first and second reading.	*/
		if (IC_Value2 > IC_Value1) difference = IC_Value2 - IC_Value1;

		/*	Distance calculation.	*/
		distance = (difference / 2) * speed_of_sound;

		MM_drive.RS_distance = update_array (MM_R_dist_arr, sizeof (MM_R_dist_arr)/sizeof (MM_R_dist_arr [0]), distance);

		osSemaphoreRelease(MM_bin_semHandle);
		osDelay(100);
	}
}
/**
  * @brief Left side distance sensor readings task.
  * @param None
  * @retval None
  */
void MM_L_dist_t( void ){
	HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_2);

	/* Infinite loop */
	for (;;) {
		osSemaphoreWait(MM_bin_semHandle, osWaitForever);

		/*	Set pin L trigger to high state for no less then 10us. */
		HAL_GPIO_WritePin(MM_L_dist_trig_GPIO_Port, MM_L_dist_trig_Pin, GPIO_PIN_SET);
		usDelay(7);
		/*	Set pin L trigger back to low state.*/
		HAL_GPIO_WritePin(MM_L_dist_trig_GPIO_Port, MM_L_dist_trig_Pin, GPIO_PIN_RESET);

		/*	Reset capture flag.	*/
		TIM5->SR = 0;

		int WD_cnt = 0;
		/*	Wait for the first echo pin, rising edge. */
		while  (!(TIM5->SR & TIM_SR_CC2IF) && (WD_cnt < WD_DELAY)) {
			WD_cnt++;
		}

		/*	Read first timer data.	*/
		IC_Value1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);
		usDelay(1);

		WD_cnt = 0;
		/*	Wait for the second echo pin, falling edge. */
		while (!(TIM5->SR & TIM_SR_CC2IF) && (WD_cnt < WD_DELAY)) {
			WD_cnt++;
		}

		/*	Read second timer data.	*/
		IC_Value2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);

		/*	Calculate the difference between first and second reading.	*/
		if (IC_Value2 > IC_Value1) difference = IC_Value2 - IC_Value1;

		/*	Distance calculation.	*/
		distance = (difference / 2) * speed_of_sound;

		MM_drive.LS_distance = update_array (MM_L_dist_arr, sizeof (MM_L_dist_arr)/sizeof (MM_L_dist_arr [0]), distance);

		osSemaphoreRelease(MM_bin_semHandle);
		osDelay(100);
	}
}
/**
  * @brief Reset array to default.
  * @param *buffer 		: pointer to the array
  * @param buffer_size 	: array size
  * @retval 			: None
  */
void clear_array(int *buffer, size_t buffer_size){
	for (int i=0; i < buffer_size; i++) {
		*(buffer + i) = 0;
	}
}
/**
  * @brief Reset array to default.
  * @param *buffer 		: pointer to the array
  * @param buffer_size 	: array size
  * @param new_value 	: value to be added as the first element of the array
  * @retval 			: median value
  */
int update_array (int *buffer, size_t buffer_size, int new_value) {
	/*	Move all array row up.	*/
	for (int i = buffer_size-1; i > 0; i--) {
		*(buffer + i) = *(buffer + i - 1);
	}
	/*	Update first row with new value. */
	*(buffer) = new_value;

	int buff_arr[buffer_size];
	/*	Copy current array to the buffer. */
	for (int i=0;i<buffer_size;i++) {
		buff_arr [i] = *(buffer + i);
	}

	/*	Sort buffer array in ascending order. */
	int n = buffer_size;
	int c, d;
	int swap;
	for (c = 0; c < (n - 1); c++) {
		for (d = 0; d < n - c - 1; d++) {
			if (buff_arr[d] > buff_arr[d + 1]) /* For decreasing order use < */
			{
				swap = buff_arr[d];
				buff_arr[d] = buff_arr[d + 1];
				buff_arr[d + 1] = swap;
			}
		}
	}

	int median=0;

	/*	if number of elements are even	*/
	if(n%2 == 0)
		median = (buff_arr[(buffer_size-1)/2] + buff_arr[buffer_size/2])/2;
	/*	if number of elements are odd	*/
	else
		median = buff_arr[buffer_size/2];

	return median;
}
void UART_DS_debug (int *val_L, int *val_R) {
	uart_buff_len = sprintf(uart_buff, "LS: %1u;  RS: %1u;\r\n", val_L, val_R);

	HAL_UART_Transmit(&huart2, uart_buff, uart_buff_len, 100);
}
/*****************************END OF FILE**************************************/
