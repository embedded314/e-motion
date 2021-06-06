/**
 * \file
 *
 * \brief Main procedure.
 *
*/
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <atmel_start.h>
#include <include/timer_0.h>
#include <include/motor_driver.h>
#include <include/motion_driver.h>
#include "main.h"


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	cli();
	
	WDT_initialization();
	
	PCIE_2_initialization();
	
	HC05_initialization();
	
	sei();

	/*	Set initial rotary encoder values */
	speed_encoder_reg_update();
	
	/*	Initializes shift register (74HC595) pinout */
	motor_sh_reg_init();

	/*	Motion command test. */
		mm_rotate(-120, 60);
		mm_rotate(360, 30);
		_delay_ms(2000);
		mm_run (10, 50, 1000);
		
		mm_run (10, -50, 500);
		mm_rotate(90, 60);
		mm_run (10, -50, 250);
		mm_rotate(-120, 80);
		mm_run (10, 10, 1100);
		motor_stop();
		mm_run (10, -50, 500);
		mm_rotate(180, 30);
		mm_run (10, -50, 1500);
		mm_rotate(-120, 60);
		mm_run (10, 10, 2000);
		motor_stop();
	while (1) {
		mm_trip_distance_calc();
	}
}

uint8_t is_bit_set(uint8_t b, uint8_t pos)
{
	return ((b >> pos) & 1) != 0;
}
uint32_t map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
