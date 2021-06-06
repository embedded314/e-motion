/**
 * \file
 *
 * \brief Motion driver.
 *
*/
#include <stdio.h>
#include <util/delay.h>
#include <atmel_start.h>
#include <usart_basic.h>
#include "main.h"
#include "motor_driver.h"
#include "usart_debug.h"
#include "motion_driver.h"

volatile float arc;
volatile uint16_t distance_trig;

/**
 * \brief Move object horizontally.
 *
 * \param[in] angle	Range between (+-) 1deg and 360 deg
 * \param[in] speed	Range between (+-) 1% and 100%
 * \param[in] distance	Range between (+-) 1mm and 32000mm
 *	If speed or distance value are negative, move object backward.
 *
 * \return Nothing
 */
void mm_run ( int16_t angle, int8_t speed, int16_t distance ) {
	motor_start();
	_delay_ms(10);
	motor_speed ( motor_front_right, speed);
	motor_speed ( motor_front_left, speed);
	motor_speed ( motor_back_right, speed);	
	motor_speed ( motor_back_left, speed);
	motor_sh_reg_send ();
	_delay_ms(100);

	if (distance < 0) distance *= -1;
	distance_trig = (mm_total_distance() + distance);
	
	/* UART debug */
#ifdef UART_DEBUG
	usart_debug.mm_distance_trig = distance_trig;
	usart_debug.mm_angle = angle;
#endif
	
	while ( 1 ) {
		/*
			TO DO: Add protection if distance_trig does not changes.
			
			Run this loop every time period set in TIM0 (OCA COMP INT),
			calculate new distance and increase total trip counter value.
			Be carefully with TIM0 INT, if somethings switches off TIM0 
			you can stuck in loop.
		*/
		mm_trip_distance_calc();
		if (mm_total_distance() > distance_trig) {
			/*	Turn off all motors in FAST STOP mode. */
			motor_stop();
		}
	}
}
/**
 * \brief Rotate object by a specific angle.
 * TO DO: Add protection if distance_trig does not changes.
 *
 * \param[in] angle	Range between (+-) 1deg and 360 deg
 * \param[in] speed	Range between (+-) 1% and 100%
 *	If speed or distance value are negative, turn object to the left,
 *	otherwise turn object to the right.
 *
 * \return Nothing
 */
void mm_rotate ( int16_t angle, uint8_t speed ) {
	motor_start();
	_delay_ms(10);
	
	//	 negative angle
	if (angle < 0) {
		arc = (float)(angle/(float)360) *  2 * (float)3.14 * (float)(WHEEL_DIAMETER / 2) * -1;
		// set "turn left" motor combination as active
		motor_speed ( motor_front_right, speed);
		motor_speed ( motor_back_right, speed);
		motor_speed ( motor_front_left, -1 * speed);
		motor_speed ( motor_back_left, -1 * speed);
		motor_sh_reg_send ();
		_delay_ms(100);
		distance_trig = (mm_total_distance() + arc);
	//	 positive angle
	} else if (angle > 0) {
		arc = (float)(angle/(float)360) *  2 * (float)3.14 * (float)(WHEEL_DIAMETER / 2);
		// set "turn right" motor combination as active
		motor_speed ( motor_front_right, -1 * speed);
		motor_speed ( motor_back_right, -1 * speed);
		motor_speed ( motor_front_left, speed);
		motor_speed ( motor_back_left, speed);
		motor_sh_reg_send ();
		_delay_ms(100);
		distance_trig = (mm_total_distance() + arc);
	}

	/* UART debug */
#ifdef UART_DEBUG
	usart_debug.mm_distance_trig = distance_trig;
	usart_debug.mm_angle = angle;
	usart_debug.mm_arc = arc;
#endif
	
	while ( distance_trig > 0 ) {
		/*	Single 360deg rotate it is a 204mm distance. */
		mm_trip_distance_calc();
		if (mm_total_distance() > distance_trig) {
			distance_trig = 0;
			motor_stop();
		}
	}
}
/**
 * \brief Distance traveled, taking into account the direction of movement.
 *	i.e. start from 0, move forward 1000 units and move backward 300 units 
 *	Logic distance should be 700 units.
 *
 * \return Logic distance.
 */
int32_t mm_logic_distance ( void ) {
	return distance_logic;
}
/**
 * \brief Total distance traveled.
 *	i.e. start from 0, move forward 1000 units and move backward 300 units 
 *	Total distance should be 1300 units.
 *
 * \return Logic distance.
 */
uint32_t mm_total_distance ( void ){
	return distance_tot;
}
/**
 * \brief It returns horizon side (north-east, east-west) after last important event.
 *
 * \return -1 If it was on west side
 * \return +1 If it was on east side
 * \return  0 If input side value was over the input values
 */
int8_t mm_side_direction ( int8_t side ){
	if (side == -1) {
		//	left side
		return L_side_dir;
	} else if (side == 1) {
		//	right side
		return R_side_dir;
	}
	return 0;
}
/**
 * \brief Return current rotation direction.
 *
 * \return -1 During the left rotation procedure.
 * \return +1 During the right rotation procedure.
 * \return  0 If rotation procedure has been finished.
 */
int8_t mm_current_rotation ( void ) {
	if ((L_side_dir == 1) && (R_side_dir == 1)) return 0;
	if ((L_side_dir == -1) && (R_side_dir == -1)) return 0;
	if ((L_side_dir == -1) && (R_side_dir == 1)) return -1;
	if ((L_side_dir == 1) && (R_side_dir == -1)) return 1;	
	return 0;
}
/**
 * \brief Calculate the distance traveled and update distance registers.
 *
 * \return  Nothing
 */
void mm_trip_distance_calc ( void ) {
	if (TC0_OVF_flag == 1) {
		if ((distance_tot_prev + 1000) > distance_tot) {
			distance_tot_prev = distance_tot;
#ifdef UART_DEBUG
			usart_TX_event();
#endif
		}
		float ENC_M1_distance, ENC_M2_distance, ENC_M3_distance, ENC_M4_distance;
		
		if (R_side_dir == 0) R_side_dir = 1;
		if (L_side_dir == 0) L_side_dir = 1;

		arc = (float)(((360/(2*20))/(float)360)) *  2 * (float)3.14 * (float)(WHEEL_DIAMETER / 2);
		ENC_M1_distance = ENC_M1_slot_cnt * arc;
		ENC_M2_distance = ENC_M2_slot_cnt * arc;
		ENC_M3_distance = ENC_M3_slot_cnt * arc;
		ENC_M4_distance = ENC_M4_slot_cnt * arc;
		
		uint16_t side_L, side_R;
		side_R = (uint16_t) ENC_M1_distance + ENC_M2_distance;
		side_L = (uint16_t) ENC_M3_distance + ENC_M4_distance;
		distance_tot += (uint32_t)((side_R / 2) + (side_L / 2))/2;
		
		//	 if right side is set to move forward, increase logic distance counter
		if (is_bit_set(motor_ctrl_reg, M1A)) {
			R_side_dir = 1;
			distance_logic += (uint16_t) (side_R / 2);
		}
		//	 if right side is set to move backward, decrease logic distance counter
		if (is_bit_set(motor_ctrl_reg, M1B)) {
			R_side_dir = -1;
			distance_logic -= (uint16_t) (side_R / 2);
		}
		
		//	 if left side is set to move forward, increase logic distance counter
		if (is_bit_set(motor_ctrl_reg, M3A)) {
			L_side_dir = 1;
			distance_logic += (uint16_t) (side_L / 2);
		}
		//	 if left side is set to move backward, decrease logic distance counter
		if (is_bit_set(motor_ctrl_reg, M3B)) {
			L_side_dir = -1;
			distance_logic -= (uint16_t) (side_L / 2);
		}

		ENC_M1_slot_cnt = 0;
		ENC_M2_slot_cnt = 0;
		ENC_M3_slot_cnt = 0;
		ENC_M4_slot_cnt = 0;
		
		TC0_OVF_flag = 0;
	}
}
