/**
 * \file
 *
 * \brief Motor driver.
 *
*/
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <atmel_start.h>
#include "main.h"
#include "motion_driver.h"
#include "timer_0.h"
#include "motor_driver.h"


volatile uint8_t speed_encoder_state = 0x00;

/* Motor driver board pin initialization. */
void motor_sh_reg_init() {
	PORTH_set_pin_dir(4, 1);
	PORTH_set_pin_dir(5, 1);
	PORTG_set_pin_dir(5, 1);
	PORTB_set_pin_dir(6, 1);

	PORTB_set_pin_dir( 5, 1 );
	PORTB_set_pin_level( 5, 1 );

	PORTE_set_pin_dir( 5, 1 );
	PORTE_set_pin_level( 5, 1 );

	PORTH_set_pin_dir( 3, 1 );
	PORTH_set_pin_level( 3, 1 );

	PORTE_set_pin_dir( 3, 1 );
	PORTE_set_pin_level( 3, 1 );
}
/**
 * \brief Set the desired motor direction to move forward.
 *
 * \param[in] motor_no motor number
 *	M1	right-front wheel
 *	M2	right-back wheel
 *	M3	left-front wheel
 *	M4	left-back wheel
 *
 * \return Nothing
 */
void motor_run_fwd ( uint8_t motor_no ) {
	switch ( motor_no ) {
		case 1:
			if ((is_bit_set(motor_ctrl_reg, M1B)) || (!is_bit_set(motor_ctrl_reg, M1A))) {
				motor_ctrl_reg	&= ~(1 << M1B);
				motor_ctrl_reg	|= (1 << M1A);
			}
			break;
		case 2:
			if ((is_bit_set(motor_ctrl_reg, M2A)) || (!is_bit_set(motor_ctrl_reg, M2B))) {
				motor_ctrl_reg	&= ~(1 << M2A);
				motor_ctrl_reg	|= (1 << M2B);
			}
			break;
		case 3:
			if ((is_bit_set(motor_ctrl_reg, M4A)) || (!is_bit_set(motor_ctrl_reg, M4B))) {
				motor_ctrl_reg	&= ~(1 << M4A);
				motor_ctrl_reg	|= (1 << M4B);
			}
			break;
		case 4:
			if ((is_bit_set(motor_ctrl_reg, M3B)) || (!is_bit_set(motor_ctrl_reg, M3A))) {
				motor_ctrl_reg	&= ~(1 << M3B);
				motor_ctrl_reg	|= (1 << M3A);
			}
			break;
	}
}
/**
 * \brief Set the desired motor direction to move backward.
 *
 * \param[in] motor_no motor number
 *	M1	right-front wheel
 *	M2	right-back wheel
 *	M3	left-front wheel
 *	M4	left-back wheel
 *
 * \return Nothing
 */
void motor_run_back ( uint8_t motor_no ) {
	switch ( motor_no ) {
		case 1:
			if ((is_bit_set(motor_ctrl_reg, M1A)) || (!is_bit_set(motor_ctrl_reg, M1B))) {
				motor_ctrl_reg	&= ~(1 << M1A);
				motor_ctrl_reg	|= (1 << M1B);
			}
			break;
		case 2:
			if ((is_bit_set(motor_ctrl_reg, M2B)) || (!is_bit_set(motor_ctrl_reg, M2A))) {
				motor_ctrl_reg	&= ~(1 << M2B);
				motor_ctrl_reg	|= (1 << M2A);
			}
			break;
		case 3:
			if ((is_bit_set(motor_ctrl_reg, M4B)) || (!is_bit_set(motor_ctrl_reg, M4A))) {
				motor_ctrl_reg	&= ~(1 << M4B);
				motor_ctrl_reg	|= (1 << M4A);
			}
			break;
		case 4:
			if ((is_bit_set(motor_ctrl_reg, M3A)) || (!is_bit_set(motor_ctrl_reg, M3B))) {
				motor_ctrl_reg	&= ~(1 << M3A);
				motor_ctrl_reg	|= (1 << M3B);
			}
			break;
	}
	//	update motor driver register
}
/**
 * \brief Set the desired motor speed.
 *
 * \param[in] motor_no motor number
 *	M1	right-front wheel
 *	M2	right-back wheel
 *	M3	left-front wheel
 *	M4	left-back wheel
 *
 * \return Nothing
 */
int8_t motor_speed ( uint8_t motor_no, int8_t speed ) {
	uint8_t mosin = 0;
	if (speed > 0) mosin = speed; 
	else if (speed < 0) mosin = speed * -1;
	uint16_t mots = map (mosin, 0, 100, motor_min_PWM, motor_max_PWM);

	/* Change PWM values but don't sent it to the shift register. */	
	switch ( motor_no ) {
		case 1:
			PWM_M1_load_duty_cycle( mots );
			if (speed > 0) motor_run_fwd ( motor_front_right );
			else if (speed < 0) motor_run_back ( motor_front_right );
			break;
		case 2:
			PWM_M2_load_duty_cycle( mots );
			if (speed > 0) motor_run_fwd ( motor_back_right );
			else if (speed < 0) motor_run_back ( motor_back_right );
			break;
		case 3:
			PWM_M3_load_duty_cycle( mots );
			if (speed > 0) motor_run_fwd ( motor_front_left );
			else if (speed < 0) motor_run_back ( motor_front_left );
			break;
		case 4:
			PWM_M4_load_duty_cycle( mots );
			if (speed > 0) motor_run_fwd ( motor_back_left );
			else if (speed < 0) motor_run_back ( motor_back_left );
			break;
	}
	return mots;
}
volatile uint8_t motor_ctrl_reg_prev;
/*	Send pin configuration to the hardware shift register. */
void motor_sh_reg_send ( void ) {
	/*	Don't make any changes when new configuration is same like the previous one. */
	if (motor_ctrl_reg_prev != motor_ctrl_reg) {
		motor_ctrl_reg_prev = motor_ctrl_reg;

		/* Send serial data to shift register. */
		PORTH_set_pin_level(4, 1);
		_delay_ms(1);
		for (uint8_t i=0; i < 8; i++) {
			bit_state [i] = is_bit_set(motor_ctrl_reg, i);
			if (bit_state [i]) {
				PORTH |= (1 << 5);
			} else {
				PORTH &= ~(1 << 5);
			}
			//	clk
			PORTG_set_pin_level(5, 0);
			_delay_ms(1);
			PORTG_set_pin_level(5, 1);
			_delay_ms(1);
		}
		PORTB_set_pin_level(6, 0);
		_delay_ms(1);
		PORTB_set_pin_level(6, 1);
		_delay_ms(1);
		PORTH_set_pin_level(4, 0);
		_delay_ms(10);
	}
}
/*	Enable all motor drivers. */
void motor_start( void ) {
	/*	Enable shift register EN line. */
	PORTH_set_pin_level(4, 0);

	PWM_M1_enable_output_ch0();
	PWM_M4_enable_output_ch0();
	PWM_M2_enable_output_ch3();
	PWM_M3_enable_output_ch0();
}
/*	Disable all motor drivers. */
void motor_stop( void ) {
	motor_ctrl_reg = 0x00;
	motor_sh_reg_send();
	/*	Enable shift register EN line. */
	PORTH_set_pin_level(4, 0);

	/*	Turn off all PWM channels */
	PWM_M1_disable_output_ch0();
	PWM_M4_disable_output_ch0();
	PWM_M2_disable_output_ch3();
	PWM_M3_disable_output_ch0();

	/*	Set all LM753 pins to H state. */
	PORTB_set_pin_level(5, 1);
	PORTE_set_pin_level(5, 1);
	PORTH_set_pin_level(3, 1);
	PORTE_set_pin_level(3, 1);
}
/*	Free run */
void motor_free( void ) {
	PORTH_set_pin_level(4, 0);
}

/**
 * \brief EXTternal pin interrupt support.
 *	All four state pins are connected to the rotary encoder discs.
 *	Every time when rotary encoder disc slot crossing IR receiver, interrupt is generated.
 *	
 *	If current pin state is different then previous one, check which one motor generates interrupt,
 *	set state flag to pin value and current timestamp as a new for that motor register.
 *	If current pin state is equal to previous (i.e. interrupt is generated faster then slot changing speed)
 *	don't make any updates.
 */
ISR(PCINT2_vect) {
	if (PORTK_get_pin_level(PK7) != is_bit_set(speed_encoder_state, 0)) ENC_M1_ts = millis();
	if (PORTK_get_pin_level(PK6) != is_bit_set(speed_encoder_state, 1)) ENC_M2_ts = millis();
	if (PORTK_get_pin_level(PK5) != is_bit_set(speed_encoder_state, 2)) ENC_M3_ts = millis();
	if (PORTK_get_pin_level(PK4) != is_bit_set(speed_encoder_state, 3)) ENC_M4_ts = millis();

	speed_encoder_reg_update();
}
/*	Update speed encoder value. */
void speed_encoder_reg_update( void ) {
	if (PORTK_get_pin_level(PK7)) speed_encoder_state |= (1 << 0); else speed_encoder_state &= ~(1 << 0);
	if (PORTK_get_pin_level(PK6)) speed_encoder_state |= (1 << 1); else speed_encoder_state &= ~(1 << 1);
	if (PORTK_get_pin_level(PK5)) speed_encoder_state |= (1 << 2); else speed_encoder_state &= ~(1 << 2);
	if (PORTK_get_pin_level(PK4)) speed_encoder_state |= (1 << 3); else speed_encoder_state &= ~(1 << 3);
}
