/**
 * \file
 *
 * \brief Motor driver.
 *
 *
 */
#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#ifdef __cplusplus
extern "C" {
	#endif

#define M3A   7
#define M2A   6
#define M1A   5
#define M1B   4
#define M2B   3
#define M4A   2
#define M3B   1
#define M4B   0

#define motor_front_right 1
#define motor_back_right 2
#define motor_front_left 3
#define motor_back_left 4

#define motor_min_PWM	0x120
#define motor_max_PWM	0x1FF

volatile uint8_t motor_ctrl_reg;
volatile uint8_t bit_state [8];

void motor_stop( void );

void motor_start( void );

void motor_free( void );

void speed_encoder_reg_update( void );

void motor_sh_reg_init();

void motor_sh_reg_send ( void );

void motor_run_fwd ( uint8_t motor_no );

void motor_run_back ( uint8_t motor_no );

int8_t motor_speed ( uint8_t motor_no, int8_t speed );

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H_ */