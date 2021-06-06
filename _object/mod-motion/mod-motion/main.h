#ifndef MAIN_H_
#define MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

void motor_run_fwd ( uint8_t motor_no );

void motor_run_back ( uint8_t motor_no );

int8_t motor_speed ( uint8_t motor_no, int8_t speed );

void motor_sh_reg_init( void );

uint32_t map(long x, long in_min, long in_max, long out_min, long out_max);

uint8_t is_bit_set(uint8_t b, uint8_t pos);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */