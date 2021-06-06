/**
 * \file
 *
 * \brief Motion driver.
 *
 *
 */
#ifndef MOTION_DRIVER_H_
#define MOTION_DRIVER_H_

#ifdef __cplusplus
extern "C" {
	#endif

#define WHEEL_DIAMETER	65
#define ENC_SLOT_DIST 10
#define ENC_SLOTS 20

//#define UART_DEBUG

volatile uint32_t ENC_M1_ts, ENC_M1_ts_prev;
volatile uint32_t ENC_M2_ts, ENC_M2_ts_prev;
volatile uint32_t ENC_M3_ts, ENC_M3_ts_prev;
volatile uint32_t ENC_M4_ts, ENC_M4_ts_prev;
volatile uint16_t ENC_M1_slot_cnt, ENC_M2_slot_cnt, ENC_M3_slot_cnt, ENC_M4_slot_cnt;
volatile uint32_t distance_tot, distance_tot_prev;
volatile int32_t  distance_logic;
volatile int8_t R_side_dir, L_side_dir;

void mm_run ( int16_t angle, int8_t speed, int16_t distance );

void mm_rotate ( int16_t angle, uint8_t speed );

int32_t mm_logic_distance ( void );

uint32_t mm_total_distance ( void );

int8_t mm_side_direction ( int8_t side );

int8_t mm_current_rotation ( void );

void mm_trip_distance_calc ( void );

#ifdef __cplusplus
}
#endif

#endif /* MOTION_DRIVER_H_ */