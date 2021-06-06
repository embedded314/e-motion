/**
 * \file
 *
 * \brief HC05 UART debug driver.
 *
 *
 */
#ifndef USART_DEBUG_H_
#define USART_DEBUG_H_

#ifdef __cplusplus
extern "C" {
	#endif

volatile struct {
	int8_t	mm_direction;
	int16_t mm_angle;
	int16_t mm_distance_trig;
	float mm_arc;
} usart_debug;

void usart_TX_event ( void );

void usartPutStr(char *s);

void usartPutInt(int16_t value, int16_t radix);

void convertFloatToStr(float f, char * txt, char dec);

#ifdef __cplusplus
}
#endif

#endif /* USART_DEBUG_H_ */