/**
 * \file
 *
 * \brief HC05 comm driver.
 *
*/
#include <stdio.h>
#include <util/delay.h>
#include <atmel_start.h>
#include "motion_driver.h"
#include "usart_debug.h"

void usart_TX_event(void) {
	//	current direction
    //usartPutInt(0x00, 10);
    //usartPutStr(";");
	
	//	current angle
    //usartPutInt(0x01, 10);
    //usartPutStr(";");

	//	arc
    //usartPutInt(0x02, 10);
    //usartPutStr(";");

	//	total_distance
    usartPutInt(mm_total_distance(), 10);
    usartPutStr(";");

	//	logic_distance
    //usartPutInt(mm_logic_distance(), 10);
    //usartPutStr(";");

	//	current speed
	//	PWM M1-M4
    usartPutStr("\r\n");
}

void usartPutStr(char *s) {
    register char c;
    while ((c = *s++)) USART_0_write(c);
}

void usartPutInt(int16_t value, int16_t radix) {
    char txt[10];

    if (value < 0) {
        usartPutStr("-");
        value *= -1;
    }
    convertFloatToStr((float) value, txt, 2);
    usartPutStr(txt);
}

void convertFloatToStr(float f, char * txt, char dec) {
	unsigned long n;
	short i = 0, j, tmp = 0, len = dec;
	unsigned long p[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

	n = f * p[dec];

	if (n < p[dec])
	tmp = 1;

	do {
		if ((i == dec) && (dec != 0)) {
			txt[i++] = '.';
			continue;
		}
		txt[i++] = n % 10 + '0';
		n /= 10;
	} while((len-- > 0) || n);
	
	if (tmp)
	txt[i++] = '0';
	
	txt[i] = '\0';

	for (j = i - 1, i = 0; i < j; i++, j--)
	tmp = txt[i], txt[i] = txt[j], txt[j] = tmp;
}
