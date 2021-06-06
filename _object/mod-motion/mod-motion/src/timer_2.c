/**
 * \file
 *
 * \brief TC8 related functionality implementation.
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 */

/**
 * \addtogroup doc_driver_tc8
 *
 * \section doc_driver_tc8_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <stdio.h>
#include <utils.h>
#include "motion_driver.h"
#include <timer_2.h>

/**
 * \brief Initialize TIMER_0 interface
 *
 * \return Initialization status.
 */
int8_t TIMER_2_init()
{

	/* Enable TC2 */
	PRR0 &= ~(1 << PRTIM2);

	TCCR2A = (  0 << COM2A1) | (0 << COM2A0) /* Normal port operation, OCA disconnected */
	         | (0 << COM2B1) | (0 << COM2B0) /* Normal port operation, OCB disconnected */
	         | (1 << WGM21)  | (0 << WGM20); /* TC8 Mode 2 CTC */

	TCCR2B = (  1 << CS22)   | (1 << CS21) | (1 << CS20)
			 | (0 << WGM22); /* 1024 prescaling */

	TIMSK2 =   0 << OCIE2B   /* Output Compare B Match Interrupt Enable: disabled */
	         | 1 << OCIE2A /* Output Compare A Match Interrupt Enable: enabled */
	         | 0 << TOIE2; /* Overflow Interrupt Enable: disabled */

	OCR2A = 0x06; /* Output compare A: 0x06 9ms */

	// OCR2B = 0x0; /* Output compare B: 0x0 */

	// GTCCR = 0 << TSM /* Timer/Counter Synchronization Mode: disabled */
	//		 | 0 << PSRASY /* Prescaler Reset Timer/Counter2: disabled */
	//		 | 0 << PSRSYNC; /* Prescaler Reset: disabled */

	return 0;
}
uint32_t millis( void ) {
	return millis_cnt;
 }
ISR(TIMER2_COMPA_vect) {
	//	PCIE jest wywolywane w momencie kiedy zmienia stan na przeciwny
	//	niestety nie mozna go wyzerowac wiec trzeba zastosowac trik
	//	jezeli poprzedni timestamp jest rozny od aktualnego tzn. ze byl obsluzony 
	//	slot czyli juz zwiekszylem licznik
	//	predkosc odswiezania mozna ustawic na 0.9ms a przy okazji zrobic sobie funkcje millis
	if (ENC_M1_ts != ENC_M1_ts_prev) {
		ENC_M1_ts_prev = ENC_M1_ts;
		ENC_M1_slot_cnt++;
	}
	if (ENC_M2_ts != ENC_M2_ts_prev) {
		ENC_M2_ts_prev = ENC_M2_ts;
		ENC_M2_slot_cnt++;
	}
	if (ENC_M3_ts != ENC_M3_ts_prev) {
		ENC_M3_ts_prev = ENC_M3_ts;
		ENC_M3_slot_cnt++;
	}
	if (ENC_M4_ts != ENC_M4_ts_prev) {
		ENC_M4_ts_prev = ENC_M4_ts;
		ENC_M4_slot_cnt++;
	}
	millis_cnt++;
	// Clear the interrupt flag
	TIFR2 = OCF2A;
}