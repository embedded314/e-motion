/**
 * \file
 *
 * \brief Motor M3 PWM driver implementation.
 *
 *
 */

/**
 * \defgroup doc_driver_pwm_basic PWM Basic Driver
 * \ingroup doc_driver_pwm
 *
 * \section doc_driver_pwm_normal_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include <pwm_M3.h>

/** Function pointer to callback function called by IRQ.
    NULL=default value: No callback function is to be used.
*/
pwm_irq_cb_t PWM_M3_cb = NULL;

/**
 * \brief Initialize PWM
 * If module is configured to disabled state, the clock to the PWM is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 */
int8_t PWM_M3_init()
{

	/* Enable TC4 */
	PRR1 &= ~(1 << PRTIM4);

	TCCR4A = (1 << COM4A1) | (0 << COM4A0)   /* Toggle OCA on Compare Match */
	         | (0 << COM4B1) | (0 << COM4B0) /* Normal port operation, OCB disconnected */
	         | (1 << WGM41) | (0 << WGM40);  /* TC16 Mode 14 Fast PWM */

	TCCR4B = (0 << WGM43) | (0 << WGM42)                /* TC16 Mode 14 Fast PWM */
	         | 0 << ICNC4                               /* Input Capture Noise Canceler: disabled */
	         | 0 << ICES4                               /* Input Capture Edge Select: disabled */
	         | (1 << CS42) | (0 << CS41) | (0 << CS40); /* IO clock divided by 1024 */

	// ICR1 = 0x0; /* Counter top value: 0x0 */

	OCR4A = 0x0FF; /* Output compare A: 0x1 */

	// OCR1B = 0x0; /* Output compare B: 0x0 */

	TIMSK4 = 0 << OCIE4B   /* Output Compare B Match Interrupt Enable: disabled */
	         | 0 << OCIE4A /* Output Compare A Match Interrupt Enable: disabled */
	         | 0 << ICIE4  /* Input Capture Interrupt Enable: disabled */
	         | 0 << TOIE4; /* Overflow Interrupt Enable: enabled */

	return 0;
}

/**
 * \brief Enable PWM_M3
 * 1. If supported by the clock system, enables the clock to the PWM
 * 2. Enables the PWM module by setting the enable-bit in the PWM control register
 *
 * \return Nothing
 */
void PWM_M3_enable()
{
	/* Enable TC4 */
	PRR1 &= ~(1 << PRTIM4);
}

/**
 * \brief Disable PWM_M3
 * 1. Disables the PWM module by clearing the enable-bit in the PWM control register
 * 2. If supported by the clock system, disables the clock to the PWM
 *
 * \return Nothing
 */
void PWM_M3_disable()
{
	/* Disable TC4 */
	PRR1 |= (1 << PRTIM4);
}

/**
 * \brief Enable PWM output on channel 0
 *
 * \return Nothing
 */
void PWM_M3_enable_output_ch0()
{

	TCCR4A |= ((0 << COM4A1) | (0 << COM4A0));
}

/**
 * \brief Disable PWM output on channel 0
 *
 * \return Nothing
 */
void PWM_M3_disable_output_ch0()
{

	TCCR4A &= ~((0 << COM4A1) | (0 << COM4A0));
}

/**
 * \brief Enable PWM output on channel 1
 *
 * \return Nothing
 */
void PWM_M3_enable_output_ch1()
{

	TCCR4A |= ((0 << COM4B1) | (0 << COM4B0));
}

/**
 * \brief Disable PWM output on channel 1
 *
 * \return Nothing
 */
void PWM_M3_disable_output_ch1()
{

	TCCR4A &= ~((0 << COM4B1) | (0 << COM4B0));
}

/**
 * \brief Load COUNTER register in PWM_M3
 *
 * \param[in] counter_value The value to load into COUNTER
 *
 * \return Nothing
 */
void PWM_M3_load_counter(PWM_M3_register_t counter_value)
{
	TCNT4 = counter_value;
}

/**
 * \brief Load TOP register in PWM_M3.
 * The physical register may different names, depending on the hardware and module mode.
 *
 * \param[in] counter_value The value to load into TOP.
 *
 * \return Nothing
 */
void PWM_M3_load_top(PWM_M3_register_t top_value)
{
	ICR4 = top_value;
}

/**
 * \brief Load duty cycle register in for channel 0.
 * The physical register may have different names, depending on the hardware.
 * This is not the duty cycle as percentage of the whole period, but the actual
 * counter compare value.
 *
 * \param[in] counter_value The value to load into the duty cycle register.
 *
 * \return Nothing
 */
void PWM_M3_load_duty_cycle(PWM_M3_register_t duty_value)
{
	OCR4A = duty_value;
}

/**
 * \brief Load duty cycle register in for channel 1.
 * The physical register may have different names, depending on the hardware.
 * This is not the duty cycle as percentage of the whole period, but the actual
 * counter compare value.
 *
 * \param[in] counter_value The value to load into the duty cycle register.
 *
 * \return Nothing
 */
void PWM_M3_load_duty_cycle_ch1(PWM_M3_register_t duty_value)
{
	OCR4B = duty_value;
}

/**
 * \brief Register a callback function to be called at the end of the overflow ISR.
 *
 * \param[in] f Pointer to function to be called
 *
 * \return Nothing.
 */
void PWM_M3_register_callback(pwm_irq_cb_t f)
{
	PWM_M3_cb = f;
}

ISR(TIMER4_OVF_vect)
{
	static volatile uint8_t callback_count = 0;

	// Clear the interrupt flag
	TIFR4 = TOV4;

	// callback function - called every 0 passes
	if ((++callback_count >= PWM_M3_INTERRUPT_CB_RATE) && (PWM_M3_INTERRUPT_CB_RATE != 0)) {
		if (PWM_M3_cb != NULL) {
			PWM_M3_cb();
		}
	}
}
