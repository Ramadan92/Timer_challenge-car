/*
 * interrupt.h
 *
 *  Created on: Dec 9, 2019
 *      Author: ahmed
 */

#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "registers.h"
/* defining the vector numbers to recognizable names*/
#define INT0_vect			__vector_1			//External interrupt 0 vector
#define INT1_vect			__vector_2			//External interrupt 1 vector
#define INT2_vect			__vector_3			//External interrupt 2 vector
#define TIMER2_COMP_vect	__vector_4			//Timer2 compare match vector
#define TIMER2_OVF_vect		__vector_5			//Timer2 overflow vector
#define TIMER1_CAPT_vect	__vector_6			//Timer1 input capture vector
#define TIMER1_COMPA_vect	__vector_7			//Timer1 compare match A vector
#define TIMER1_COMPB_vect	__vector_8			//Timer1 compare match B vector
#define TIMER1_OVF_vect		__vector_9			//Timer1 overflow vector
#define TIMER0_COMP_vect	__vector_10			//Timer0 compare match vector
#define TIMER0_OVF_vect		__vector_11			//Timer0 overflow vector
#define SPI_STC_vect		__vector_12
#define USART_RXC_vect		__vector_13
#define USART_UDRE_vect		__vector_14
#define USART_TXC_vect		__vector_15
#define ADC_vect			__vector_16
#define EE_RDY_vect			__vector_17
#define ANA_COMP_vect		__vector_18
#define TWI_vect			__vector_19
#define SPM_RDY_vect		__vector_20

/* Defining the interrupt service routine function to be a signal function */
#define InterruptServiceRoutine(vector_num) void vector_num(void) __attribute__((signal));\
												void vector_num(void)
												
/* #define sei()  __asm__ __volatile__ ("sei" ::)
#define cli()  __asm__ __volatile__ ("cli" ::) */

#endif /* INTERRUPT_H_ */
