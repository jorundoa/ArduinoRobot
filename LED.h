/************************************************************************/
// File:			LED.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         For debugging and status identification
//
// Port and pins defined in defines.h file
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

#ifndef LED_H_
#define LED_H_

/* Pin defines */
#include "defines.h" 

/* Initialize LEDport pins as output */
void vLED_init();

/* Set a specific, single LED high */
void vLED_singleHigh(int ledCOLOR);

/* Set a specific, single LED low */
void vLED_singleLow(int ledCOLOR);

/* Toggle a single LED */
void vLED_toggle(int ledCOLOR);

#endif /* LED_H_ */