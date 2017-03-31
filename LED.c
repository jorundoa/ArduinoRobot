/************************************************************************/
// File:			LED.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         For debugging and status identification
// 
// Port and pins defined in defines.h file
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

/*  Custom includes    */
#include "LED.h"

/* Initialize LEDport pins as output */
void vLED_init(){
	ledReg |= (1<<ledGREEN) | (1<<ledYELLOW) | (1<<ledRED);
}

/* Set a specific, single LED high */
void vLED_singleHigh(int ledCOLOR){
	if (ledCOLOR == ledGREEN)			ledPORT |= (1<<ledGREEN);
	else if (ledCOLOR == ledYELLOW)		ledPORT |= (1<<ledYELLOW);
	else if (ledCOLOR == ledRED)		ledPORT |= (1<<ledRED);
}

/* Set a specific, single LED low */
void vLED_singleLow(int ledCOLOR){
    if (ledCOLOR == ledGREEN)			ledPORT &= ~(1<<ledGREEN);
    else if (ledCOLOR == ledYELLOW)		ledPORT &= ~(1<<ledYELLOW);
    else if (ledCOLOR == ledRED)		ledPORT &= ~(1<<ledRED);
}

/* Toggle a single LED */
void vLED_toggle(int ledCOLOR){
	if (ledCOLOR == ledGREEN)		ledPORT ^= (1<<ledGREEN);
	else if (ledCOLOR == ledYELLOW)	ledPORT ^= (1<<ledYELLOW);
	else if (ledCOLOR == ledRED)		ledPORT ^= (1<<ledRED);
}

