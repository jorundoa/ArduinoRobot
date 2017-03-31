/************************************************************************/
// File:			distSens_g2d12.h
// Authors:			Håkon Skjelten, NTNU Fall 204
//                  Johannes Schrimpf, NTNU Fall 2007
//                  Jannicke Selnes Tusvik, NTNU Fall 2009
//                  Erlend Ese, NTNU Spring 2016
//
// Purpose:         Driver for infrared distance sensors: Sharp G2D12
//
// FUNCTIONS IMPLEMENTED:
//
// Port and pins defined in defines.h file
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

#ifndef DISTIR_G2D12_H_
#define DISTIR_G2D12_H_

/* Initialize distance sensors and ADC */
void vDistSens_init();

/* Turn on distance sensors */
void vDistSens_On();

/* Turn off distance sensors */
void vDistSens_Off();

/* Reads a value from the IR sensor ui8_num and returns a value in cm */
uint8_t ui8DistSens_readCM(uint8_t distSensDir);

/* Reads a value from the IR sensor ui8_num and returns analog value */
uint8_t ui8DistSens_readAnalog(uint8_t distSensDir);

#endif /* DISTIR_G2D12_H_ */