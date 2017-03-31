/************************************************************************/
// File:			functions.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Various functions needed
//
/************************************************************************/

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

/* Take any angle and put it inside -pi,pi */
void vFunc_Inf2pi(float *angle_in_radians);

// parse the update message
void vFunc_ParseUpdate(char *cin, float *theta, int16_t *radius);

// reverses a string 'str' of length 'len'
void vFunc_reverse(char* p, char* q);

// increase function for decimal parsing
char* vFunc_inc(char* s, char* e);

// C program for implementation of ftoa()
char* vFunc_ftoa(double num, char* dest, int afterPoint);

/* Legacy functions that may become useful for future work */
/**
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed and moved to own file by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @file position.c
 * @brief position routines
 */

/**
 *  calculates the x-position of a measured object. 
 */
 int16_t objectPosX(int16_t i16_sensorT, uint8_t ui8_sensorDelta, int8_t i8_servoAngle, float f_robotTheta, float f_robotX);	
/**
 *  calculates the y-position of a measured object.
 */
 int16_t objectPosY(int16_t i16_sensorT, uint8_t ui8_sensorDelta, int8_t i8_servoAngle, float f_robotTheta, float f_robotY);

/**
 * Calculates a heading and a distance to move to a position
 */
void goRobotPos(int16_t i16_goX, int16_t i16_goY, float f_robotX, float f_robotY, uint8_t *ui8_targetHeading, int8_t *i8_targetDistance);

#endif /* FUNCTIONS_H_ */