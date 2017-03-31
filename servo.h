/************************************************************************/
// File:			servo.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Servo driver for sensor tower
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

#ifndef SERVO_H_
#define SERVO_H_

/*  Initializes PWM for correct pins and timer for the servo
    Non-inverted fast PWM, 20ms period, prescaler 64            
    Servo Output: PortD Pin 4 - 16bit Timer1 (OC1B) */
void vServo_init(uint8_t servoAngleDeg);
/*  Set servo angle to a specific degree */
void vServo_setAngle(uint8_t ServoAngleDeg);

#endif /* SERVO_H_ */