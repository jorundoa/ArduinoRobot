/************************************************************************/
// File:			motor.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Register ticks from optical encoders mounted to
//                  cogwheels
//
// Note: Motor is active low
// ISR routines in main.c
/************************************************************************/


#ifndef MOTOR_H_
#define MOTOR_H_

/************************************************************************/
// Initialize the motor encoder and control pins
// Trigger on rising edge
// Input: pins PD2 and PD3 (INT0 and INT1)
// Note: ISR Routines are in main.c
/************************************************************************/
void vMotor_init();

void vMotorMoveLeftForward(uint8_t actuation, uint8_t *leftWheelDirection);
void vMotorMoveRightForward(uint8_t actuation, uint8_t *rightWheelDirection);

void vMotorMoveLeftBackward(uint8_t actuation, uint8_t *leftWheelDirection);
void vMotorMoveRightBackward(uint8_t actuation, uint8_t *rightWheelDirection);

void vMotorBrakeLeft();
void vMotorBrakeRight();

void vMotorGlideLeft();
void vMotorGlideRight();

/* Soft PWM for motor logic. Actuation is set in percent "on" during    */
/* the duty cycle of 20 ms 0%(off) < 40%(min) < 100%(max)               */
/* NB1: This function HAS to be called from a task                      */
/* NB2: vTaskDelay(0) will cause this task to yield                     */
/* Movement can be moveForward, moveBackward,                           */
/* ... moveClockWise or moveCounterClockwise                            */
void vMotorMovementSwitch(uint8_t movement, int16_t leftOutput, int16_t rightOutput, int16_t tmp_leftWheelTicks, int16_t tmp_rightWheelTicks, uint8_t *leftWheelDirection, uint8_t *rightWheelDirection);

/* Handle ISR ticks from encoder */
void vMotorEncoderLeftTickFromISR(uint8_t wheelDirection, int16_t *leftWheelTicks, uint8_t leftEncoderTicks);

/* Handle ISR ticks from encoder */
void vMotorEncoderRightTickFromISR(uint8_t wheelDirection, int16_t *rightWheelTicks, uint8_t rightEncoderTicks);

#endif /* MOTORENCODER_H_ */