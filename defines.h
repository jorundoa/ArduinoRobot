/************************************************************************/
// File:			defines.h
// Author:			Erlend Ese, NTNU Spring 2016
// Defines located in one file
//
// Note:
//  Defines regarding FreeRTOS are not here, check
//      - FreeRTOSConfig.h
//      - port.c in portable/GCC/ATmega1284
//  Not all registers are redefined 
//      - Timer/Counter Control registers
//      - Interrupt control registers
//      - ADCs
//
/************************************************************************/

#ifndef DEFINES_H_
#define DEFINES_H_
/************************************************************************/
/* PHYSICAL CONSTANTS - If the robot is changed these need to be changed 
 Some of these will be sent to server during the start-up-handshake   
 Wheel factor is the circumference divided by ticks per rotation 
 Cog wheels: 40/16 and 40/8
 -> ((pi*81.6)/(198 * 40/16 * 40/8)) / 2 = 0.2072 mm 
                        Length the robot travels per divided tick
                        (Ticks are divided in motor.c)                  */
#define WHEELBASE_MM             215.0 /* Length between wheel centers  */
#define ROBOT_TOTAL_WIDTH_MM     "210" /* From outer rim to outer rim   */
#define ROBOT_TOTAL_LENGTH_MM    "220" /* From front to aft, total      */
#define ROBOT_AXEL_OFFSET_MM     "39"  /* From center of square         */
#define SENSOR_TOWER_OFFSET_X_MM "30"  /* From center of square         */ 
#define SENSOR_TOWER_OFFSET_Y_MM "0"   /* From center of square         */
#define SENSOR_OFFSET_RADIUS_MM  "22"  /* From center of tower          */
#define ROBOT_DEADLINE_MS        "200" /* Interval between measurements */
#define SENSOR1_HEADING_DEG      "0"   /* Sensor angle relative to body */
#define SENSOR2_HEADING_DEG      "90"
#define SENSOR3_HEADING_DEG      "180"
#define SENSOR4_HEADING_DEG      "270"

#define WHEEL_FACTOR_MM 0.2072          /* Calculated, see above CHANGE TO 0.2340`?  */ 

/************************************************************************/
/* Program settings                                                     */
#define PERIOD_MOTOR_MS         20
#define PERIOD_ESTIMATOR_MS     40
#define PERIOD_SENSORS_MS       200
#define moveStop                0
#define moveForward             1
#define moveBackward            2
#define moveClockwise           3
#define moveCounterClockwise    4
#define moveLeftForward         5
#define moveRightForward        6
#define moveLeftBackward        7
#define moveRightBackward       8
#define F_CPU                   16000000UL

/************************************************************************/
/* MICROCONTROLLER PIN CONSTANTS                                        */
/* Distance sensor                                                      */
#define distSensPort    PORTF   /* Sensors connected to port A          */
#define distSensReg     DDRF    /* Port A register used                 */
#define distSensLeft    PINF0   /* Forward sensor connected to PA4      */
#define distSensRear    PINF3   /* Backward sensor connected to PA3     */
#define distSensRight   PINF2   /* Right sensor connected to PA2        */
#define distSensFwd     PINF1   /* Left sensor connected to PA3         */

/* Motor control                                                        */
#define motorRightOn        PINB7
#define motorRightPWM       OCR0A
#define motorRightForward   PINA2   /* Right motor, pin 1               */
#define motorRightBackward  PINA4   /* Right motor, pin 2               */

#define motorLeftOn         PING5
#define motorLeftPWM        OCR0B
#define motorLeftBackward   PINC7   /* Left motor, pin 1                */
#define motorLeftForward    PINC5   /* Left motor, pin 2                */

/* Motor encoder                                                        */
#define encoderPinLeft  PIND3       /*   Left encoder connected to PD3  */
#define encoderPinRight PIND2       /*   Right encoder connected to PD2 */
#define leftWheelCount INT3_vect    /* Interrupt vector for PD3         */
#define rightWheelCount INT2_vect   /* Interrupt vector for PD2         */


/* Servo                                                                */
#define servoReg        DDRB        /* Port D register used             */
#define servoPin        PINB5       /* Servo connected to pin 4         */
#define servoOCR        OCR1A       /* Output compare register for PB5  */

/* LED                                                                  */
#define ledPORT		    PORTL   /*  LEDs connected to port D            */
#define ledReg          DDRL    /*  Port D register used                */
#define ledGREEN	    PINL2   /*  Green LED connected                 */
#define ledYELLOW	    PINL1   /*  Yellow LED connected                */
#define ledRED		    PINL0   /*  Green LED connected                 */

/* Communication                                                        */
#define USART_BAUDRATE 38400UL  /* If changed, also change in nRF51 code*/
#define nRF17 PINA7             /* Available GPIO pins from nRF51 Dongle*/
#define nRF18 PINC6             
#define nRF19 PINE4             
#define nRF51_status INT4_vect  /* PinE4 has interrupt available        */
#define nRF20 PINC2             

#define DDR_SPI DDRB            /* Data direction register for SPI      */
#define DD_MOSI PINB2           /* MOSI pin                             */
#define DD_SCK  PINB1           /* SCK pin                              */
#define DD_MISO PINB3           /* MISO pin                             */
#define IMU_SS  PINB0           /* Slave select pin for IMU             */


/************************************************************************/
/* COMMANDS                                                             */
#define UPDATE "{U,"
#define HANDSHAKE "{H,"
#define STATUS "{S,"
#define STATUS_CONFIRM "{S,CON}"
#define STATUS_PAUSE "{S,PAU}"
#define STATUS_UNPAUSE "{S,UNP}"
#define STATUS_IDLE "{S,IDL}"
#define TRUE 1
#define FALSE 0

/************************************************************************/
/* Macros                                                               */
#define nRFconnected !(PINE & (1<<nRF19))
#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI


#endif /* DEFINES_H_ */