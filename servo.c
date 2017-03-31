/************************************************************************/
// File:			servo.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Servo driver for sensor tower
/************************************************************************/

/*  AVR includes    */
#include <avr/io.h>

/*  Custom includes    */
#include "servo.h"
#include "defines.h"    
    
/************************************************************************/
//     Values found by testing and then linearized in matlab
//     0 degrees = 1300, 90 degrees = 3100
//     Yields:
//      y =  20.00*x + 1300
// MATLAB CODE START
// n = 90;
// for i=1:n+1
//     x(i) = 20.00 * (i-1) + 1300;
//     array = transpose(round(x));
// end
// disp('Check variable array for row-table of pwm values');
// MATLAB CODE STOP
// Array index contains corresponding calibrated PWM value.
/************************************************************************/
/************************************************************************/
// Array to map angle from degrees to pulse-width in order to avoid
// floating point operation. More details in declaration below
/************************************************************************/
    const uint16_t  DEG_TO_PWM[91] = {
        1300,1320,1340,1360,1380,1400,1420,1440,1460,1480,1500,1520,
        1540,1560,1580,1600,1620,1640,1660,1680,1700,1720,1740,1760,
        1780,1800,1820,1840,1860,1880,1900,1920,1940,1960,1980,2000,
        2020,2040,2060,2080,2100,2120,2140,2160,2180,2200,2220,2240,
        2260,2280,2300,2320,2340,2360,2380,2400,2420,2440,2460,2480,
        2500,2520,2540,2560,2580,2600,2620,2640,2660,2680,2700,2720,
        2740,2760,2780,2800,2820,2840,2860,2880,2900,2920,2940,2960,
        2980,3000,3020,3040,3060,3080,3100
    };

/************************************************************************/
// Initializes PWM for correct pins and timer for the servo
// Non-inverted, fast PWM, clear on compare match, 20ms period, presc 8
// Output: PortD Pin 4
/************************************************************************/
void vServo_init(uint8_t servoAngleDeg){
    /* Clear OCnA/OCnB on Compare Match, set */
    /* OCnA/OCnB at BOTTOM (non-inverting mode) */
    /* Datasheet p.132 Table 14-3 */
    TCCR1A |= (1<<COM1A1) | (0<<COM0A0);
    
    /* Waveform generation mode 14: Fast PWM */
    /* top: ICRn, Update bottom, flag set on top */
    /* Datasheet p.133 Table 14-5 */
    TCCR1B |= (1<<WGM13) | (1<<WGM12);
    TCCR1A |= (1<<WGM11) | (0<<WGM10);

    /* Clock select bit description: */
    /* clkI/O/8 (From prescaler) - Datasheet p.134 Table 14-6*/
    TCCR1B |= (0<<CS12) | (1<<CS11) | (0<<CS10);
    
    /* 50Hz 20ms period => 16Mhz/(8clk*50Hz) - 1 = ICR1] */
    /* Datasheet p.125 */
    ICR1 = 39999; // 49999 for 20mhz, 39 999 for 16mhz
    
    /*PortB Pin 5 as servo PWM Output (OC1A)*/
    servoReg |= (1<<servoPin);
    
    /*  Set angle to desired start angle (usually 0)*/
    vServo_setAngle(servoAngleDeg);
}

/* Sets servo angle to a specific degree */
void vServo_setAngle(uint8_t ServoAngleDeg){
    /* Ensure feasible values */
    if (ServoAngleDeg >= 90){
        ServoAngleDeg = 90;
    }
    else if(ServoAngleDeg <= 0){
        ServoAngleDeg = 0;
    }
    /* Fetch pulse width from array and set to output */
    servoOCR = DEG_TO_PWM[ServoAngleDeg];
}