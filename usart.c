/************************************************************************/
// File:			usart.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver USART. Protected printf with mutex.
/************************************************************************/
#include <stdio.h>
#include <avr/io.h>

#include "usart.h"

/* Baud rate set in defines */
#include "defines.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern SemaphoreHandle_t xUartMutex;

static void uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

/* Set up baud prescale according to datasheet table 17-1 page 174 */
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16))) - 1)

/************************************************************************/
//Initialize USART driver, note that RXD0/TXD0 (PD0/PD1) is used
// Note that the nRF51 dongle is limited to send 20 characters
// in each package
/************************************************************************/
void vUSART_init(){
    /* Set baud rate, has to match nRF51 dongle! */
    UBRR2H = (unsigned char)(BAUD_PRESCALE>>8);
    UBRR2L = (unsigned char)BAUD_PRESCALE;
    
    /* RX/TX Complete, data register empty */
    UCSR2A = (1<<RXC2) | (1<<TXC2) | (1<<UDRE2);

    /* Enable reciever, transmitter, and recieve interrupt enable*/
    UCSR2B = (1<<RXEN2) | (1<<TXEN2) | (1<<RXCIE2);

    /* Set frame format: 8data, 1 stop bit, no parity */
    UCSR2C = (1<<UCSZ20) | (1<<UCSZ21);
    UCSR2C &= ~((1<<USBS2) & (1<<UPM21) & (1<<UPM20));

    stdout = &mystdout; //Required for printf init
}

void vUSART_sendC(char c){
    if (nRFconnected){ // No point in sending if we're not connected.
        while ( !( UCSR2A & (1<<UDRE2)) );
        UDR2 = c;
    }
}

// Included to be able to use printf
static void uart_putchar(char c, FILE *stream)
{
    if (nRFconnected){ // No point in sending if we're not connected.        
        xSemaphoreTake(xUartMutex,200);
        loop_until_bit_is_set(UCSR2A, UDRE2); // Macro from <avr/io.h>, wait until bit bit in IO register is set.
        UDR2 = c;
        xSemaphoreGive(xUartMutex);
    } 
}

void vUSART_sendS(char *s){
    while (*s != 0x00){
        vUSART_sendC(*s);
        s++;
    }
}

void vUSART_sendHandshake(){
    printf(HANDSHAKE
    ROBOT_TOTAL_WIDTH_MM","     // width of robot
    ROBOT_TOTAL_LENGTH_MM","    // length of robot
    SENSOR_TOWER_OFFSET_Y_MM"," // sensor tower X offset from center
    SENSOR_TOWER_OFFSET_X_MM"," // sensor tower Y offset from center
    ROBOT_AXEL_OFFSET_MM","     // axel offset from center
    SENSOR_OFFSET_RADIUS_MM","  // sensor offset from center
    SENSOR_OFFSET_RADIUS_MM","  // sensor offset from center
    SENSOR_OFFSET_RADIUS_MM","  // sensor offset from center
    SENSOR_OFFSET_RADIUS_MM","  // sensor offset from center
    SENSOR1_HEADING_DEG","      // sensor headings ...
    SENSOR2_HEADING_DEG","
    SENSOR3_HEADING_DEG","
    SENSOR4_HEADING_DEG","
    ROBOT_DEADLINE_MS"}\n");    // update-frequency, LF

}

void vUSART_sendUpdate(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm){
    printf(UPDATE"%i,%i,%i,%i,%i,%i,%i,%i}\n",x_cm,y_cm,heading_deg,towerAngle_deg,S1_cm,S2_cm,S3_cm,S4_cm);
}