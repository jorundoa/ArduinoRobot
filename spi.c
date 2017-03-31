/************************************************************************/
// File:			spi.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Communication protocol to peripherals, SPI
/************************************************************************/

#include <avr/io.h>
#include "spi.h"

#include "defines.h"

void vSPI_MasterInit(){
    /* Set MOSI SCK and slave select pin as output */
    DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<IMU_SS);
    DDR_SPI &= ~(1 << DD_MISO); // Set MISO as input
    
    /* Enable SPI, master, set clockrate at fck/128, MSB first */
    /* Max frequency for LSM6DS3 is 10Mhz, we use 156 250Hz */
    // Data is captured on rising edge of clock (CPHA = 0)
    // Base value of the clock is HIGH (CPOL = 1)
    SPCR |= (1<<SPI2X) | (0<<SPR1) | (0<<SPR0);
    SPCR |= (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
    SPCR &= ~(1<<DORD); // MSB first
}

uint8_t ui8SPI_MasterTransmit(char cData){
    /* Start transmission */
    SPDR = cData;
    /* Wait for transmission complete */
    asm volatile("nop");
    while(!(SPSR & (1<<SPIF)));
    /* Return anything recieved */
    return SPDR;
}

/* Send a string, for the lulz  */
void vSPI_MasterTransmitString(char *s){
    while (*s != 0x00){
        ui8SPI_MasterTransmit(*s);
        s++;
    }
}