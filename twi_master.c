/************************************************************************/
// File:			twi_master.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Communication protocol to peripherals, i2c master
/************************************************************************/ 

#include <avr/io.h>
#include <util/twi.h>


#include "twi_master.h"
#include "defines.h"

#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void vTWI_init(void){
    // Set TWI Bit Rate Register
    TWBR = (uint8_t)TWBR_val;
}

uint8_t ui8TWI_start(uint8_t address){
    // reset TWI control register
    TWCR = 0;
    // transmit START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    
    // check if the start condition was successfully transmitted
    if((TWSR & 0xF8) != TW_START){ return 1; }
    
    // load slave address into data register
    TWDR = address;
    // start transmission of address
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    
    // check if the device has acknowledged the READ / WRITE mode
    uint8_t twst = TW_STATUS & 0xF8;
    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
    
    return 0;
}

uint8_t ui8TWI_write(uint8_t data){
    // load data into data register
    TWDR = data;
    // start transmission of data
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    
    if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
    
    return 0;
}

uint8_t ui8TWI_read_ack(void){
    // start TWI module and acknowledge data after reception
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    // return received data from TWDR
    return TWDR;
}

uint8_t ui8TWI_read_nack(void){
    // start receiving without acknowledging reception
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait for end of transmission
    while( !(TWCR & (1<<TWINT)) );
    // return received data from TWDR
    return TWDR;
}

uint8_t ui8TWI_transmit(uint8_t address, uint8_t* data, uint16_t length){
    
    if (ui8TWI_start(address | TWI_WRITE)) return 1;
    
    for (uint16_t i = 0; i < length; i++)
    {
        if (ui8TWI_write(data[i])) return 1;
    }
    
    vTWI_stop();
    
    return 0;
}

uint8_t ui8TWI_receive(uint8_t address, uint8_t* data, uint16_t length){
    
    if (ui8TWI_start(address | TWI_READ)) return 1;
    
    for (uint16_t i = 0; i < (length-1); i++){
        data[i] = ui8TWI_read_ack();
    }
    data[(length-1)] = ui8TWI_read_nack();
    
    vTWI_stop();
    
    return 0;
}

uint8_t ui8TWI_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length){
    
    if (ui8TWI_start(devaddr | 0x00)) return 1;

    ui8TWI_write(regaddr);

    for (uint16_t i = 0; i < length; i++){
        if (ui8TWI_write(data[i])) return 1;
    }

    vTWI_stop();

    return 0;
}

uint8_t ui8TWI_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length){
    if (ui8TWI_start(devaddr)) return 1;

    ui8TWI_write(regaddr);

    if (ui8TWI_start(devaddr | 0x01)) return 1;

    for (uint16_t i = 0; i < (length-1); i++){
        data[i] = ui8TWI_read_ack();
    }
    data[(length-1)] = ui8TWI_read_nack();

    vTWI_stop();

    return 0;
}

void vTWI_stop(void){
    // transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}