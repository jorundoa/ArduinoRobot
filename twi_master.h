/************************************************************************/
// File:			twi_master.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Communication protocol to peripherals, i2c master
/************************************************************************/
#include <stdint.h>

#ifndef TWI_MASTER_H_
#define TWI_MASTER_H_

#define TWI_READ 0x01
#define TWI_WRITE 0x00

void vTWI_init(void);

uint8_t ui8TWI_start(uint8_t address);

uint8_t ui8TWI_write(uint8_t data);

uint8_t ui8TWI_read_ack(void);

uint8_t ui8TWI_read_nack(void);

uint8_t ui8TWI_transmit(uint8_t address, uint8_t* data, uint16_t length);

uint8_t ui8TWI_receive(uint8_t address, uint8_t* data, uint16_t length);

uint8_t ui8TWI_writeReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);

uint8_t ui8TWI_readReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);

void vTWI_stop(void);



#endif /* TWI_MASTER_H_ */