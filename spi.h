/************************************************************************/
// File:			spi.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Communication protocol to peripherals, SPI
/************************************************************************/


#ifndef SPI_H_
#define SPI_H_

void vSPI_MasterInit();

uint8_t ui8SPI_MasterTransmit(char cData);
/* Send a string  */
void vSPI_MasterTransmitString(char *s);

#endif /* SPI_H_ */