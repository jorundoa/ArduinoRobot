/************************************************************************/
// File:			usart.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver USART. Protected printf with mutex.
/************************************************************************/

#ifndef USART_H_
#define USART_H_

/************************************************************************/
//Initialize USART driver, note that RXD0/TXD0 (PD0/PD1) is used
// Note that the nRF51 dongle is limited to send 20 characters
// in each package
/************************************************************************/
void vUSART_init();

/* Send one char */
void vUSART_sendC(char c);

/* Send array of chars */
void vUSART_sendS(char *s);

/* Send predefined handshake */
void vUSART_sendHandshake();

/* Send a update to the server */
void vUSART_sendUpdate(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm);

#endif /* USART_H_ */

