/*
 * Nextion.h
 *
 *  Created on: 26-Aug-2022
 *      Author: Muhammad Asghar
 */

#ifndef MAIN_NEXTION_H_
#define MAIN_NEXTION_H_

#define UART UART_NUM_2

#define TXD_PIN 17
#define RXD_PIN 16
static const int RX_BUF_SIZE = 1024;



void uart_init(void);
void Display_Nextion(char* cmd , char* data);




#endif /* MAIN_NEXTION_H_ */
