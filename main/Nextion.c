/*
 * Nextion.c
 *
 *  Created on: 26-Aug-2022
 *      Author: Muhammad Asghar
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Nextion.h"
#include "driver/uart.h"

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}



void Display_Nextion(char* cmd , char* data)
{
	char buffer[50];
	uint8_t terminator[3] = {0xff , 0xff , 0xff};
	sprintf (buffer,  "%s.txt=\"%s\"", cmd , data);
	uart_write_bytes(UART, (uint8_t*)buffer, strlen(buffer));
	uart_write_bytes(UART, &terminator, 3);
}
