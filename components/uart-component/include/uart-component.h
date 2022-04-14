/**
 * @file uart-component.h
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief Very simple FreeRTOS implementation of DSKY UART driver
 * @version 0.1
 * @date 2022-03-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UART_COMP_H
#define UART_COMP_H

#include <stdint.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define UART_BUF_SIZE           (1024)
#define UART_PKT_SIZE           (1+3*6+2+12+4*6+1)
const char char_SOP = '<';
const char char_EOP = '>';

QueueHandle_t xQueue_packets;

typedef struct uart_pkt
{
    char data[UART_PKT_SIZE+1];
} uart_pkt_t;

esp_err_t uart_begin();

// Global queue for keystrokes
QueueHandle_t xQueue_keystrokes;

#endif

