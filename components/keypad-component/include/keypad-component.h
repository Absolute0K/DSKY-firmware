/**
 * @file keypad-component.h
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief Very simple FreeRTOS implementation of DSKY keypad driver
 * @version 0.1
 * @date 2022-03-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef KEYPAD_COMP_H
#define KEYPAD_COMP_H

#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define NUM_COLS  (7)
#define NUM_ROWS  (3)
#define NUM_QUEUE (5)

esp_err_t keypad_begin(uint32_t scan_interval_ms);

// Global queue for keystrokes
QueueHandle_t xQueue_keystrokes;

#endif

