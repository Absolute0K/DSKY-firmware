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

#define NUM_ROWS (7)
#define NUM_COLS (3)

esp_err_t keypad_begin(uint32_t scan_interval_ms);

#endif

