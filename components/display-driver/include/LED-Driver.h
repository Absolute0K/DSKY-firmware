/**
 * @file LED-Driver.h
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief LTP-305G/IS31FL3730 I2C Driver for ESP32 Header file (GNU GPL).
 * Inspired by https://github.com/mlukasek/microdotphat_library
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LED_DRIVER_H_
#define LED_DRIVER_H_

#include <stdint.h>
#include "driver/i2c.h"

#define IS31FL3730_DEFAULT_ADDR (0x60)
#define NUM_TOTAL_DRIVERS       (13)
#define NUM_DISP_PER_DRIVER     (2)
#define NUM_DRIVER_PER_PORT     (4)
#define NUM_TOTAL_DISPS         (25)


esp_err_t ltp305g_begin(uint8_t brightness);

esp_err_t ltp305g_set_total_brightness(uint8_t current, uint8_t brightness);

esp_err_t ltp305g_update(uint8_t driver_id);

esp_err_t ltp305g_write_digit(uint8_t display_id, uint8_t ch);

esp_err_t ltp305g_clear();

esp_err_t ltp305g_puts(char* buf);

/* Look up tables */
extern const uint8_t driver_addr_LUT[4];
extern const uint8_t ltp_305g_LUT[485];


#endif
