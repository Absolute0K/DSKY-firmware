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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define IS31FL3730_DEFAULT_ADDR (0x60)
#define NUM_TOTAL_DRIVERS       (13)
#define NUM_DISP_PER_DRIVER     (2)
#define NUM_DRIVER_PER_PORT     (4)
#define NUM_TOTAL_DISPS         (25)

#define DISP_REG0_0             (0)
#define DISP_REG0_1             (1)
#define DISP_REG0_2             (2)
#define DISP_REG0_3             (3)
#define DISP_REG0_4             (4)
#define DISP_REG0_5             (5)
#define DISP_REG1_0             (6)
#define DISP_REG1_1             (7)
#define DISP_REG1_2             (8)
#define DISP_REG1_3             (9)
#define DISP_REG1_4             (10)
#define DISP_REG1_5             (11)
#define DISP_REG2_0             (12)
#define DISP_REG2_1             (13)
#define DISP_REG2_2             (14)
#define DISP_REG2_3             (15)
#define DISP_REG2_4             (16)
#define DISP_REG2_5             (17)
#define DISP_NOUN_0             (18)
#define DISP_NOUN_1             (19)
#define DISP_VERB_0             (20)
#define DISP_VERB_1             (21)
#define DISP_PROG_0             (22)
#define DISP_PROG_1             (23)
#define DISP_COMPACTY           (24)
#define DISP_LAMP               (25)

#define LAMP_UPLINK_ACTY        (0)
#define LAMP_TEMP               (1)
#define LAMP_HOLD               (2)
#define LAMP_PROG               (3)
#define LAMP_FREE               (4)
#define LAMP_RESTART            (5)
#define LAMP_NO_ATT             (6)
#define LAMP_TRACKER            (7)
#define LAMP_STBY               (8)
#define LAMP_EMPTY              (9)
#define LAMP_KEY_REL            (10)
#define LAMP_OPR_ERROR          (11)

typedef struct display_packet
{
    uint32_t id;
    uint8_t digit;
} display_packet_t;

esp_err_t ltp305g_begin(uint8_t brightness);

esp_err_t ltp305g_set_current_bright(uint8_t current, uint8_t brightness, 
                                     uint8_t start, uint8_t count);

esp_err_t ltp305g_update(uint8_t driver_id);

esp_err_t ltp305g_write_lamp(uint32_t index, uint8_t value);

esp_err_t ltp305g_write_lamps(char* str_lamps);

esp_err_t ltp305g_write_digit(uint8_t display_id, uint8_t ch);

esp_err_t ltp305g_clear(uint8_t start, uint8_t count);

esp_err_t ltp305g_puts(char* buf, uint8_t start, uint8_t count);

/* Look up tables */
extern const uint8_t driver_addr_LUT[4];
extern const uint8_t ltp_305g_LUT[485];


#endif
