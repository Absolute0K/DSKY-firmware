/**
 * @file atomic-i2c.h
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief Wrapper for atomic i2c routines
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ATOMIC_I2C_H_
#define ATOMIC_I2C_H_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include "driver/i2c.h"

esp_err_t atomic_i2c_begin();

esp_err_t atomic_i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address,
                                            const uint8_t* write_buffer, size_t write_size,
                                            TickType_t ticks_to_wait);

esp_err_t atomic_i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address,
                                            uint8_t* read_buffer, size_t read_size,
                                            TickType_t ticks_to_wait);
                                            
#endif
