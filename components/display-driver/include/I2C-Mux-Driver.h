/**
 * @file I2C-Mux-Driver.h
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief PCA9548A I2C Driver for ESP32 Header file (MIT License).
 * Inspired by: https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef PCA9548A_H_
#define PCA9548A_H_

#include <stdint.h>
#include "driver/i2c.h"

#define QWIIC_MUX_DEFAULT_ADDRESS 0x70
#define QWIIC_MUX_MAX_ADDRESS     (QWIIC_MUX_DEFAULT_ADDRESS + 0x7)

/**
 * @brief Check communication and initialize device with given I2C pins.
 * Returns true if device responded correctly. All ports will be disabled.
 * Valid addresses for the TCA9548 are 0x70 to 0x77.
 * @param device_addr 
 * @return esp_err_t 
 */
esp_err_t pca9548_begin(uint8_t i2c_addr, int pin_scl, int pin_sda, uint32_t clk_speed);

/**
 * @brief Returns true if device acks at the I2C address.
 * Tests for device ack to I2C address. Then tests if device behaves 
 * as we expect. Leaves with all ports disabled
 * @return esp_err_t 
 */
esp_err_t pca9548_is_connected();

/**
 * @brief Enable a single port. All other ports disabled.
 * If port number if out of range, disable all ports.
 * @param port_number 
 * @return esp_err_t 
 */
esp_err_t pca9548_set_port(uint8_t port_number);

/**
 * @brief Overwrite port register with all 8 bits.
 * Allows multiple bit writing in one call.
 * Overwrites any other bits.
 * This allows us to enable/disable multiple ports at same time.
 * @param port_bits 
 * @return esp_err_t 
 */
esp_err_t pca9548_set_port_state(uint8_t port_bits);


/**
 * @brief Returns the current 8-bit wide state in *state.
 * May have multiple bits set in 8-bit field. Returns error.
 * @param state 
 * @return esp_err_t 
 */
esp_err_t pca9548_get_port_state(uint8_t *state);

#endif