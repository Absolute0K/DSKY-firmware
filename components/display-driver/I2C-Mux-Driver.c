/**
 * @file I2C-Mux-Driver.c
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief PCA9548A I2C Driver for ESP32 Source (MIT License).
 *  Inspired by: https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "I2C-Mux-Driver.h"
#include "constants.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "atomic-i2c.h"

// Default unshifted 7-bit address
static uint8_t dev_addr = QWIIC_MUX_DEFAULT_ADDRESS;

esp_err_t pca9548_begin(uint8_t i2c_addr, int pin_scl, int pin_sda, uint32_t clk_speed)
{
    // Check address
    if (QWIIC_MUX_DEFAULT_ADDRESS > i2c_addr && i2c_addr > QWIIC_MUX_MAX_ADDRESS)
    {
        ESP_LOGE("PCA9548A_I2C", "ADDRESS IS OUT OF BOUNDS: %X", i2c_addr);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Set up I2C driver
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = clk_speed
    };

    dev_addr = i2c_addr;

    // Install the driver
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                    I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    // Check if the device ack's over I2C
    return pca9548_is_connected();
}


esp_err_t pca9548_is_connected()
{
    //Write to device, expect a return
    pca9548_set_port_state(0xA4);
    //Set port register to a known value
    uint8_t response[1];
    if (pca9548_get_port_state(response) != ESP_OK) return ESP_ERR_INVALID_RESPONSE;
    pca9548_set_port_state(0x00);
    //Disable all ports
    return (response[0] == 0xA4) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}


esp_err_t pca9548_set_port(uint8_t portNumber)
{
    uint8_t packet[1] = {0};
    if (portNumber > 7)
    {
        ESP_LOGW("PCA9548A_I2C", "PORT IS OUT OF BOUNDS: %X", portNumber);
        packet[0] = 0;
    }
    else packet[0] = 1 << portNumber;

    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, 
                                      packet, sizeof(packet), 
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


esp_err_t pca9548_set_port_state(uint8_t portBits)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, 
                                      &portBits, 1, 
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t pca9548_get_port_state(uint8_t *state)
{
    return i2c_master_read_from_device(I2C_MASTER_NUM, dev_addr, 
                                       state, 1, 
                                       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}