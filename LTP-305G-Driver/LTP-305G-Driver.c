/**
 * @file LTP-305G-Driver.c
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief LTP-305G/IS31FL3730 I2C Driver for ESP32 Source file (GNU GPL).
 * Inspired by https://github.com/mlukasek/microdotphat_library
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "LTP-305G-Driver.h"
#include "PCA9548A.h"
#include "constants.h"
#include "esp_log.h"

#define LTP305G_GET_DRIVER_MUX_PORT(x)     (x / NUM_DRIVER_PER_PORT)
#define LTP305G_GET_DRIVER_ID(x)           (x % NUM_DRIVER_PER_PORT)
#define LTP305G_GET_DRIVER_ID_FROM_DISP(x) (x / NUM_DISP_PER_DRIVER)

static uint8_t buffer[NUM_TOTAL_DISPS];


/**
 * @brief Set the PCA9548A I2C MUX ports correctly to make sure the
 * correct I2C driver is being communicated. NOTE: Static local function.
 * @param display_index 
 * @return esp_err_t 
 */
static esp_err_t set_correct_display_mux(uint8_t display_index)
{
    // Keep track of current port to see if any changes are needed
    static uint8_t current_port = 0;
    if (display_index >= NUM_TOTAL_DISPS) return ESP_ERR_NOT_SUPPORTED;

    // Calculate the new required port for the I2C mux
    uint8_t new_port = LTP305G_GET_DRIVER_MUX_PORT(display_index);
    // Switch if necessary
    if (current_port != new_port) return pca9548_set_port(new_port);
    else                          return ESP_OK;
}


static esp_err_t is31fl3730_write(uint8_t driver_id, uint8_t addr, uint8_t val)
{
    // Send reset I2C commands
    esp_err_t ret = ESP_OK;
    // Set correct port
    if (set_correct_display_mux(driver_id) != ESP_OK)
    {
        ESP_LOGE("PCA9548A_I2C", "Error Mux Port Switch for Driver ID %d", driver_id);
        return ESP_ERR_INVALID_STATE;
    }
    // Send packets
    uint8_t packet[2] = {val, addr};
    return i2c_master_write_to_device(I2C_MASTER_NUM, 
                                      driver_addr_LUT[LTP305G_GET_DRIVER_ID(driver_id)], 
                                      packet, sizeof(packet), 
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


/**
 * @brief Converts column data to rows to drive 2nd display.
 * Full credit: https://github.com/mlukasek/microdotphat_library
 * @param digit 
 * @param row 
 */
static void col2RowConv(uint8_t digit, uint8_t* row)
{
    uint8_t rowindex = 0;
    for (rowindex = 0; rowindex < 7; rowindex++) {
        row[rowindex] = 0;
        row[rowindex] =  (bitRead(ltp_305g_LUT[digit * 5 + 0], rowindex) * 1)
                                        + (bitRead(ltp_305g_LUT[digit * 5 + 1], rowindex) * 2)
                                        + (bitRead(ltp_305g_LUT[digit * 5 + 2], rowindex) * 4)
                                        + (bitRead(ltp_305g_LUT[digit * 5 + 3], rowindex) * 8)
                                        + (bitRead(ltp_305g_LUT[digit * 5 + 4], rowindex) * 16);
    }
}


void ltp305g_begin(uint8_t brightness)
{
    // TODO: Remove the delay?
    vTaskDelay(15 / portTICK_RATE_MS);
    esp_err_t ret = ESP_OK;

    // Reset the buffer to empty display " " or SPACE
    for (int i = 0; i < NUM_TOTAL_DISPS; i++) buffer[i] = ' ';

    for (int i = 0; i < NUM_TOTAL_DRIVERS; i++)
    {
        // Send reset I2C commands
        if (is31fl3730_write(i, 0xFF, 0x00) != ESP_OK)
        {
            ESP_LOGE("LTP-305G/IS31FL3730", "Error Reset Driver ID %d", i);
            return ret;
        }
        // Send 8x8 configuration I2C commands
        if (is31fl3730_write(i, 0x00, 0x18) != ESP_OK)
        {
            ESP_LOGE("LTP-305G/IS31FL3730", "Error Configuration Driver ID %d", i);
            return ret;
        }
    }

    // Set brightness to 40mA, brightness
    ltp305g_set_total_brightness(0x00, brightness);
    
    // TODO: Remove the delay?
    vTaskDelay(15 / portTICK_RATE_MS);

    return ESP_OK;
}


esp_err_t ltp305g_set_total_brightness(uint8_t current, uint8_t brightness)
{
    esp_err_t ret = ESP_OK;

    for (int i = 0; i < NUM_TOTAL_DRIVERS; i++)
    {
        // Send current data
        if (is31fl3730_write(i, 0x0D, current) != ESP_OK)
        {
            ESP_LOGE("LTP-305G/IS31FL3730", "Error Current Update Driver ID %d", i);
            return ret;
        }
        // Send PWM data
        if (is31fl3730_write(i, 0x19, brightness) != ESP_OK)
        {
            ESP_LOGE("LTP-305G/IS31FL3730", "Error PWM Update Driver ID %d", i);
            return ret;
        }
    }

    return ESP_OK;
}


esp_err_t ltp305g_update(uint8_t driver_id)
{
    if (is31fl3730_write(driver_id, 0x0c, 0xff) != ESP_OK)
    {
        ESP_LOGE("LTP-305G/IS31FL3730", "Error Driver Index %d Update", driver_id);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}


esp_err_t ltp305g_write_digit(uint8_t display_id, uint8_t ch)
{    
    if (display_id > NUM_TOTAL_DISPS)
    {
        ESP_LOGE("LTP-305G/IS31FL3730", "Incorrect Display ID %d", display_id);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    uint8_t driver_id = LTP305G_GET_DRIVER_ID_FROM_DISP(display_id);
    uint8_t packets[8] = {0x00};
    uint8_t row[7];
    uint8_t x = (ch & 0x7F) - 32;
    buffer[display_id - 1] = ch;

    if (display_id % 2)
    {
        packets[0] = 0x0E;
        for(int y = 0; y < 5; y++) packets[y + 1] = ltp_305g_LUT[x * 5 + y];
        packets[7] = (ch & 0x80) ? 0x40 : 0x00;
    }
    else
    {
        packets[0] = 0x01;
        col2RowConv(x, row);
        for(int rx=0; rx < 7; rx++)
            packets[rx + 1] = (ch & 0x80) && rx == 6 ? row[rx] | 0x80 : row[rx];
    }

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 
                                     driver_addr_LUT[LTP305G_GET_DRIVER_ID(driver_id)], 
                                     packets, sizeof(packets), 
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("LTP-305G/IS31FL3730", "Error Writing Display ID %d", display_id);
        return ESP_ERR_INVALID_ARG;
    }

    return ltp305g_update(driver_id);
}

void clear()
{
    for (int i = 0; i < NUM_TOTAL_DISPS; i++) ltp305g_write_digit(i, ' ');
}

esp_err_t ltp305g_write_digit(char* buf)
{
    for (int i = 0; i < NUM_TOTAL_DISPS; i++)
        if (i < strlen(buf)) ltp305g_write_digit(i, buf[i]);
        else                 ltp305g_write_digit(i, ' ');
}