/**
 * @file LED-Driver.c
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief LTP-305G/IS31FL3730 I2C Driver for ESP32 Source file (GNU GPL).
 * Inspired by https://github.com/mlukasek/microdotphat_library
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "LED-Driver.h"
#include "I2C-Mux-Driver.h"
#include "constants.h"
#include "esp_log.h"
#include "atomic-i2c.h"
#include <string.h>

#define PRI_DISP_EVENT                     (4)
#define LTP305G_GET_DRIVER_MUX_PORT(x)     (x / NUM_DRIVER_PER_PORT)
#define LTP305G_GET_DRIVER_ID(x)           (x % NUM_DRIVER_PER_PORT)
#define LTP305G_GET_DRIVER_ID_FROM_DISP(x) (x / NUM_DISP_PER_DRIVER)
#define bitRead(value, bit)                (((value) >> (bit)) & 0x01)

static uint8_t buffer[NUM_TOTAL_DISPS];
static SemaphoreHandle_t xSemaphore_displays;
static char state_lamps[12] = "000000000000";
/**
 * @brief Set the PCA9548A I2C MUX ports correctly to make sure the
 * correct I2C driver is being communicated. NOTE: Static local function.
 * @param display_index 
 * @return esp_err_t 
 */
static esp_err_t set_correct_display_mux(uint8_t display_index)
{
    // Keep track of current port to see if any changes are needed
    static uint8_t current_port = -1;
    if (display_index >= NUM_TOTAL_DISPS) return ESP_ERR_NOT_SUPPORTED;

    // Calculate the new required port for the I2C mux
    uint8_t new_port = LTP305G_GET_DRIVER_MUX_PORT(display_index);

    // Switch if necessary
    if (current_port != new_port)
    {
        ESP_LOGV("PCA9548A", "Switching from Port %d to Port %d", current_port, new_port);
        current_port = new_port;
        return pca9548_set_port(new_port);
    }
    return ESP_OK;
}


static esp_err_t is31fl3730_write(uint8_t driver_id, uint8_t addr, uint8_t val)
{
    // Send reset I2C commands
    // Set correct port
    if (set_correct_display_mux(driver_id) != ESP_OK)
    {
        ESP_LOGE("PCA9548A", "Error Mux Port Switch for Driver ID %d", driver_id);
        return ESP_ERR_INVALID_STATE;
    }
    // Send packets
    uint8_t packet[2] = {addr, val};
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


esp_err_t ltp305g_begin(uint8_t brightness)
{
    // TODO: Remove the delay?
    vTaskDelay(15 / portTICK_PERIOD_MS);
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
        ESP_LOGV("LTP-305G/IS31FL3730", "Successfully Initialized Driver ID %d", i);
    }

    // Set brightness to 40mA, brightness
    ltp305g_set_current_bright(0x00, brightness, 0, NUM_TOTAL_DRIVERS);
    
    // TODO: Remove the delay?
    vTaskDelay(15 / portTICK_PERIOD_MS);

    // Create  and start task
    xSemaphore_displays = xSemaphoreCreateMutex();
    if (xSemaphore_displays == NULL) return ESP_ERR_NO_MEM;

    return ESP_OK;
}


esp_err_t ltp305g_set_current_bright(uint8_t current, uint8_t brightness, uint8_t start, uint8_t count)
{
    esp_err_t ret = ESP_OK;

    for (int i = start; i < start + count; i++)
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

esp_err_t ltp305g_write_lamp(uint32_t index, uint8_t value)
{
    state_lamps[index] = value + '0';
    return ltp305g_write_lamps(state_lamps);
}

esp_err_t ltp305g_write_lamps(char* str_lamps)
{
    esp_err_t ret = ESP_OK;
    uint8_t display_id = NUM_TOTAL_DISPS;
    uint8_t driver_id = LTP305G_GET_DRIVER_ID_FROM_DISP(display_id);

    xSemaphoreTake(xSemaphore_displays, portMAX_DELAY);

    if (set_correct_display_mux(driver_id) != ESP_OK)
    {
        ESP_LOGE("PCA9548A", "Error Mux Port Switch for Driver ID %d", driver_id);
        return ESP_ERR_INVALID_STATE;
    }
    // Address
    uint8_t packets[4] = {0};
    packets[0] = 0x01;
    for (int i = 0; i < 12; i++)
    {
        // Except for KEY_REL, OPR_ERROR (special). Can only be adjusted manually
        // Set lamp state
        state_lamps[i] = (i == LAMP_KEY_REL || i == LAMP_OPR_ERROR) ?
                          state_lamps[i] : str_lamps[i];
        // through write_lamp(..)
        packets[((11 - i)/4) + 1] |= (state_lamps[i] - '0') << (3 - (i % 4));
    }

    

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 
                                     driver_addr_LUT[LTP305G_GET_DRIVER_ID(driver_id)], 
                                     packets, 4, 
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("LTP-305G/IS31FL3730", "Error Writing Display ID %d", display_id);
        return ESP_ERR_INVALID_ARG;
    }

    ret = ltp305g_update(driver_id);
    xSemaphoreGive(xSemaphore_displays);

    return ret;
}

esp_err_t ltp305g_write_digit(uint8_t display_id, uint8_t ch)
{    
    if (display_id > NUM_TOTAL_DISPS + 1)
    {
        ESP_LOGE("LTP-305G/IS31FL3730", "Incorrect Display ID %d", display_id);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(xSemaphore_displays, portMAX_DELAY);

    esp_err_t ret = ESP_OK;
    uint8_t driver_id = LTP305G_GET_DRIVER_ID_FROM_DISP(display_id);
    uint8_t packets[9] = {0x00};
    uint8_t packet_size = 0;
    uint8_t row[7];
    uint8_t x = (ch & 0x7F) - 32;

    // Even number ID's mean 1st display in the pair, Odd means 2nd display
    if (display_id % 2) // Odd
    {
        // The Address, 7 row data
        packet_size = 8;
        packets[0] = 0x01;
        col2RowConv(x, row);
        for(int rx=0; rx < 7; rx++)
            packets[rx + 1] = (ch & 0x80) && rx == 6 ? row[rx] | 0x80 : row[rx];

    }
    else // Even
    {
        // The Address, 7 column data, 1 decimal data
        packet_size = 9;
        packets[0] = 0x0E;
        for(int y = 0; y < 5; y++) packets[y + 1] = ltp_305g_LUT[x * 5 + y];
        packets[8] = (ch & 0x80) ? 0x40 : 0x00;
    }

    if (set_correct_display_mux(driver_id) != ESP_OK)
    {
        ESP_LOGE("PCA9548A", "Error Mux Port Switch for Driver ID %d", driver_id);
        return ESP_ERR_INVALID_STATE;
    }

    // printf("Display %d, Driver %d, ID:%d, ADDR:%x LTP-305G/IS31FL3730 Packet(%d): %x %x %x %x %x %x %x %x %x\n", display_id, driver_id, LTP305G_GET_DRIVER_ID(driver_id), driver_addr_LUT[LTP305G_GET_DRIVER_ID(driver_id)], packet_size,
            //  packets[0], packets[1], packets[2], packets[3], packets[4], packets[5], packets[6], packets[7], packets[8]);

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, 
                                     driver_addr_LUT[LTP305G_GET_DRIVER_ID(driver_id)], 
                                     packets, packet_size, 
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("LTP-305G/IS31FL3730", "Error Writing Display ID %d", display_id);
        return ESP_ERR_INVALID_ARG;
    }

    ret = ltp305g_update(driver_id);

    xSemaphoreGive(xSemaphore_displays);

    return ret;
}

esp_err_t ltp305g_clear(uint8_t start, uint8_t count)
{
    esp_err_t ret = ESP_OK;
    if (start + count >= NUM_TOTAL_DISPS) return ESP_ERR_INVALID_ARG;
    for (int i = start; i < start + count; i++)
    {
        if (ltp305g_write_digit(i, ' ') != ESP_OK) ret = ESP_ERR_NOT_FINISHED;
    }
    return ret;
}

esp_err_t ltp305g_puts(char* buf, uint8_t start, uint8_t count)
{
    esp_err_t ret = ESP_OK;
    if (start + count >= NUM_TOTAL_DISPS) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < count; i++)
    {
        uint8_t digit = '0';
        if (i < strlen(buf)) digit = buf[count - i - 1];
        if (ltp305g_write_digit(i + start, digit) != ESP_OK) ret = ESP_ERR_NOT_FINISHED;
    }
    return ret;
}

