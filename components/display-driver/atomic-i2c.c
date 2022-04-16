#include "atomic-i2c.h"

/* Semaphore to prevent race conditions using i2c bus */
static SemaphoreHandle_t xSemaphore_i2c_0 = NULL;

esp_err_t atomic_i2c_begin()
{
    xSemaphore_i2c_0 = xSemaphoreCreateMutex();
    return (xSemaphore_i2c_0 != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t atomic_i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address,
                                            const uint8_t* write_buffer, size_t write_size,
                                            TickType_t ticks_to_wait)
{
    if (xSemaphore_i2c_0 == NULL) return ESP_ERR_NO_MEM;

    do {} while (xSemaphoreTake(xSemaphore_i2c_0, portMAX_DELAY) != pdPASS);
    esp_err_t ret;
    ret = i2c_master_write_to_device(i2c_num, device_address, write_buffer, write_size, ticks_to_wait);
    xSemaphoreGive(xSemaphore_i2c_0);
    return ret;
}

esp_err_t atomic_i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address,
                                            uint8_t* read_buffer, size_t read_size,
                                            TickType_t ticks_to_wait)
{
    if (xSemaphore_i2c_0 == NULL) return ESP_ERR_NO_MEM;

    do {} while (xSemaphoreTake(xSemaphore_i2c_0, portMAX_DELAY) != pdPASS);
    esp_err_t ret;
    ret = i2c_master_read_from_device(i2c_num, device_address, read_buffer, read_size, ticks_to_wait);
    xSemaphoreGive(xSemaphore_i2c_0);
    return ret;
}