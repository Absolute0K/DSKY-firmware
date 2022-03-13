#include <stdio.h>
#include "keypad-component.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/timers.h"

static int id = 1;
static uint8_t matrix[NUM_ROWS][NUM_COLS];

static void keypad_timer_callback(TimerHandle_t xTimer)
{
    static uint64_t counter = 0;
    ESP_LOGI("KEYPAD", "Tick %llu", counter);

    for (int row = 0; row < NUM_ROWS; row++)
    {
        // Set HIGH OUTPUT to row (row)

        for (int col = 0; col < NUM_COLS; col++)
        {
            // Set INPUT PULLDOWN to col (col)
            // Then check if the pin is high
            // Set INPUT NO PULL RESISTORS
        }
        // Set INPUT NO PULL RESISTORS to row
    }
}

esp_err_t keypad_begin(uint32_t scan_interval_ms)
{
    TimerHandle_t t = xTimerCreate("keypad_scan", pdMS_TO_TICKS(scan_interval_ms), 
                                   pdTRUE, (void*) id, &keypad_timer_callback);
    
    if (xTimerStart(t, 10) != pdPASS) return ESP_ERR_NOT_FINISHED;

    for (int row = 0; row < NUM_ROWS; row++)
        for (int col = 0; col < NUM_COLS; col++) matrix[row][col] = 0;

    return ESP_OK;
}