#include <stdio.h>
#include "keypad-component.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

static int id = 1;
static uint64_t row_LUT[NUM_ROWS] = {17, 16, 15};
static uint64_t col_LUT[NUM_COLS] = {0, 2, 4, 5, 12, 13, 14};
static uint8_t char_LUT[NUM_ROWS][NUM_COLS] = {
    {'+', '7', '8', '9', 'C', 'N', 'R'},
    {'-', '4', '5', '6', 'P', 'V', ' '},
    {'0', '1', '2', '3', 'K', 'E', ' '},
};

static void keypad_timer_callback(TimerHandle_t xTimer)
{
    static uint64_t counter = 0;

    for (int row = 0; row < NUM_ROWS; row++)
    {
        // Set HIGH OUTPUT to row (row)
        gpio_config_t io_conf = {};
        io_conf.intr_type    = GPIO_INTR_DISABLE;
        io_conf.mode         = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = 1ULL << row_LUT[row];
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en   = 0;
        gpio_config(&io_conf);
        gpio_set_level(row_LUT[row], 1);

        for (int col = 0; col < NUM_COLS; col++)
        {
            // Set INPUT PULLDOWN to col (col)
            io_conf.intr_type    = GPIO_INTR_DISABLE;
            io_conf.mode         = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = 1ULL << col_LUT[col];
            io_conf.pull_down_en = 1;
            io_conf.pull_up_en   = 0;
            gpio_config(&io_conf);

            if (gpio_get_level(col_LUT[col]))
            {
                // Send to queue if keystroke is detected
                xQueueSendToBack(xQueue_keystrokes, &(char_LUT[row][col]), 1000);
            }

            // Set INPUT NO PULL RESISTORS
            io_conf.intr_type    = GPIO_INTR_DISABLE;
            io_conf.mode         = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = 1ULL << col_LUT[col];
            io_conf.pull_down_en = 0;
            io_conf.pull_up_en   = 0;
            gpio_config(&io_conf);
        }
        // Set INPUT NO PULL RESISTORS to row
        io_conf.intr_type    = GPIO_INTR_DISABLE;
        io_conf.mode         = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << row_LUT[row];
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en   = 0;
        gpio_config(&io_conf);
    }
}

esp_err_t keypad_begin(uint32_t scan_interval_ms)
{
    TimerHandle_t t = xTimerCreate("keypad_scan", pdMS_TO_TICKS(scan_interval_ms), 
                                   pdTRUE, (void*) id, &keypad_timer_callback);
    
    if (xTimerStart(t, 10) != pdPASS) return ESP_ERR_NOT_FINISHED;

    // Set up queue
    xQueue_keystrokes = xQueueCreate(NUM_QUEUE, sizeof(uint8_t));
    if (xQueue_keystrokes == NULL) return ESP_ERR_NO_MEM;


    return ESP_OK;
}