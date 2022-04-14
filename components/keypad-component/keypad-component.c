#include <stdio.h>
#include "keypad-component.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "LED-Driver.h"

#define PRI_KEYSTROKES (6)

static int id = 1;
static uint64_t row_LUT[NUM_ROWS] = {17, 16, 15};
static uint64_t col_LUT[NUM_COLS] = {0, 2, 4, 5, 12, 13, 14};
static uint8_t char_LUT[NUM_ROWS][NUM_COLS] = {
    {'+', '7', '8', '9', 'C', 'N', 'R'},
    {'-', '4', '5', '6', 'P', 'V', ' '},
    {'0', '1', '2', '3', 'K', 'E', ' '},
};
static enum states_VN {WAIT_VERB, INPUT_VERB, WAIT_NOUN, INPUT_NOUN, ENTER};

static void v_receiver_keystrokes(void *pvParameters)
{
    enum states_VN state = WAIT_VERB;
    uint32_t counter = 0;
    pair_VN_t pair_vn = {0};
    pair_VN_t pair_vn_prev = {0};

    for (;;)
    {
        char recv_key;
        if (xQueueReceive(xQueue_keystrokes, &recv_key, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI("KEYPAD", "Pressed %c", recv_key);

            /* Handle CLEAR */
            if (recv_key == 'C') state = WAIT_VERB;
            else if (recv_key == 'R') state = WAIT_VERB; // TODO: IMPLEMENT RESET

            switch (state)
            {
                case WAIT_VERB:
                    /* Clear VERB/NOUN and highlight displays if VERB is pressed */
                    pair_vn.verb = 0;
                    pair_vn.noun = 0;
                    if (recv_key == 'V')
                    {
                        ltp305g_write_digit(DISP_VERB_0, ' ', 1);
                        ltp305g_write_digit(DISP_VERB_1, ' ', 1);
                        ltp305g_write_digit(DISP_NOUN_0, ' ', 1);
                        ltp305g_write_digit(DISP_NOUN_1, ' ', 1);
                        state = INPUT_VERB;
                    }
                    else
                    {
                        // OPERR
                    }
                    break;
                case INPUT_VERB:
                    counter = (counter + 1) % 2;
                    recv_key -= '0';
                    if (recv_key > 9)
                    {
                        // TODO: LIGHT UP OPERR
                        state = WAIT_VERB;
                        break;
                    }
                    ltp305g_write_digit(DISP_VERB_1 - counter, recv_key + '0', 0);
                    pair_vn.verb = pair_vn.verb * 10 + recv_key;
                    state = (counter > 1) ? INPUT_VERB : WAIT_NOUN;
                    break;
                case WAIT_NOUN:
                    if (recv_key == 'V')
                        state = INPUT_VERB;
                    else
                    {
                        state = WAIT_VERB;
                        // OPERR
                    }
                    break;
                case INPUT_NOUN:
                    counter = (counter + 1) % 2;
                    recv_key -= '0';
                    if (recv_key > 9)
                    {
                        // TODO: LIGHT UP OPERR
                        state = WAIT_VERB;
                        break;
                    }
                    ltp305g_write_digit(DISP_NOUN_1 - counter, recv_key + '0', 0);
                    pair_vn.noun = pair_vn.noun * 10 + recv_key;
                    state = (counter > 1) ? INPUT_NOUN : ENTER;
                    break;
                case ENTER:
                    if (recv_key == 'E')
                    {
                        // Maybe a task for COMPACTY and LAMPS?
                        xQueueSendToBack(xQueue_VN, &pair_vn, 1000);
                        state = WAIT_VERB;
                        pair_vn_prev = pair_vn;
                    }
                    else
                    {
                        // OPERR
                        state = WAIT_VERB;
                    }
                    break;
                default:
                    state = WAIT_VERB;
                    // OPERR
                    break;
            }
        }
    }
}

static void keypad_timer_callback(TimerHandle_t xTimer)
{
    static uint64_t counter = 0;
    static uint8_t char_status[NUM_ROWS][NUM_COLS] = {0};

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
                if (char_status[row][col] == 0)
                {
                    // Send to queue if keystroke is detected
                    xQueueSendToBack(xQueue_keystrokes, &(char_LUT[row][col]), 1000);
                    char_status[row][col] = 1;
                }
            }
            else char_status[row][col] = 0;

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
    xQueue_VN = xQueueCreate(NUM_QUEUE, sizeof(pair_VN_t));
    if (xQueue_VN == NULL) return ESP_ERR_NO_MEM;

    xTaskCreate(v_receiver_keystrokes, "KeystrokesHandler", 2048, 
                (void*) NULL, PRI_KEYSTROKES, NULL);

    return ESP_OK;
}