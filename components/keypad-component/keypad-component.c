#include <stdio.h>
#include "keypad-component.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "LED-Driver.h"

#define PRI_KEYSTROKES      (2)

static int id = 1;
static uint64_t row_LUT[NUM_ROWS] = {17, 16, 15};
static uint64_t col_LUT[NUM_COLS] = {0, 2, 4, 5, 12, 13, 14};
static uint8_t char_LUT[NUM_ROWS][NUM_COLS] = {
    {'+', '7', '8', '9', 'C', 'N', 'E'},
    {'-', '4', '5', '6', 'P', 'V', ' '},
    {'0', '1', '2', '3', 'K', 'R', ' '},
};
enum states_VN {WAIT_VERB, INPUT_VERB, WAIT_NOUN, INPUT_NOUN, ENTER};

void output_AGC_uart(data_in_t _data_in, pair_VN_t _pair_vn)
{
    char temp_buf[LEN_PKT_AGC_UART + 1] = {0};
    sprintf(temp_buf, "%c%05o%c%05o%c%05o%c%05o%02o%02o", 
    (_data_in.ATX   >= 0) ? '+' : '-', _data_in.ATX,
    (_data_in.invRB >= 0) ? '+' : '-', _data_in.invRB, 
    (_data_in.invRA >= 0) ? '+' : '-', _data_in.invRA,
    (_data_in.GM    >= 0) ? '+' : '-', _data_in.GM,
    _pair_vn.noun, _pair_vn.verb);
    // Reverse the array as the FPGA's implementation is weird
    for (int low = 0, high = LEN_PKT_AGC_UART - 1; low < high; low++, high--)
    {
        int temp = temp_buf[low];
        temp_buf[low] = temp_buf[high];
        temp_buf[high] = temp;
    }
    printf("<%s>\n", temp_buf);

}

static void v_receiver_keystrokes(void *pvParameters)
{
    enum states_VN state = WAIT_VERB;
    uint32_t counter = 0, update_ok = 1;
    pair_VN_t pair_vn_new = {0};
    for (;;)
    {
        char recv_key;
        if (xQueueReceive(xQueue_keystrokes, &recv_key, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI("KEYPAD", "Pressed %c", recv_key);

            /* Handle CLEAR */
            if (recv_key == 'C' || recv_key == 'R') 
            {
                ltp305g_clear(DISP_REG0_0, DISP_NOUN_0);
                ltp305g_clear(DISP_PROG_0, 2);
                ltp305g_write_lamps("000000000000");
                ltp305g_write_lamp(LAMP_STBY, 1);
                ltp305g_write_lamp(LAMP_OPR_ERROR, 0);
                ltp305g_write_digit(DISP_COMPACTY, ' ');
                state = WAIT_VERB;
            }

            switch (state)
            {
                case INPUT_VERB:
                    ESP_LOGI("KEYPAD", "STATE: INPUT_VERB %d : %c", counter, recv_key);
                    if ('0' <= recv_key && recv_key <= '9')
                    {
                        ltp305g_write_lamp(LAMP_OPR_ERROR, 0);
                        ltp305g_write_digit(DISP_VERB_1 - counter, recv_key);
                        pair_vn_new.verb = pair_vn_new.verb * 10 + recv_key - '0';
                        state = (counter == 1) ? WAIT_NOUN : INPUT_VERB;
                        counter = (counter + 1) % 2;
                    } else state = WAIT_VERB;
                    break;
                case WAIT_NOUN:
                    ESP_LOGI("KEYPAD", "STATE: WAIT_NOUN: %c", recv_key);
                    if (recv_key == 'N')
                    {
                        ltp305g_write_digit(DISP_NOUN_0, '_');
                        ltp305g_write_digit(DISP_NOUN_1, '_');
                        state = INPUT_NOUN;
                    }
                    else state = WAIT_VERB;
                    break;
                case INPUT_NOUN:
                    ESP_LOGI("KEYPAD", "STATE: INPUT_NOUN %d %c", counter, recv_key);
                    if ('0' <= recv_key && recv_key <= '9')
                    {
                        ltp305g_write_digit(DISP_NOUN_1 - counter, recv_key);
                        pair_vn_new.noun = pair_vn_new.noun * 10 + recv_key - '0';
                        state = (counter == 1) ? ENTER : INPUT_NOUN;
                        counter = (counter + 1) % 2;                        
                    } else state = WAIT_VERB;
                    break;
                case ENTER:
                    ESP_LOGI("KEYPAD", "STATE: ENTER: %c", recv_key);
                    if (recv_key == 'E')
                    {
                        ESP_LOGI("KEYPAD", "STATE: ENTER GO");
                        // Maybe a task for COMPACTY and LAMPS?
                        state = WAIT_VERB;
                        pair_vn = pair_vn_new;
                        output_AGC_uart(data_in, pair_vn);
                        ltp305g_write_digit(DISP_COMPACTY, '#');
                        update_ok = 1;
                    } else state = WAIT_VERB;
                    // break; // NO BREAKING
                case WAIT_VERB:
                    ESP_LOGI("KEYPAD", "STATE: WAIT_VERB: %c", recv_key);
                    /* Check error condition */
                    if (!update_ok)
                    {
                        ltp305g_write_lamp(LAMP_OPR_ERROR, 1);
                        ESP_LOGI("KEYPAD", "STATE: OPERATOR ERROR: %c", recv_key);
                    }
                    /* Clear VERB/NOUN and highlight displays if VERB is pressed */
                    pair_vn_new.verb = 0;
                    pair_vn_new.noun = 0;
                    counter = 0;
                    update_ok = 1;
                    /* Write to display */
                    ltp305g_write_digit(DISP_NOUN_0, pair_vn.noun % 10 + '0');
                    ltp305g_write_digit(DISP_NOUN_1, pair_vn.noun / 10 + '0');
                    ltp305g_write_digit(DISP_VERB_0, pair_vn.verb % 10 + '0');
                    ltp305g_write_digit(DISP_VERB_1, pair_vn.verb / 10 + '0');

                    if (recv_key == 'V')
                    {
                        ltp305g_write_digit(DISP_VERB_0, '_');
                        ltp305g_write_digit(DISP_VERB_1, '_');
                        ltp305g_write_digit(DISP_NOUN_0, '#');
                        ltp305g_write_digit(DISP_NOUN_1, '#');
                        state = INPUT_VERB;
                        update_ok = 0;
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
    static uint8_t char_status[NUM_ROWS][NUM_COLS] = {0};
    uint8_t flag_keypress = 0;

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
                flag_keypress = 1;
                if (char_status[row][col] == 0)
                {
                    // Send to queue if keystroke is detected
                    xQueueSendToBack(xQueue_keystrokes, &(char_LUT[row][col]), portMAX_DELAY);
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

    // Write to KEY_REL on keypress
    ltp305g_write_lamp(LAMP_KEY_REL, flag_keypress);
}

esp_err_t keypad_begin(uint32_t scan_interval_ms)
{
    // Set up queue
    xQueue_keystrokes = xQueueCreate(NUM_QUEUE, sizeof(uint8_t));
    if (xQueue_keystrokes == NULL) return ESP_ERR_NO_MEM;

    TimerHandle_t t = xTimerCreate("keypad_scan", pdMS_TO_TICKS(scan_interval_ms), 
                                   pdTRUE, (void*) id, &keypad_timer_callback);
    
    if (xTimerStart(t, 10) != pdPASS) return ESP_ERR_NOT_FINISHED;

    xTaskCreate(v_receiver_keystrokes, "KeystrokesHandler", 2048, 
                (void*) NULL, PRI_KEYSTROKES, NULL);

    return ESP_OK;
}