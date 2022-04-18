/**
 * @file main.c
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief Main Task Code for the DSKY
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "I2C-Mux-Driver.h"
#include "LED-Driver.h"
#include "keypad-component.h"
#include "uart-component.h"
#include "bluetooth-component.h"
#include "atomic-i2c.h"
#include "time.h"
#include "sys/time.h"

#define PRI_AGC_REPLY      (06)
#define PRI_MISSIONBUILTIN (05)

#define VERB_MISSIONTIME   (05)
#define VERB_TESTDISP      (06)
#define VERB_RESETIDLE     (07)
#define VERB_PROGCHNG      (39)


static void v_prog_builtin(void *pvParameters)
{
    char reg_date[7], reg_time[7], reg_secs[7];
    struct tm* dt;
    uint32_t counter = 0;

    for (;;)
    {
        gettimeofday(&current_time, NULL);

        switch (pair_vn.verb)
        {
            case VERB_RESETIDLE:
                ltp305g_clear(DISP_REG0_0, DISP_NOUN_0);
                ltp305g_clear(DISP_PROG_0, 2);
                ltp305g_write_lamps("000000000000");
                ltp305g_write_lamp(LAMP_STBY, 1);
                ltp305g_write_digit(DISP_COMPACTY, ' ');
                break;

            case VERB_MISSIONTIME:
                /* Get mission time */
                ltp305g_write_lamps("000000000000");
                ltp305g_write_lamp(LAMP_EMPTY, 1);
                ltp305g_write_lamp(LAMP_PROG, 1);
                ltp305g_write_lamp(LAMP_STBY, 1);

                ltp305g_write_digit(DISP_PROG_0, '0');
                ltp305g_write_digit(DISP_PROG_1, '0');
                ltp305g_write_digit(DISP_COMPACTY, '#');

                dt = localtime(&(current_time.tv_sec));
                sprintf(reg_date, "%02u%02u%02u", dt->tm_year, dt->tm_mon + 1, dt->tm_mday);
                reg_date[2] |= 0x80; reg_date[4] |= 0x80;
                sprintf(reg_time, "%02u%02u%02u", dt->tm_hour, dt->tm_min, dt->tm_sec); 
                reg_time[2] |= 0x80; reg_time[4] |= 0x80;
                sprintf(reg_secs, "%06d", (int) current_time.tv_usec);
                reg_secs[3] |= 0x80;
                ltp305g_puts(reg_date, DISP_REG2_0, 6);
                ltp305g_puts(reg_time, DISP_REG1_0, 6);
                ltp305g_puts(reg_secs, DISP_REG0_0, 6);
                break;

            case VERB_TESTDISP:
                /* Cycle through the characters */
                for (int i = DISP_REG0_0; i < DISP_NOUN_0; i++)
                    ltp305g_write_digit(i, '0' + counter);
                ltp305g_write_digit(DISP_PROG_0, '0' + counter);
                ltp305g_write_digit(DISP_PROG_1, '0' + counter);
                ltp305g_write_digit(DISP_COMPACTY, '#');
                counter = (counter + 1) % 10;
                ltp305g_write_digit(DISP_COMPACTY, (counter % 2) ? '#' : ' ');
                /* Light up the lamps */
                if (counter > 4)  ltp305g_write_lamps("000000000000");
                else              ltp305g_write_lamps("111111111100");
                vTaskDelay(60 / portTICK_PERIOD_MS);
                break;
        
            case VERB_PROGCHNG:
            default:
                break;
        }

        /* Every 20 ms */
        vTaskDelay(20 / portTICK_PERIOD_MS);   
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ret |= atomic_i2c_begin();
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("MAIN", "Hello world!");
    current_time.tv_sec = 0;
    current_time.tv_usec = 0;
    settimeofday(&current_time, NULL);

    ESP_LOGI("MAIN", "Installing I2C Driver and checking...");

    ret = pca9548_begin(QWIIC_MUX_DEFAULT_ADDRESS, 19, 18, 100000);
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Shit");

    ret = ltp305g_begin(0x80);
    ret |= ltp305g_set_current_bright(0x8, 0x40, NUM_TOTAL_DRIVERS - 1, 1);
    ret |= ltp305g_set_current_bright(3, 0x80, 0, NUM_TOTAL_DRIVERS - 1);
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Fuck");

    ESP_LOGI("MAIN", "All good. Waiting for keypad\n");

    ret = keypad_begin(100);
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Ass");

    ret = bluetooth_begin();
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Damn");

    ret = uart_begin();
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Crap");

    xTaskCreate(v_prog_builtin, "BuiltinHandler", 2048, 
                (void*) NULL, PRI_MISSIONBUILTIN, NULL);

    // Default to testing mode
    pair_vn.verb = VERB_TESTDISP;

    for (;;)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    fflush(stdout);
    esp_restart();
}
