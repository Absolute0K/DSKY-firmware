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
#include "time.h"
#include "sys/time.h"

#define PRI_AGC_REPLY      (06)
#define PRI_MISSIONBUILTIN (05)

#define VERB_MISSIONTIME   (05)
#define VERB_TESTDISP      (06)
#define VERB_RESETIDLE     (07)
#define VERB_PROGCHNG      (39)

static struct timeval current_time = {0};
static data_in_t data_in = {0};
static pair_VN_t pair_vn = {0};

static void v_reply_AGC(void *pvParameters)
{
    for (;;)
        {
        /* Constantly update data*/
        xQueueReceive(xQueue_data_in, &data_in, portMAX_DELAY);
        xQueueReceive(xQueue_VN, &pair_vn, portMAX_DELAY);
        printf("<%o%o%c%o%c%o%c%o%c%o>", pair_vn.verb, pair_vn.noun, 
        (data_in.GM    >= 0) ? '+' : '-', data_in.GM,
        (data_in.invRA >= 0) ? '+' : '-', data_in.invRA,
        (data_in.invRB >= 0) ? '+' : '-', data_in.invRB, 
        (data_in.ATX   >= 0) ? '+' : '-', data_in.ATX);

    }
}

static void v_prog_builtin(void *pvParameters)
{
    char reg_date[7], reg_time[7], reg_secs[7];
    struct tm* dt;

    for (;;)
    {
        gettimeofday(&current_time, NULL);

        switch (pair_vn.verb)
        {
        case VERB_RESETIDLE:
            ltp305g_clear(DISP_REG0_0, DISP_NOUN_0);
            break;

        case VERB_MISSIONTIME:
            dt = localtime(&(current_time.tv_sec));
            sprintf(reg_date, "%u%u%u", dt->tm_year%100, (dt->tm_mon + 1)%100, (dt->tm_mday)%100);
            reg_date[1] |= 0x80; reg_date[3] |= 0x80;
            sprintf(reg_time, "%u%u%u", dt->tm_hour, dt->tm_min, dt->tm_sec); 
            reg_time[1] |= 0x80; reg_time[3] |= 0x80;
            sprintf(reg_secs, "%6d", (int) current_time.tv_usec);
            reg_secs[2] |= 0x80;
            ltp305g_puts(reg_date, DISP_REG0_0, 6);
            ltp305g_puts(reg_time, DISP_REG0_1, 6);
            ltp305g_puts(reg_secs, DISP_REG0_2, 6);
            break;

        case VERB_TESTDISP:
            /* Every 100 ms */
            if (current_time.tv_usec % 100000) break;
            /* Cycle through the characters */
            for (int i = DISP_REG0_0; i < DISP_NOUN_0; i++)
            {
                ltp305g_write_digit(i, '0' + (current_time.tv_usec/100000) % 36, 0);
            }
            /* Light up the lamps */
            uint8_t packets[12] = {0};
            for (int i = 0; i < 12; i++) packets[i] = 0xFF;
            ltp305g_write_lamps(packets, 12);
            break;
    
        default:
            break;
        }    
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("MAIN", "Hello world!");
    settimeofday(&current_time, NULL);

    /* Print chip information */
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
    //         CONFIG_IDF_TARGET,
    //         chip_info.cores,
    //         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
    //         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    // printf("silicon revision %d, ", chip_info.revision);

    // printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
    //         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    ESP_LOGI("MAIN", "Installing I2C Driver and checking...");

    ret = pca9548_begin(QWIIC_MUX_DEFAULT_ADDRESS, 19, 18, 100000);

    if (ret != ESP_OK) ESP_LOGE("MAIN", "Shit");

    ret = ltp305g_begin(0x80);
    ltp305g_set_total_brightness(0x9, 0x40);
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Fuck");

    ESP_LOGI("MAIN", "All good. Waiting for keypad\n");

    ret = keypad_begin(100);
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Ass");

    ret = bluetooth_begin();
    if (ret != ESP_OK) ESP_LOGE("MAIN", "Damn");

    xTaskCreate(v_reply_AGC, "ReplyAGCHandler", 2048, 
                (void*) NULL, PRI_AGC_REPLY, NULL);

    xTaskCreate(v_prog_builtin, "BuiltinHandler", 2048, 
                (void*) NULL, PRI_MISSIONBUILTIN, NULL);

    // uint32_t counter = 0;
    for (;;)
    {
        // uint32_t digit = counter;
        // for (int i = 0; i < 6; i++)
        // {
        //     ltp305g_write_digit(i, '0' + digit % 10);
        //     digit /= 10;
        // }
        // counter++;

        // if (counter % 20 == 0)
        // {
        //     uint8_t packets[12] = {0};
        //     for (int i = 0; i < 12; i++) packets[i] = 0xFF;
        //     ltp305g_write_lamps(packets, 12);
        // }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOGI("MAIN", "Restarting now.\n");

    fflush(stdout);
    esp_restart();
}
