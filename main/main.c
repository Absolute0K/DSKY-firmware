/* DSKY Main Task
    Jae Choi 2022 (jaewoonc)
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "I2C-Mux-Driver.h"
#include "LED-Driver.h"


void app_main(void)
{
    ESP_LOGI("MAIN", "Hello world!");

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

    esp_err_t ret = pca9548_begin(QWIIC_MUX_DEFAULT_ADDRESS, 19, 18, 100000);

    if (ret != ESP_OK) ESP_LOGE("MAIN", "Shit");

    ret = ltp305g_begin(0);

    if (ret != ESP_OK) ESP_LOGE("MAIN", "Fuck");


    ESP_LOGI("MAIN", "All good. Waiting for keypad\n");

    vTaskDelay(10000 / portTICK_PERIOD_MS);
    ESP_LOGI("MAIN", "Restarting now.\n");

    fflush(stdout);
    esp_restart();
}
