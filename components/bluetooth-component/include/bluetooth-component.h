/**
 * @file uart-component.h
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief Very simple FreeRTOS implementation of DSKY UART driver
 * @version 0.1
 * @date 2022-03-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef BLUETOOTH_COMP_H
#define BLUETOOTH_COMP_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SPP_TAG "SPP"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXCAMPLE_DEVICE_NAME "ESP_AGC_DEMO"

typedef struct data_in
{
    uint32_t verb;
    uint32_t noun;
    int32_t GM;
    int32_t invRA;
    int32_t invRB;
    int32_t ATX;
    int32_t scale[3];
    int32_t BURN_BABY_BURN;
    uint64_t mission_time;
} data_in_t;

struct timeval current_time;
data_in_t data_in;

esp_err_t bluetooth_spp_write(char *str, uint32_t len);
esp_err_t bluetooth_begin();

#endif
