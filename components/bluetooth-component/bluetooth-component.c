/**
 * @file bluetooth-component.c
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief  Implementation for Bluetooth DSKY Component
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "bluetooth-component.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "keypad-component.h"


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    data_in_t data_in = {0};

    switch (event)
    {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                    param->data_ind.len, param->data_ind.handle);
            if (param->data_ind.len < 1023)
            {
                sscanf((char*) param->data_ind.data, "%d%d%d%d%d\n", 
                       &(data_in.GM), &(data_in.invRA), &(data_in.invRB), 
                       &(data_in.ATX), &(data_in.BURN_BABY_BURN));
                xQueueSendToBack(xQueue_data_in, &data_in, 1000);
                // esp_spp_write(param->write.handle, param->data_ind.len, param->data_ind.data);
            }
            else
            {
                esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
            }
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
            break;
        default:
            break;
        }
}


esp_err_t bluetooth_begin()
{
    esp_err_t ret = ESP_OK;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s Initialize Controller failed\n", __func__);
        return ret;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s Enable Controller failed\n", __func__);
        return ret;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s Initialize Bluedroid failed\n", __func__);
        return ret;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s Enable Bluedroid failed\n", __func__);
        return ret;
    }

    if (esp_spp_register_callback(esp_spp_cb) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s SPP Register failed\n", __func__);
        return ret;
    }

    if (esp_spp_init(ESP_SPP_MODE_CB) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s SPP Init failed\n", __func__);
        return ret;
    }

    xQueue_data_in = xQueueCreate(20, sizeof(data_in_t));
    if (xQueue_data_in == NULL) return ESP_ERR_NO_MEM;

    return ret;
}