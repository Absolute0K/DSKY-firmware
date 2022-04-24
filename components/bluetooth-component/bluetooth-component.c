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
#include "keypad-component.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define NUM_BLU_PKTS (50)

static uint32_t __handle = 0;
// static data_in_t data_in = {0};

esp_err_t bluetooth_spp_write(char *str, uint32_t len)
{
    return esp_spp_write(__handle, len, (uint8_t *)str);
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    uint32_t index_packet = 0;
    uint8_t strbuf[NUM_BLU_PKTS] = {0};
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
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
        if (param->start.status == ESP_SPP_SUCCESS)
        {
            esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        }
        else
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT Status:%d", param->start.status);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 1023)
        {
            for (int i = 0; i < param->data_ind.len; i++)
            {
                if (param->data_ind.data[i] != '<' && index_packet == 0)
                    continue;
                /* If we find the SOP */
                for (int j = i; j < param->data_ind.len; j++)
                {
                    strbuf[index_packet] = param->data_ind.data[j];
                    /* Validate index and EOP */
                    if (strbuf[index_packet] == '>' && index_packet < NUM_BLU_PKTS - 1)
                    {
                        strbuf[index_packet + 1] = '\0';
                        // <+01234+12345+23456+34567 10 10 1 123 123123>
                        // <+02322+30071+02322+30071 10 10 1 123 123123>
                        // <+20000+20000+20000+20000 10 10 1 123 123123>
                        sscanf((char *)strbuf, "<%*c%05o%*c%05o%*c%05o%*c%05o %d %d %d %d %llu>",
                               &(data_in.GM), &(data_in.invRA), &(data_in.invRB),
                               &(data_in.ATX), &(data_in.scale[0]), &(data_in.scale[1]),
                               &(data_in.scale[2]), &(data_in.BURN_BABY_BURN), &(data_in.mission_time));
                        output_AGC_uart(data_in, pair_vn);
                        // current_time.tv_sec = data_in.mission_time;
                        index_packet = 0;
                        break;
                    }
                    index_packet++;
                }
            }
        }
        else
        {
            esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
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
        __handle = param->write.handle;
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "Authentication success: %s", param->auth_cmpl.device_name);
        }
        else
        {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT Mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(SPP_TAG, "Event: %d", event);
        break;
    }
    return;
}

esp_err_t bluetooth_begin()
{
    esp_err_t ret = ESP_OK;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    data_in = (data_in_t){0};
    data_in.scale[0] = 1;
    data_in.scale[1] = 1;
    data_in.scale[2] = 1;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s Initialize Controller failed\n", __func__);
        return ret;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s Enable Controller failed\n", __func__);
        return ret;
    }

    if (esp_bluedroid_init() != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s Initialize Bluedroid failed\n", __func__);
        return ret;
    }

    if (esp_bluedroid_enable() != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s Enable Bluedroid failed\n", __func__);
        return ret;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if (esp_spp_register_callback(esp_spp_cb) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s SPP Register failed\n", __func__);
        return ret;
    }

    if (esp_spp_init(ESP_SPP_MODE_CB) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s SPP Init failed\n", __func__);
        return ret;
    }

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    return ret;
}