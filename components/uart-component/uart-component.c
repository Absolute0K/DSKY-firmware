/**
 * @file uart-component.c
 * @author Jae Choi (AbsoluteA0K@gmail.com)
 * @brief  Implementation for UART DSKY Component
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include "uart-component.h"
#include "esp_log.h"
#include "LED-Driver.h"
#include "bluetooth-component.h"

static QueueHandle_t xQueue_UART0;
const char char_SOP = '<';
const char char_EOP = '>';

#define PRI_UART_EVENT      (8)
#define PRI_UART_DISP       (7)

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data_uart[1024] = {0};
    uart_pkt_t packet = {0};
    uint32_t index_packet = 0, len;

    for (;;)
    {
        /* Waiting for UART event. */
        if (xQueueReceive(xQueue_UART0, (void*) &event, (TickType_t) portMAX_DELAY))
        {
            switch(event.type) {
                /* Event of UART receving data */
                case UART_DATA:
                    len = uart_read_bytes(UART_NUM_0, &data_uart, event.size, portMAX_DELAY);
                    printf("Got: ");
                    for (int i = 0; i < len; i++)
                    {
                        printf("%c\n", data_uart[i]);
                        if (data_uart[i] != char_SOP && index_packet == 0) continue;
                        /* If we find the SOP */
                        printf("Found: ");
                        for (int j = i; j < len && index_packet < UART_PKT_SIZE; j++)
                        {
                            printf("%c[%d] ", data_uart[j], index_packet);
                            packet.data[index_packet] = data_uart[j];
                            index_packet++;
                        }
                        /* Validate index and EOP */
                        if (index_packet == UART_PKT_SIZE)
                        {
                            printf("\nSend: %s\n", packet.data);
                            if (packet.data[index_packet-1] == char_EOP)
                                xQueueSend(xQueue_packets, &packet, portMAX_DELAY);
                            index_packet = 0;
                            break;
                        }
                    }
                    break;
                /* Event of HW FIFO overflow detected */
                case UART_FIFO_OVF:
                    ESP_LOGI("UART", "HW FIFO overflow");
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(xQueue_UART0);
                    break;
                /* Event of UART ring buffer full */
                case UART_BUFFER_FULL:
                    ESP_LOGI("UART", "Ring buffer is full");
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(xQueue_UART0);
                    break;
                /* Event of UART RX break detected */
                case UART_BREAK:
                    ESP_LOGI("UART", "UART RX break");
                    break;
                /* Others */
                default:
                    ESP_LOGI("UART", "UART event type: %d", event.type);
                    break;
            }
        }
    }
}

static void uart_display_task(void *pvParameters)
{
    uart_pkt_t packet = {0};
    uint32_t dsky_REG0, dsky_REG1, dsky_REG2, res_dva, res_dvatx, res_dvb, res_dvbtx, dsky_prog;
    uint8_t lamps[13] = {0}; 
    char str_output[40] = {0};
    char str_REG0[7] = {0}, str_REG1[7] = {0}, str_REG2[7] = {0}, str_PROG[3] = {0};
    
    for (;;)
    {
        /* Waiting for UART event. */
        if (xQueueReceive(xQueue_packets, &packet, (TickType_t) portMAX_DELAY))
        {
            /* Scan: <DSKYREG1_DSKYREG2_DSKYREG3_PROGNUM_LAMPS_DVA_DVATX_DVB_DVBTX> */
            packet.data[UART_PKT_SIZE - 1] = 0;
            // sscanf(packet.data + 1, "%6o%6o%6o%2o%12s%6o%6o%6o%6o",
            sscanf(packet.data + 1, "%*c%5o%*c%5o%*c%5o%2o%12s%*c%5o%*c%5o%*c%5o%*c%5o",
                   &dsky_REG0, &dsky_REG1, &dsky_REG2, &dsky_prog, lamps, 
                   &res_dva,   &res_dvatx, &res_dvb,   &res_dvbtx);
            printf("%o %o %o %o %12s %o %o %o %o\n",
                   dsky_REG0, dsky_REG1, dsky_REG2, dsky_prog, lamps, 
                   res_dva,   res_dvatx, res_dvb,   res_dvbtx);

            /* Ignore if AGC wants to display mission time */
            if (dsky_REG0 == 0x7FFF || dsky_REG1 == 0x7FFF || dsky_REG2 == 0x7FFF) continue;

            /* PERFORM DENORMALIZATION */

            /* Display on DSKY */
            sprintf(str_REG0, "%c%05d", (dsky_REG0 >= 0) ? '+' : '-', dsky_REG0);
            sprintf(str_REG1, "%c%05d", (dsky_REG1 >= 0) ? '+' : '-', dsky_REG1);
            sprintf(str_REG2, "%c%05d", (dsky_REG2 >= 0) ? '+' : '-', dsky_REG2);
            sprintf(str_PROG, "%02d", dsky_prog);
            ltp305g_puts(str_REG0, DISP_REG0_0, 6);
            ltp305g_puts(str_REG0, DISP_REG1_0, 6);
            ltp305g_puts(str_REG0, DISP_REG2_0, 6);
            ltp305g_puts(str_REG0, DISP_PROG_0, 2);

            /* Output to Python Visualizer */
            sprintf(str_output, "%05d %05d %05d %05d %02d\n", 
                    res_dva, res_dvatx, res_dvb, res_dvbtx, dsky_prog);
            bluetooth_spp_write(str_output, 40);

        }
    }
}

esp_err_t uart_begin()
{   
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    esp_err_t ret = ESP_OK;
    /* Install UART driver, and get the queue. */
    ret = uart_driver_install(UART_NUM_0, UART_BUF_SIZE*2, UART_BUF_SIZE*2, 20, &xQueue_UART0, 0);
    if (ret != ESP_OK) ESP_LOGE("UART", "Error installing UART0 driver");
    ret = uart_param_config(UART_NUM_0, &uart_config);
    if (ret != ESP_OK) ESP_LOGE("UART", "Error configuring UART0 driver");

    /* Create a task to handler UART event from ISR */
    xQueue_packets = xQueueCreate(20, sizeof(uart_pkt_t));
    if (xQueue_packets == NULL) return ESP_ERR_NO_MEM;

    xTaskCreate(uart_event_task,   "uart_event_task", 8192, NULL, PRI_UART_EVENT, NULL);
    xTaskCreate(uart_display_task, "uart_display_task", 2048, NULL, PRI_UART_DISP, NULL);

    return ret;
}