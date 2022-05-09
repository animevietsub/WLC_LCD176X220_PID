/**
 ******************************************************************************
 * @file           :  SR04M_LIB.c
 * @brief          : AJ-SR04M sonic sensor
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 Espressif.
 * All rights reserved.
 *
 * Vo Duc Toan / B1907202
 * Can Tho University.
 * March - 2022
 * Built with ESP-IDF Version: 4.4.
 * Target device: ESP32-WROOM.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "SR04M_LIB.h"

static const char *TAG = "[SR04M]";
const uart_port_t uart_num = UART_NUM_2;
const char SONIC_MEASURE_REQUEST[] = {0xFF};

void SR04M_Init()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_set_pin(uart_num, SONIC_RX_PIN, SONIC_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, 0, 0));
}

void SM04M_getDistance(uint16_t *distance)
{
    uint8_t *data = (uint8_t *)malloc(SONIC_BUFFER_SIZE);
    uart_write_bytes(uart_num, SONIC_MEASURE_REQUEST, sizeof(SONIC_MEASURE_REQUEST) / sizeof(char));
    int len = uart_read_bytes(uart_num, data, (SONIC_BUFFER_SIZE - 1), pdMS_TO_TICKS(50));
    if (len)
    {
        if (data[0] == 0xFF)
        {
#if SR04M_DEBUG
            ESP_LOGI(TAG, "Recv data: %d, %d, %d", data[0], data[1], data[2]);
            ESP_LOGI(TAG, "Distance: %d mm", (uint16_t)((data[1] << 8) + data[2]));
#endif
            *distance = (uint16_t)((data[1] << 8) + data[2]); // Error
        }
        else
        {
#if SR04M_DEBUG
            ESP_LOGE(TAG, "Error in getting distance");
            ESP_LOGI(TAG, "Recv data: %d, %d, %d", data[0], data[1], data[2]);
#endif
            *distance = 0; // Error in getting distance
        }
    }
    free(data);
}