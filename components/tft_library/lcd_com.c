/**
 ******************************************************************************
 * @file           : lcd_com.c
 * @brief          : WLC_LCD176X220_PID
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
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lcd_com.h"

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL portEXIT_CRITICAL(&mux)

static uint8_t HC595_LCD_CTRL_BUFFER = 0x00;
static uint8_t HC595_LCD_DATA_BUFFER = 0x00;

DRAM_ATTR QueueHandle_t xQueue1;
DRAM_ATTR int64_t timeStartReceive, timeEndReceive;

static const char *TAG = "[WLC_PID]";

IRAM_ATTR uint8_t HC595_SortBit(uint8_t *data) // Sorting by BIT
{
	uint8_t temp_buff = 0x00;
	temp_buff |= ((*data & BIT0) != 0) * HC595_D0_BIT;
	temp_buff |= ((*data & BIT1) != 0) * HC595_D1_BIT;
	temp_buff |= ((*data & BIT2) != 0) * HC595_D2_BIT;
	temp_buff |= ((*data & BIT3) != 0) * HC595_D3_BIT;
	temp_buff |= ((*data & BIT4) != 0) * HC595_D4_BIT;
	temp_buff |= ((*data & BIT5) != 0) * HC595_D5_BIT;
	temp_buff |= ((*data & BIT6) != 0) * HC595_D6_BIT;
	temp_buff |= ((*data & BIT7) != 0) * HC595_D7_BIT;
	return temp_buff;
}

IRAM_ATTR void HC595_QueueDelayI2S(uint32_t delayUs)
{
	uint16_t HC595_TEMP_BUFFER = HC595_SortBit(&HC595_LCD_DATA_BUFFER) | HC595_LCD_CTRL_BUFFER << 8;
	for (uint32_t i = 0; i < delayUs / I2S_WS_PERIOD; i++)
	{
		xQueueSend(xQueue1, &HC595_TEMP_BUFFER, portMAX_DELAY);
	}
}

static void HC595_TaskSend()
{
	size_t i2s_bytes_write = DMA_BUFFER_PREPARE * sizeof(uint16_t);										 // For first writing
	uint16_t *sampleData = heap_caps_malloc(DMA_BUFFER_PREPARE * sizeof(uint16_t) * 1, MALLOC_CAP_8BIT); // Create DMA-buffer
	memset(sampleData, 0x0000, DMA_BUFFER_PREPARE * sizeof(uint16_t) * 1);								 // Clear memory data
	uint16_t *sampleDataBegin = sampleData;																 // sampleData begin address
	uint16_t lastData = 0x0000;
	uint32_t queueMessagesWaiting = 0;
	// uint8_t dmaSelect = 0;
	while (1)
	{
		// if (dmaSelect == 0) // Change dma buffer
		// {
		// 	sampleData = sampleDataBegin + 1; // Start from first-half, add 1 more shift
		// 	dmaSelect = 1;
		// }
		// else if (dmaSelect == 1)
		// {
		// 	sampleData = sampleDataBegin + DMA_BUFFER_PREPARE + 1; // Start from second-half, add 1 more shift
		// 	dmaSelect = 0;
		// }
		sampleData = sampleDataBegin + 1; // Start from first-half, add 1 more shift
		// timeStartReceive = esp_timer_get_time();
		for (uint16_t i = 0; i < DMA_BUFFER_PREPARE / 2; i++)
		{
			if (uxQueueMessagesWaiting(xQueue1) > 0)
			{
				queueMessagesWaiting = uxQueueMessagesWaiting(xQueue1);
				if ((DMA_BUFFER_PREPARE / 2) - i < queueMessagesWaiting)
				{
					queueMessagesWaiting = (DMA_BUFFER_PREPARE / 2) - i;
				}
				for (uint32_t temp = 0; temp < queueMessagesWaiting; temp++)
				{
					xQueueReceive(xQueue1, sampleData, portMAX_DELAY); // Get new data
					lastData = *sampleData;
					sampleData++;
					*sampleData = 0x0000;
					sampleData++;
				}
				i += queueMessagesWaiting - 1;
			}
			else
			{
				*sampleData = lastData;
				sampleData++;
				*sampleData = 0x0000;
				sampleData++;
			}
		}
		// timeEndReceive = esp_timer_get_time();
		// ESP_LOGI("[Queue]", "%lld us", timeEndReceive - timeStartReceive);
		sampleData = sampleData - DMA_BUFFER_PREPARE - 1; // Remove the shift
		// timeStartReceive = esp_timer_get_time();
		i2s_write(I2S_NUM, sampleData, DMA_BUFFER_PREPARE * sizeof(uint16_t), &i2s_bytes_write, portMAX_DELAY);
		// timeEndReceive = esp_timer_get_time();
		// ESP_LOGI("[I2S]", "%lld us", timeEndReceive - timeStartReceive);
	}
	heap_caps_free(sampleData);
	vTaskDelete(NULL);
}

void HC595_I2SInit()
{
	xQueue1 = xQueueCreate(DMA_BUFFER_PREPARE * QUEUE_DMA_MULTIPLIER, sizeof(uint16_t));
	i2s_config_t i2s_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX,
		.sample_rate = (HC595_CLKFREQ / I2S_NUM_CHANNEL / I2S_NUM_BIT),
		.bits_per_sample = I2S_NUM_BIT,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format = I2S_COMM_FORMAT_STAND_MSB,
		.use_apll = false,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.dma_buf_count = DMA_BUFFER_COUNT,
		.dma_buf_len = DMA_BUFFER_LENGTH,
	};
	i2s_pin_config_t pin_config = {
		.mck_io_num = I2S_PIN_NO_CHANGE,
		.bck_io_num = HC595_NUM_SRCLK,
		.ws_io_num = HC595_NUM_RCLK,
		.data_out_num = HC595_NUM_SER,
		.data_in_num = I2S_PIN_NO_CHANGE,
	};
	i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
	i2s_set_pin(I2S_NUM, &pin_config);
	// SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A_V, 1, I2S_CLKM_DIV_A_S);
	// SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B_V, 1, I2S_CLKM_DIV_B_S);
	// SET_PERI_REG_BITS(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM_V, 2, I2S_CLKM_DIV_NUM_S);
	// SET_PERI_REG_BITS(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BCK_DIV_NUM_V, 2, I2S_TX_BCK_DIV_NUM_S);
	i2s_start(I2S_NUM);
	xTaskCreate(HC595_TaskSend, "[HC595_TaskSend]", 1024 * 4, NULL, 3, NULL);
}

IRAM_ATTR void HC595_SendDataToQueue() // Add new data to Queue
{
	uint16_t HC595_TEMP_BUFFER = HC595_SortBit(&HC595_LCD_DATA_BUFFER) | HC595_LCD_CTRL_BUFFER << 8;
	xQueueSend(xQueue1, &HC595_TEMP_BUFFER, portMAX_DELAY);
}

IRAM_ATTR void HC595_SetCtrlBit(uint8_t HC595_BIT, uint8_t level) // Set ctrl bit
{
	if (level)
	{
		HC595_LCD_CTRL_BUFFER |= HC595_BIT;
	}
	else
	{
		HC595_LCD_CTRL_BUFFER &= ~HC595_BIT;
	}
	HC595_SendDataToQueue();
}

IRAM_ATTR void HC595_LCDWriteData(unsigned char *data, size_t size)
{
	for (int i = 0; i < size; i++)
	{
		HC595_LCD_DATA_BUFFER = data[i];
		HC595_SendDataToQueue();
		HC595_SetCtrlBit(HC595_WR_BIT, 0);
		HC595_SetCtrlBit(HC595_WR_BIT, 1);
	}
}

void lcd_write_table(TFT_t *dev, const void *table, int16_t size)
{
	int i;
	uint8_t *p = (uint8_t *)table;
	while (size > 0)
	{
		uint8_t cmd = *p++;
		uint8_t len = *p++;
		if (cmd == TFTLCD_DELAY8)
		{
			lcd_delay_ms(len);
			len = 0;
		}
		else
		{
			lcd_write_comm_byte(dev, cmd);
			for (i = 0; i < len; i++)
			{
				uint8_t data = *p++;
				lcd_write_data_byte(dev, data);
			}
		}
		size -= len + 2;
	}
}

void lcd_write_table16(TFT_t *dev, const void *table, int16_t size)
{
	uint16_t *p = (uint16_t *)table;
	while (size > 0)
	{
		uint16_t cmd = *p++;
		uint16_t dat = *p++;
		if (cmd == TFTLCD_DELAY16)
			lcd_delay_ms(dat);
		else
		{
			lcd_write_register_word(dev, cmd, dat);
		}
		size -= 2 * sizeof(int16_t);
	}
}

void lcd_write_comm_byte(TFT_t *dev, uint8_t cmd)
{
	unsigned char c[1];
	c[0] = cmd;
	HC595_SetCtrlBit(HC595_CS_BIT | HC595_RS_BIT, 0);
	HC595_LCDWriteData(c, 1);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_write_comm_word(TFT_t *dev, uint16_t cmd)
{
	unsigned char c[2];
	c[0] = (cmd >> 8) & 0xFF;
	c[1] = cmd & 0xFF;
	HC595_SetCtrlBit(HC595_CS_BIT | HC595_RS_BIT, 0);
	HC595_LCDWriteData(c, 2);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_write_data_byte(TFT_t *dev, uint8_t data)
{
	unsigned char d[1];
	d[0] = data;
	HC595_SetCtrlBit(HC595_CS_BIT, 0);
	HC595_SetCtrlBit(HC595_RS_BIT, 1);
	HC595_LCDWriteData(d, 1);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_write_data_word(TFT_t *dev, uint16_t data)
{
	unsigned char d[2];
	d[0] = (data >> 8) & 0xFF;
	d[1] = data & 0xFF;
	HC595_SetCtrlBit(HC595_CS_BIT, 0);
	HC595_SetCtrlBit(HC595_RS_BIT, 1);
	HC595_LCDWriteData(d, 2);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_write_addr(TFT_t *dev, uint16_t addr1, uint16_t addr2)
{
	unsigned char c[4];
	c[0] = (addr1 >> 8) & 0xFF;
	c[1] = addr1 & 0xFF;
	c[2] = (addr2 >> 8) & 0xFF;
	c[3] = addr2 & 0xFF;
	HC595_SetCtrlBit(HC595_CS_BIT, 0);
	HC595_SetCtrlBit(HC595_RS_BIT, 1);
	HC595_LCDWriteData(c, 4);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_write_color(TFT_t *dev, uint16_t color, uint16_t size)
{
	uint8_t *data;
	data = heap_caps_malloc(size * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
	int index = 0;
	for (int i = 0; i < size; i++)
	{
		data[index++] = (color >> 8) & 0xFF;
		data[index++] = color & 0xFF;
	}
	HC595_SetCtrlBit(HC595_CS_BIT, 0);
	HC595_SetCtrlBit(HC595_RS_BIT, 1);
	HC595_LCDWriteData(data, size * 2);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	heap_caps_free(data);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_write_colors(TFT_t *dev, uint16_t *colors, uint16_t size)
{
	unsigned char *data;
	data = heap_caps_malloc(size * 2, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
	int index = 0;
	for (int i = 0; i < size; i++)
	{
		data[index++] = (colors[i] >> 8) & 0xFF;
		data[index++] = colors[i] & 0xFF;
	}
	HC595_SetCtrlBit(HC595_CS_BIT, 0);
	HC595_SetCtrlBit(HC595_RS_BIT, 1);
	HC595_LCDWriteData(data, size * 2);
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	heap_caps_free(data);
	if (dev->_delay != 0)
		HC595_QueueDelayI2S(dev->_delay);
	// ets_delay_us(dev->_delay);
}

void lcd_delay_ms(int delay_time)
{
	vTaskDelay(delay_time / portTICK_RATE_MS);
}

void lcd_write_register_word(TFT_t *dev, uint16_t addr, uint16_t data)
{
	lcd_write_comm_word(dev, addr);
	lcd_write_data_word(dev, data);
}

void lcd_write_register_byte(TFT_t *dev, uint8_t addr, uint16_t data)
{
	lcd_write_comm_byte(dev, addr);
	lcd_write_data_word(dev, data);
}

esp_err_t lcd_interface_cfg(TFT_t *dev, int interface)
{
	HC595_SetCtrlBit(HC595_CS_BIT, 1);
	HC595_SetCtrlBit(HC595_RS_BIT, 1);
	HC595_SetCtrlBit(HC595_RD_BIT, 1);
	HC595_SetCtrlBit(HC595_WR_BIT, 1);
	HC595_SetCtrlBit(HC595_RST_BIT, 1);
	vTaskDelay(pdMS_TO_TICKS(100));
	HC595_SetCtrlBit(HC595_RST_BIT, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	HC595_SetCtrlBit(HC595_RST_BIT, 1);
	dev->_interface = interface;
	dev->_delay = 0;
	return ESP_OK;
}