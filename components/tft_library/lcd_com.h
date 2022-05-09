#ifndef __LCD_COM_H__
#define __LCD_COM_H__

#include "i2s_lcd_driver.h"

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY16 0xFFFF
#define TFTLCD_DELAY8 0x7F

#define INTERFACE_I2S 1
#define INTERFACE_GPIO 2
#define INTERFACE_REG 3
#define INTERFACE_74HC595 4

#define GPIO_PORT_NUM 0

#define MODE_RESET 0
#define MODE_OUTPUT 1
#define MODE_INPUT 2

#define NUMSAMPLES 2  // Number of samples
#define COMP 2		  // Coordinate tolerance
#define AVERAGETIME 4 // Number of samples when averaging
#define RXPLATE 300

#define HC595_CLKFREQ (8 * 1000 * 1000)
#define I2S_NUM_CHANNEL 2
#define I2S_NUM_BIT I2S_BITS_PER_SAMPLE_16BIT
#define I2S_NUM (0)
#define I2S_WS_PERIOD ((16 * I2S_NUM_CHANNEL) / (HC595_CLKFREQ / (1000 * 1000))) // 16 bit data - 4 us
#define DMA_BUFFER_LENGTH 256
#define DMA_BUFFER_COUNT 16
#define DMA_BUFFER_PREPARE (DMA_BUFFER_LENGTH * 2) // 64 queues * 2 channels
#define QUEUE_DMA_MULTIPLIER 8

#define HC595_NUM_RCLK 26
#define HC595_NUM_SRCLK 27
#define HC595_NUM_SER 25

#define HC595_D0_BIT (BIT2)
#define HC595_D1_BIT (BIT7)
#define HC595_D2_BIT (BIT6)
#define HC595_D3_BIT (BIT5)
#define HC595_D4_BIT (BIT4)
#define HC595_D5_BIT (BIT3)
#define HC595_D6_BIT (BIT1)
#define HC595_D7_BIT (BIT0)
#define HC595_LED_BIT (BIT1)
#define HC595_RST_BIT (BIT2)
#define HC595_RD_BIT (BIT3)
#define HC595_WR_BIT (BIT4)
#define HC595_RS_BIT (BIT5)
#define HC595_CS_BIT (BIT7)

typedef struct
{
	uint16_t _width;
	uint16_t _height;
	uint16_t _offsetx;
	uint16_t _offsety;
	uint16_t _font_direction;
	uint16_t _font_fill;
	uint16_t _font_fill_color;
	uint16_t _font_underline;
	uint16_t _font_underline_color;
	int16_t _rd;
	int16_t _wr;
	int16_t _rs;
	int16_t _cs;
	int16_t _d0;
	int16_t _d1;
	int16_t _d2;
	int16_t _d3;
	int16_t _d4;
	int16_t _d5;
	int16_t _d6;
	int16_t _d7;
	int16_t _delay;
	int16_t _interface;
	bool _debug;
	i2s_lcd_handle_t i2s_lcd_handle;
	adc1_channel_t _adc_yp;
	adc1_channel_t _adc_xm;
	int16_t _gpio_xp;
	int16_t _gpio_xm;
	int16_t _gpio_yp;
	int16_t _gpio_ym;
	int16_t _rawxp;
	int16_t _rawyp;
	bool _calibration;
	int16_t _min_xp; // Minimum xp calibration
	int16_t _min_yp; // Minimum yp calibration
	int16_t _max_xp; // Maximum xp calibration
	int16_t _max_yp; // Maximum yp calibration
	int16_t _min_xc; // Minimum x coordinate
	int16_t _min_yc; // Minimum y coordinate
	int16_t _max_xc; // Maximum x coordinate
	int16_t _max_yc; // Maximum y coordinate
} TFT_t;

void lcd_write_table(TFT_t *dev, const void *table, int16_t size);
void lcd_write_table16(TFT_t *dev, const void *table, int16_t size);
void lcd_write_comm_byte(TFT_t *dev, uint8_t cmd);
void lcd_write_comm_word(TFT_t *dev, uint16_t cmd);
void lcd_write_data_byte(TFT_t *dev, uint8_t data);
void lcd_write_data_word(TFT_t *dev, uint16_t data);
void lcd_write_addr(TFT_t *dev, uint16_t addr1, uint16_t addr2);
void lcd_write_color(TFT_t *dev, uint16_t color, uint16_t size);
void lcd_write_colors(TFT_t *dev, uint16_t *colors, uint16_t size);
void lcd_delay_ms(int delay_time);
void lcd_write_register_word(TFT_t *dev, uint16_t addr, uint16_t data);
void lcd_write_register_byte(TFT_t *dev, uint8_t addr, uint16_t data);
esp_err_t lcd_interface_cfg(TFT_t *dev, int interface);
void HC595_I2SInit();
IRAM_ATTR void HC595_QueueDelayI2S(uint32_t delayUs);
IRAM_ATTR uint8_t HC595_SortBit(uint8_t *data);
IRAM_ATTR void HC595_SendDataToQueue();
IRAM_ATTR void HC595_SetCtrlBit(uint8_t HC595_BIT, uint8_t level);
IRAM_ATTR void HC595_LCDWriteData(unsigned char *data, size_t size);

#endif
