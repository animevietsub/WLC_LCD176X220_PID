/**
 ******************************************************************************
 * @file           : animation.c
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
#include "animation.h"

uint8_t old_percentage_speed = 0;
uint8_t old_percentage_level = 0;

void setDisplaySpeed(TFT_t *dev, uint8_t percentage)
{
    uint8_t x1 = old_percentage_speed * 127 / 100;
    uint8_t x2 = percentage * 127 / 100;
    if (x2 > x1)
    {
        lcdDrawFillRect(dev, 15 + x1, 201, 16 + x2, 209, GREEN);
    }
    else if (x2 < x1)
    {
        lcdDrawFillRect(dev, 15 + x2, 201, 16 + x1, 209, BLACK);
    }
    old_percentage_speed = percentage;
}

void setDisplayLevel(TFT_t *dev, uint8_t percentage)
{
    uint8_t x1 = old_percentage_level * 65 / 100;
    uint8_t x2 = percentage * 65 / 100;
    if (x2 > x1)
    {
        lcdDrawFillRect(dev, 55 + x1, 5, 56 + x2, 46, CYAN);
    }
    else if (x2 < x1)
    {
        lcdDrawFillRect(dev, 55 + x2, 5, 56 + x1, 46, 0xC69B);
    }
    old_percentage_level = percentage;
}

void setSV(TFT_t *dev, FontxFile *fx, uint16_t value)
{
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    lcdSetFontDirection(dev, 1);
    char text[6];
    sprintf(text, "%d", value);
    lcdDrawString(dev, fx, 70, 106, text, WHITE);
}

void setCV(TFT_t *dev, FontxFile *fx, uint16_t value)
{
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    lcdSetFontDirection(dev, 1);
    char text[6];
    sprintf(text, "%d", value);
    lcdDrawString(dev, fx, 53, 106, text, YELLOW);
}

void setP(TFT_t *dev, FontxFile *fx, uint16_t value)
{
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    lcdSetFontDirection(dev, 1);
    char text[6];
    sprintf(text, "%d", value);
    lcdDrawString(dev, fx, 6, 18, text, RED);
}

void setI(TFT_t *dev, FontxFile *fx, uint16_t value)
{
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    lcdSetFontDirection(dev, 1);
    char text[6];
    sprintf(text, "%d", value);
    lcdDrawString(dev, fx, 6, 62, text, GREEN);
}

void setD(TFT_t *dev, FontxFile *fx, uint16_t value)
{
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    lcdSetFontDirection(dev, 1);
    char text[6];
    sprintf(text, "%d", value);
    lcdDrawString(dev, fx, 6, 112, text, CYAN);
}

void drawImage(TFT_t *dev, char *file, uint16_t offsetX, uint16_t offsetY, uint16_t width, uint16_t height)
{
    pixel_jpeg **pixels;
    uint16_t imageWidth;
    uint16_t imageHeight;
    esp_err_t err = decode_jpeg(&pixels, file, width, height, &imageWidth, &imageHeight);
    if (err == ESP_OK)
    {
        uint16_t *colors = (uint16_t *)malloc(sizeof(uint16_t) * width);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                colors[x] = pixels[y][x];
            }
            lcdDrawMultiPixels(dev, offsetX, y + offsetY, width, colors);
        }
        free(colors);
        release_image(&pixels, width, height);
    }
    else
    {
        ESP_LOGE(__FUNCTION__, "decode_image err=%d imageWidth=%d imageHeight=%d", err, imageWidth, imageHeight);
    }
}

void drawLightRED(TFT_t *dev, uint16_t offsetX, uint16_t offsetY)
{
    char file[32];
    strcpy(file, "/spiffs/light_red.jpg");
    drawImage(dev, file, offsetX, offsetY, 16, 16);
}

void drawLightGREEN(TFT_t *dev, uint16_t offsetX, uint16_t offsetY)
{
    char file[32];
    strcpy(file, "/spiffs/light_green.jpg");
    drawImage(dev, file, offsetX, offsetY, 16, 16);
}