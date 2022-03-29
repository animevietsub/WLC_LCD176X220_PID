/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : RCC_LCD176X220_NRF24L01
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
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "lcd_com.h"
#include "lcd_lib.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"

#include "ili9225.h"

#define DRIVER "ST7775"
#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

static const char *TAG_I2S = "[HC595_I2S]";
static const char *TAG_SPIFFS = "[SPIFFS]";

static void checkSPIFFS(char *path)
{
    DIR *dir = opendir(path);
    assert(dir != NULL);
    while (1)
    {
        struct dirent *data = readdir(dir);
        if (!data)
            break;
        ESP_LOGI(TAG_SPIFFS, "d_name=%s", data->d_name);
    }
    closedir(dir);
}

TickType_t AddressTest(TFT_t *dev, int width, int height, uint16_t color)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    lcdFillScreen(dev, color);
    lcdDrawFillRect(dev, 0, 0, 19, 19, RED);
    lcdDrawFillRect(dev, 20, 20, 39, 39, GREEN);
    lcdDrawFillRect(dev, 40, 40, 59, 59, BLUE);
    lcdDrawFillRect(dev, 60, 60, 79, 79, ~color);

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t FillTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    lcdFillScreen(dev, RED);
    vTaskDelay(50);
    lcdFillScreen(dev, GREEN);
    vTaskDelay(50);
    lcdFillScreen(dev, BLUE);
    vTaskDelay(50);

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t ColorBarTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    if (width < height)
    {
        uint16_t y1, y2;
        y1 = height / 3;
        y2 = (height / 3) * 2;
        lcdDrawFillRect(dev, 0, 0, width - 1, y1 - 1, RED);
        vTaskDelay(1);
        lcdDrawFillRect(dev, 0, y1 - 1, width - 1, y2 - 1, GREEN);
        vTaskDelay(1);
        lcdDrawFillRect(dev, 0, y2 - 1, width - 1, height - 1, BLUE);
    }
    else
    {
        uint16_t x1, x2;
        x1 = width / 3;
        x2 = (width / 3) * 2;
        lcdDrawFillRect(dev, 0, 0, x1 - 1, height - 1, RED);
        vTaskDelay(1);
        lcdDrawFillRect(dev, x1 - 1, 0, x2 - 1, height - 1, GREEN);
        vTaskDelay(1);
        lcdDrawFillRect(dev, x2 - 1, 0, width - 1, height - 1, BLUE);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t ArrowTest(TFT_t *dev, FontxFile *fx, char *model, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    // get font width & height
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    ESP_LOGD(__FUNCTION__, "fontWidth=%d fontHeight=%d", fontWidth, fontHeight);

    uint16_t xpos;
    uint16_t ypos;
    int stlen;
    uint8_t ascii[24];
    uint16_t color;

    lcdFillScreen(dev, BLACK);

    if (width < height)
    {
        xpos = ((width - fontHeight) / 2) - 1;
        ypos = (height - (strlen(model) * fontWidth)) / 2;
        lcdSetFontDirection(dev, DIRECTION90);
    }
    else
    {
        ypos = ((height - fontHeight) / 2) - 1;
        xpos = (width - (strlen(model) * fontWidth)) / 2;
        lcdSetFontDirection(dev, DIRECTION0);
    }

    ESP_LOGD(__FUNCTION__, "xpos=%d ypos=%d model=%s", xpos, ypos, model);
    strcpy((char *)ascii, model);
    color = WHITE;
    lcdDrawString(dev, fx, xpos, ypos, ascii, color);

    lcdSetFontDirection(dev, DIRECTION0);
    color = RED;
    lcdDrawFillArrow(dev, 10, 10, 0, 0, 5, color);
    strcpy((char *)ascii, "0,0");
    lcdDrawString(dev, fx, 0, 30, ascii, color);

    color = GREEN;
    lcdDrawFillArrow(dev, width - 11, 10, width - 1, 0, 5, color);
    // strcpy((char *)ascii, "79,0");
    sprintf((char *)ascii, "%d,0", width - 1);
    stlen = strlen((char *)ascii);
    xpos = (width - 1) - (fontWidth * stlen);
    lcdDrawString(dev, fx, xpos, 30, ascii, color);

    color = GRAY;
    lcdDrawFillArrow(dev, 10, height - 11, 0, height - 1, 5, color);
    // strcpy((char *)ascii, "0,159");
    sprintf((char *)ascii, "0,%d", height - 1);
    ypos = (height - 11) - (fontHeight) + 5;
    lcdDrawString(dev, fx, 0, ypos, ascii, color);

    color = CYAN;
    lcdDrawFillArrow(dev, width - 11, height - 11, width - 1, height - 1, 5, color);
    // strcpy((char *)ascii, "79,159");
    sprintf((char *)ascii, "%d,%d", width - 1, height - 1);
    stlen = strlen((char *)ascii);
    xpos = (width - 1) - (fontWidth * stlen);
    lcdDrawString(dev, fx, xpos, ypos, ascii, color);

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t DirectionTest(TFT_t *dev, FontxFile *fx, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    // get font width & height
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    // ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

    uint16_t color;
    lcdFillScreen(dev, BLACK);
    uint8_t ascii[20];

    color = RED;
    strcpy((char *)ascii, "Direction=0");
    lcdSetFontDirection(dev, 0);
    lcdDrawString(dev, fx, 0, fontHeight - 1, ascii, color);

    color = BLUE;
    strcpy((char *)ascii, "Direction=2");
    lcdSetFontDirection(dev, 2);
    lcdDrawString(dev, fx, (width - 1), (height - 1) - (fontHeight * 1), ascii, color);

    color = CYAN;
    strcpy((char *)ascii, "Direction=1");
    lcdSetFontDirection(dev, 1);
    lcdDrawString(dev, fx, (width - 1) - fontHeight, 0, ascii, color);

    color = GREEN;
    strcpy((char *)ascii, "Direction=3");
    lcdSetFontDirection(dev, 3);
    lcdDrawString(dev, fx, (fontHeight - 1), height - 1, ascii, color);

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t HorizontalTest(TFT_t *dev, FontxFile *fx, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    // get font width & height
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    // ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

    uint16_t color;
    lcdFillScreen(dev, BLACK);
    uint8_t ascii[20];

    color = RED;
    strcpy((char *)ascii, "Direction=0");
    lcdSetFontDirection(dev, 0);
    lcdDrawString(dev, fx, 0, fontHeight * 1 - 1, ascii, color);
    lcdSetFontUnderLine(dev, RED);
    lcdDrawString(dev, fx, 0, fontHeight * 2 - 1, ascii, color);
    lcdUnsetFontUnderLine(dev);

    lcdSetFontFill(dev, GREEN);
    lcdDrawString(dev, fx, 0, fontHeight * 3 - 1, ascii, color);
    lcdSetFontUnderLine(dev, RED);
    lcdDrawString(dev, fx, 0, fontHeight * 4 - 1, ascii, color);
    lcdUnsetFontFill(dev);
    lcdUnsetFontUnderLine(dev);

    color = BLUE;
    strcpy((char *)ascii, "Direction=2");
    lcdSetFontDirection(dev, 2);
    lcdDrawString(dev, fx, width - 1, height - (fontHeight * 1) - 1, ascii, color);
    lcdSetFontUnderLine(dev, BLUE);
    lcdDrawString(dev, fx, width - 1, height - (fontHeight * 2) - 1, ascii, color);
    lcdUnsetFontUnderLine(dev);

    lcdSetFontFill(dev, YELLOW);
    lcdDrawString(dev, fx, width - 1, height - (fontHeight * 3) - 1, ascii, color);
    lcdSetFontUnderLine(dev, BLUE);
    lcdDrawString(dev, fx, width - 1, height - (fontHeight * 4) - 1, ascii, color);
    lcdUnsetFontFill(dev);
    lcdUnsetFontUnderLine(dev);

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t VerticalTest(TFT_t *dev, FontxFile *fx, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    // get font width & height
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    // ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

    uint16_t color;
    lcdFillScreen(dev, BLACK);
    uint8_t ascii[20];

    color = RED;
    strcpy((char *)ascii, "Direction=1");
    lcdSetFontDirection(dev, 1);
    lcdDrawString(dev, fx, width - (fontHeight * 1), 0, ascii, color);
    lcdSetFontUnderLine(dev, RED);
    lcdDrawString(dev, fx, width - (fontHeight * 2), 0, ascii, color);
    lcdUnsetFontUnderLine(dev);

    lcdSetFontFill(dev, GREEN);
    lcdDrawString(dev, fx, width - (fontHeight * 3), 0, ascii, color);
    lcdSetFontUnderLine(dev, RED);
    lcdDrawString(dev, fx, width - (fontHeight * 4), 0, ascii, color);
    lcdUnsetFontFill(dev);
    lcdUnsetFontUnderLine(dev);

    color = BLUE;
    strcpy((char *)ascii, "Direction=3");
    lcdSetFontDirection(dev, 3);
    lcdDrawString(dev, fx, (fontHeight * 1) - 1, height - 1, ascii, color);
    lcdSetFontUnderLine(dev, BLUE);
    lcdDrawString(dev, fx, (fontHeight * 2) - 1, height - 1, ascii, color);
    lcdUnsetFontUnderLine(dev);

    lcdSetFontFill(dev, YELLOW);
    lcdDrawString(dev, fx, (fontHeight * 3) - 1, height - 1, ascii, color);
    lcdSetFontUnderLine(dev, BLUE);
    lcdDrawString(dev, fx, (fontHeight * 4) - 1, height - 1, ascii, color);
    lcdUnsetFontFill(dev);
    lcdUnsetFontUnderLine(dev);

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t LineTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    uint16_t color;
    // lcdFillScreen(dev, WHITE);
    lcdFillScreen(dev, BLACK);
    color = RED;
    for (int ypos = 0; ypos < height; ypos = ypos + 10)
    {
        lcdDrawLine(dev, 0, ypos, width, ypos, color);
    }

    for (int xpos = 0; xpos < width; xpos = xpos + 10)
    {
        lcdDrawLine(dev, xpos, 0, xpos, height, color);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t CircleTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    uint16_t color;
    // lcdFillScreen(dev, WHITE);
    lcdFillScreen(dev, BLACK);
    color = CYAN;
    uint16_t xpos = width / 2;
    uint16_t ypos = height / 2;
    for (int i = 5; i < height; i = i + 5)
    {
        lcdDrawCircle(dev, xpos, ypos, i, color);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t RoundRectTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    uint16_t color;
    uint16_t limit = width;
    if (width > height)
        limit = height;
    // lcdFillScreen(dev, WHITE);
    lcdFillScreen(dev, BLACK);
    color = BLUE;
    for (int i = 5; i < limit; i = i + 5)
    {
        if (i > (limit - i - 1))
            break;
        // ESP_LOGI(__FUNCTION__, "i=%d, width-i-1=%d",i, width-i-1);
        lcdDrawRoundRect(dev, i, i, (width - i - 1), (height - i - 1), 10, color);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t RectAngleTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    uint16_t color;
    // lcdFillScreen(dev, WHITE);
    lcdFillScreen(dev, BLACK);
    color = CYAN;
    uint16_t xpos = width / 2;
    uint16_t ypos = height / 2;

    uint16_t w = width * 0.6;
    uint16_t h = w * 0.5;
    int angle;
    for (angle = 0; angle <= (360 * 3); angle = angle + 30)
    {
        lcdDrawRectAngle(dev, xpos, ypos, w, h, angle, color);
        usleep(10000);
        lcdDrawRectAngle(dev, xpos, ypos, w, h, angle, BLACK);
    }

    for (angle = 0; angle <= 180; angle = angle + 30)
    {
        lcdDrawRectAngle(dev, xpos, ypos, w, h, angle, color);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t TriangleTest(TFT_t *dev, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    uint16_t color;
    // lcdFillScreen(dev, WHITE);
    lcdFillScreen(dev, BLACK);
    color = CYAN;
    uint16_t xpos = width / 2;
    uint16_t ypos = height / 2;

    uint16_t w;
    uint16_t h;
    if (height > width)
    {
        w = width * 0.6;
        h = w * 1.0;
    }
    else
    {
        w = height * 0.6;
        h = w * 1.0;
    }
    int angle;

    for (angle = 0; angle <= (360 * 3); angle = angle + 30)
    {
        lcdDrawTriangle(dev, xpos, ypos, w, h, angle, color);
        usleep(10000);
        lcdDrawTriangle(dev, xpos, ypos, w, h, angle, BLACK);
    }

    for (angle = 0; angle <= 360; angle = angle + 30)
    {
        lcdDrawTriangle(dev, xpos, ypos, w, h, angle, color);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

TickType_t JPEGTest(TFT_t *dev, char *file, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    startTick = xTaskGetTickCount();

    lcdSetFontDirection(dev, 0);
    lcdFillScreen(dev, BLACK);

    int _width = width;
    if (width > 240)
        _width = 240;
    int _height = height;
    if (height > 320)
        _height = 320;

    pixel_jpeg **pixels;
    uint16_t imageWidth;
    uint16_t imageHeight;
    startTick = xTaskGetTickCount();
    esp_err_t err = decode_jpeg(&pixels, file, _width, _height, &imageWidth, &imageHeight);
    endTick = xTaskGetTickCount();
    if (err == ESP_OK)
    {
        ESP_LOGI(__FUNCTION__, "imageWidth=%d imageHeight=%d", imageWidth, imageHeight);

        uint16_t jpegWidth = width;
        uint16_t offsetX = 0;
        if (width > imageWidth)
        {
            jpegWidth = imageWidth;
            offsetX = (width - imageWidth) / 2;
        }
        ESP_LOGD(__FUNCTION__, "jpegWidth=%d offsetX=%d", jpegWidth, offsetX);

        uint16_t jpegHeight = height;
        uint16_t offsetY = 0;
        if (height > imageHeight)
        {
            jpegHeight = imageHeight;
            offsetY = (height - imageHeight) / 2;
        }
        ESP_LOGD(__FUNCTION__, "jpegHeight=%d offsetY=%d", jpegHeight, offsetY);
        uint16_t *colors = (uint16_t *)malloc(sizeof(uint16_t) * jpegWidth);
        for (int y = 0; y < jpegHeight; y++)
        {
            for (int x = 0; x < jpegWidth; x++)
            {
                colors[x] = pixels[y][x];
            }
            lcdDrawMultiPixels(dev, offsetX, y + offsetY, jpegWidth, colors);
            // vTaskDelay(1);
        }
        free(colors);
        release_image(&pixels, _width, _height);
        ESP_LOGD(__FUNCTION__, "Finish");
    }
    else
    {
        ESP_LOGE(__FUNCTION__, "decode_image err=%d imageWidth=%d imageHeight=%d", err, imageWidth, imageHeight);
    }

    endTick = xTaskGetTickCount();
    diffTick = endTick - startTick;
    ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d", diffTick * portTICK_RATE_MS);
    return diffTick;
}

static void taskLCDContoller()
{
    FontxFile fx16G[2];
    FontxFile fx24G[2];
    FontxFile fx32G[2];
    InitFontx(fx16G, "/spiffs/ILGH16XB.FNT", ""); // 8x16Dot Gothic
    InitFontx(fx24G, "/spiffs/ILGH24XB.FNT", ""); // 12x24Dot Gothic
    InitFontx(fx32G, "/spiffs/ILGH32XB.FNT", ""); // 16x32Dot Gothic
    FontxFile fx16M[2];
    FontxFile fx24M[2];
    FontxFile fx32M[2];
    InitFontx(fx16M, "/spiffs/ILMH16XB.FNT", ""); // 8x16Dot Mincyo
    InitFontx(fx24M, "/spiffs/ILMH24XB.FNT", ""); // 12x24Dot Mincyo
    InitFontx(fx32M, "/spiffs/ILMH32XB.FNT", ""); // 16x32Dot Mincyo
    TFT_t dev;
    lcd_interface_cfg(&dev, 1);
    ili9225_lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);
    // char stats_buffer[1024];
    // vTaskList(stats_buffer);
    // ESP_LOGI("[stats_buffer]", "%s", stats_buffer);
#if CONFIG_INVERSION
    ESP_LOGI(TAG, "Enable Display Inversion");
    lcdInversionOn(&dev);
#endif
    while (1)
    {
        // AddressTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, BLACK);
        // WAIT;
        // AddressTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, WHITE);
        // WAIT;
        // FillTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // ColorBarTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // ArrowTest(&dev, fx16G, DRIVER, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // LineTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // CircleTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // RoundRectTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // RectAngleTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        // TriangleTest(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);
        // WAIT;
        char file[32];
        strcpy(file, "/spiffs/logo_gamo.jpg");
        JPEGTest(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
        WAIT;
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG_SPIFFS, "Initializing SPIFFS");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 10,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG_SPIFFS, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG_SPIFFS, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG_SPIFFS, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPIFFS, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG_SPIFFS, "Partition size: total: %d, used: %d", total, used);
    }
    // esp_spiffs_format(conf.partition_label);
    checkSPIFFS("/spiffs/"); // Check files
    ESP_LOGI(TAG_I2S, "Starting init LCD_I2S");
    HC595_I2SInit();
    xTaskCreate(taskLCDContoller, "[taskLCDContoller]", 1024 * 32, NULL, 2, NULL);
    vTaskDelete(NULL);
}
