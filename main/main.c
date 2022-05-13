/**
 ******************************************************************************
 * @file           : main.c
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
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "lcd_com.h"
#include "lcd_lib.h"
#include "fontx.h"
#include "decode_jpeg.h"
#include "animation.h"

#include "ili9225.h"
#include "SR04M_LIB.h"
#include "l298n_library.h"

#include "port.h"
#include "main.h"

#define DRIVER "ST7775"
#define INTERVAL 500
#define WAIT vTaskDelay(INTERVAL)
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static const char *TAG_I2S = "[HC595_I2S]";
static const char *TAG_SPIFFS = "[SPIFFS]";

static int16_t cv_value = 0;
static int16_t zv_value = 0;
static int16_t sv_value = 0;
static int16_t p_value = 0;
static int16_t i_value = 0;
static int16_t d_value = 0;

menu_list_t menu_list = MENU_IDLE;
nvs_handle_t nvs_handle_data;
l298n_control_t l298n_control;

SemaphoreHandle_t xSemaphore1;

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

TickType_t JPEGLOGO(TFT_t *dev, char *file, int width, int height)
{
    TickType_t startTick, endTick, diffTick;
    lcdSetFontDirection(dev, 0);
    lcdFillScreen(dev, BLACK);
    startTick = xTaskGetTickCount();
    pixel_jpeg **pixels;
    uint16_t imageWidth;
    uint16_t imageHeight;
    // uint freeRAM = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    // ESP_LOGI("[DRAM]", "free RAM is %d.", freeRAM);
    esp_err_t err = decode_jpeg(&pixels, file, width, height, &imageWidth, &imageHeight);
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
        release_image(&pixels, width, height);
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
        char file[32];
        strcpy(file, "/spiffs/logo_gamo.jpg");
        JPEGLOGO(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
        nvs_get_i16(nvs_handle_data, "sv_value", &sv_value);
        nvs_get_i16(nvs_handle_data, "zv_value", &zv_value);
        nvs_get_i16(nvs_handle_data, "p_value", &p_value);
        nvs_get_i16(nvs_handle_data, "i_value", &i_value);
        nvs_get_i16(nvs_handle_data, "d_value", &d_value);
        WAIT;
        if (gpio_get_level(SWITCH_UP_PIN) == false && gpio_get_level(SWITCH_DOWN_PIN) == false)
        {
            taskCalibration(&dev, fx16G);
        }

        strcpy(file, "/spiffs/background.jpg");
        JPEGLOGO(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
        vTaskDelay(10);
        while (1)
        {
            if (menu_list == MENU_SV)
                lcdSetFontUnderLine(&dev, RED);
            setSV(&dev, fx16G, sv_value);
            lcdUnsetFontUnderLine(&dev);
            setCV(&dev, fx16G, cv_value);
            if (menu_list == MENU_P)
                lcdSetFontUnderLine(&dev, RED);
            setP(&dev, fx16G, p_value);
            lcdUnsetFontUnderLine(&dev);
            if (menu_list == MENU_I)
                lcdSetFontUnderLine(&dev, RED);
            setI(&dev, fx16G, i_value);
            lcdUnsetFontUnderLine(&dev);
            if (menu_list == MENU_D)
                lcdSetFontUnderLine(&dev, RED);
            setD(&dev, fx16G, d_value);
            lcdUnsetFontUnderLine(&dev);
            if (l298n_control.L298N_PWM_DUTY_B == 0 || l298n_control.L298N_DIRECTION_B == L298N_DIRECTION_HH || l298n_control.L298N_DIRECTION_B == L298N_DIRECTION_HL)
                drawLightRED(&dev, 94, 78);
            else
                drawLightGREEN(&dev, 94, 78);
            setDisplaySpeed(&dev, l298n_control.L298N_PWM_DUTY_B);
            setDisplayLevel(&dev, MIN(MAX(0, cv_value) * 2 / 5, 100));
            xSemaphoreTake(xSemaphore1, pdMS_TO_TICKS(500));
        }
    }
    vTaskDelete(NULL);
}

static void taskMotorController()
{
    L298N_Init(&l298n_control);
    while (1)
    {
        if (cv_value < sv_value)
        {
            L298N_SetPWMDir(&l298n_control, L298N_CHANNEL_B, 80);
        }
        else
        {
            L298N_Stop(&l298n_control, L298N_CHANNEL_B);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

void initButton()
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = false,
        .pull_up_en = false,
    };
    io_conf.pin_bit_mask = (1ULL << SWITCH_UP_PIN) | (1ULL << SWITCH_DOWN_PIN) | (1ULL << SWITCH_LEFT_PIN) | (1ULL << SWITCH_RIGHT_PIN);
    gpio_config(&io_conf);
}

static void taskButton()
{
    while (1)
    {
        if (gpio_get_level(SWITCH_UP_PIN) == 0)
        {
            menu_list--;
            if (menu_list < MENU_IDLE)
                menu_list = MENU_D;
            xSemaphoreGive(xSemaphore1);
        }
        else if (gpio_get_level(SWITCH_DOWN_PIN) == 0)
        {
            menu_list++;
            if (menu_list > MENU_D)
                menu_list = MENU_IDLE;
            xSemaphoreGive(xSemaphore1);
        }
        else if (gpio_get_level(SWITCH_LEFT_PIN) == 0)
        {
            if (menu_list == MENU_SV)
            {
                if (sv_value > 0)
                {
                    sv_value--;
                }
                nvs_set_i16(nvs_handle_data, "sv_value", sv_value);
                xSemaphoreGive(xSemaphore1);
            }
            else if (menu_list == MENU_P)
            {
                p_value--;
                nvs_set_i16(nvs_handle_data, "p_value", p_value);
                xSemaphoreGive(xSemaphore1);
            }
            else if (menu_list == MENU_I)
            {
                i_value--;
                nvs_set_i16(nvs_handle_data, "i_value", i_value);
                xSemaphoreGive(xSemaphore1);
            }
            else if (menu_list == MENU_D)
            {
                d_value--;
                nvs_set_i16(nvs_handle_data, "d_value", d_value);
                xSemaphoreGive(xSemaphore1);
            }
        }
        else if (gpio_get_level(SWITCH_RIGHT_PIN) == 0)
        {
            if (menu_list == MENU_SV)
            {
                sv_value++;
                nvs_set_u16(nvs_handle_data, "sv_value", sv_value);
                xSemaphoreGive(xSemaphore1);
            }
            else if (menu_list == MENU_P)
            {
                p_value++;
                nvs_set_i16(nvs_handle_data, "p_value", p_value);
                xSemaphoreGive(xSemaphore1);
            }
            else if (menu_list == MENU_I)
            {
                i_value++;
                nvs_set_i16(nvs_handle_data, "i_value", i_value);
                xSemaphoreGive(xSemaphore1);
            }
            else if (menu_list == MENU_D)
            {
                d_value++;
                nvs_set_i16(nvs_handle_data, "d_value", d_value);
                xSemaphoreGive(xSemaphore1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void taskSR04M()
{
    SR04M_Init();
    int16_t distance;
    while (1)
    {
        SM04M_getDistance(&distance);
        cv_value = zv_value - distance;
        if (cv_value < 0)
        {
            cv_value = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

void taskCalibration(TFT_t *dev, FontxFile *fx)
{
    lcdFillScreen(dev, BLACK);
    uint8_t buffer[FontxGlyphBufSize];
    uint8_t fontWidth;
    uint8_t fontHeight;
    GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
    lcdSetFontDirection(dev, 1);
    char ascii[20];
    strcpy(ascii, "CALIBRATING");
    setTextInCenter(dev, fx, ascii, 10, YELLOW);
    for (int8_t i = 5; i > 0; i--)
    {
        sprintf(ascii, "%d ...", i);
        setTextInCenter(dev, fx, ascii, -10, RED);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    uint32_t zv_temp = 0;
    for (int i = 0; i < CAL_SAMPLES; i++)
    {
        SM04M_getDistance(&zv_value);
        vTaskDelay(pdMS_TO_TICKS(10));
        zv_temp += zv_value;
    }
    zv_temp /= CAL_SAMPLES;
    zv_value = zv_temp;
    nvs_set_i16(nvs_handle_data, "zv_value", zv_value);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void app_main(void)
{
    xSemaphore1 = xSemaphoreCreateBinary();
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
    nvs_flash_init();
    nvs_open("storage", NVS_READWRITE, &nvs_handle_data);
    initButton();
    xTaskCreate(taskLCDContoller, "[taskLCDContoller]", 1024 * 6, NULL, 2, NULL);
    xTaskCreate(taskMotorController, "[taskMotorController]", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(taskButton, "[taskButton]", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(taskSR04M, "[taskSR04M]", 1024 * 3, NULL, 2, NULL);
}
