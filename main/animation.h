/**
 ******************************************************************************
 * @file           : animation.h
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
#ifndef __ANIMATION_H__
#define __ANIMATION_H__

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
#include "decode_jpeg.h"

#include "ili9225.h"

void setDisplaySpeed(TFT_t *dev, uint8_t percentage);
void setDisplayLevel(TFT_t *dev, uint8_t percentage);
void setSV(TFT_t *dev, FontxFile *fx, uint16_t value);
void setCV(TFT_t *dev, FontxFile *fx, uint16_t value);
void setP(TFT_t *dev, FontxFile *fx, uint16_t value);
void setI(TFT_t *dev, FontxFile *fx, uint16_t value);
void setD(TFT_t *dev, FontxFile *fx, uint16_t value);
void drawImage(TFT_t *dev, char *file, uint16_t offsetX, uint16_t offsetY, uint16_t width, uint16_t height);
void drawLightRED(TFT_t *dev, uint16_t offsetX, uint16_t offsetY);
void drawLightGREEN(TFT_t *dev, uint16_t offsetX, uint16_t offsetY);

#endif