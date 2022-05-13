/**
 ******************************************************************************
 * @file           : main.h
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
#ifndef __MAIN_H__
#define __MAIN_H__

#define CAL_SAMPLES 10

typedef enum
{
    DUMMY = -1,
    MENU_IDLE = 0,
    MENU_SV,
    MENU_P,
    MENU_I,
    MENU_D,
} menu_list_t;

void initButton();
void taskCalibration(TFT_t *dev, FontxFile *fx);

#endif
