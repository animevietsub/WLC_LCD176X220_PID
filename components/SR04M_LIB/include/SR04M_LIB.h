/**
 ******************************************************************************
 * @file           : SR04M_LIB.h
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
#ifndef __SR04M_LIB_H__
#define __SR04M_LIB_H__

#define SONIC_TX_PIN 16
#define SONIC_RX_PIN 17
#define UART_BUFFER_SIZE (1024) // Size for reveiving distance DATA
#define SONIC_BUFFER_SIZE (64)  // Size from SONIC
#define SR04M_DEBUG 0

void SR04M_Init();
void SM04M_getDistance(uint16_t *distance);

#endif
