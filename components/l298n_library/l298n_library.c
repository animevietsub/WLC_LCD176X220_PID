/**
 ******************************************************************************
 * @file           : l298n_library.c
 * @brief          : L298N library - H Bridge
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
#include "l298n_library.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

void L298N_Init(l298n_control_t *control)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    ledc_timer_config_t l298n_timer = {
        .speed_mode = L298N_PWM_SPEED_MODE,
        .timer_num = L298N_TIMER,
        .duty_resolution = L298N_PWM_RES,
        .freq_hz = L298N_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&l298n_timer));

#if (L298N_EN_A_PIN != -1)
    ledc_channel_config_t l298n_channel_a = {
        .speed_mode = L298N_PWM_SPEED_MODE,
        .channel = L298N_PWM_CHANNEL_A,
        .timer_sel = L298N_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = L298N_EN_A_PIN,
        .duty = L298N_PWM_DUTY_INIT_A,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&l298n_channel_a));
    io_conf.pin_bit_mask = (1ULL << L298N_IN1_A_PIN) | (1ULL << L298N_IN2_A_PIN);
    gpio_config(&io_conf);
    L298N_Brake(control, L298N_CHANNEL_A);
#endif

#if (L298N_EN_B_PIN != -1)
    ledc_channel_config_t l298n_channel_b = {
        .speed_mode = L298N_PWM_SPEED_MODE,
        .channel = L298N_PWM_CHANNEL_B,
        .timer_sel = L298N_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = L298N_EN_B_PIN,
        .duty = L298N_PWM_DUTY_INIT_B,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&l298n_channel_b));
    io_conf.pin_bit_mask = (1ULL << L298N_IN3_B_PIN) | (1ULL << L298N_IN4_B_PIN);
    gpio_config(&io_conf);
    L298N_Brake(control, L298N_CHANNEL_B);
#endif
}

void L298N_SetDirection(l298n_control_t *control, l298n_channel_t channel, l298n_direction_t dir)
{
    if (channel == L298N_CHANNEL_A)
    {
        switch (dir)
        {
        case L298N_DIRECTION_CW:
            gpio_set_level(L298N_IN1_A_PIN, true);
            gpio_set_level(L298N_IN2_A_PIN, false);
            break;
        case L298N_DIRECTION_CCW:
            gpio_set_level(L298N_IN1_A_PIN, false);
            gpio_set_level(L298N_IN2_A_PIN, true);
            break;
        case L298N_DIRECTION_HH:
            gpio_set_level(L298N_IN1_A_PIN, true);
            gpio_set_level(L298N_IN2_A_PIN, true);
            break;
        case L298N_DIRECTION_HL:
            gpio_set_level(L298N_IN1_A_PIN, false);
            gpio_set_level(L298N_IN2_A_PIN, false);
            break;
        }
        (*control).L298N_DIRECTION_A = dir;
    }
    else if (channel == L298N_CHANNEL_B)
    {
        switch (dir)
        {
        case L298N_DIRECTION_CW:
            gpio_set_level(L298N_IN3_B_PIN, true);
            gpio_set_level(L298N_IN4_B_PIN, false);
            break;
        case L298N_DIRECTION_CCW:
            gpio_set_level(L298N_IN3_B_PIN, false);
            gpio_set_level(L298N_IN4_B_PIN, true);
            break;
        case L298N_DIRECTION_HH:
            gpio_set_level(L298N_IN3_B_PIN, true);
            gpio_set_level(L298N_IN4_B_PIN, true);
            break;
        case L298N_DIRECTION_HL:
            gpio_set_level(L298N_IN3_B_PIN, false);
            gpio_set_level(L298N_IN4_B_PIN, false);
            break;
        }
        (*control).L298N_DIRECTION_B = dir;
    }
}

void L298N_SetPWMDuty(l298n_control_t *control, l298n_channel_t channel, uint8_t percent)
{
    if (channel == L298N_CHANNEL_A)
    {
        ledc_set_duty(L298N_PWM_SPEED_MODE, L298N_PWM_CHANNEL_A, L298N_PWM_MAX_DUTY * percent / 100);
        ledc_update_duty(L298N_PWM_SPEED_MODE, L298N_PWM_CHANNEL_A);
        (*control).L298N_PWM_DUTY_A = percent;
    }
    else if (channel == L298N_CHANNEL_B)
    {
        ledc_set_duty(L298N_PWM_SPEED_MODE, L298N_PWM_CHANNEL_B, L298N_PWM_MAX_DUTY * percent / 100);
        ledc_update_duty(L298N_PWM_SPEED_MODE, L298N_PWM_CHANNEL_B);
        (*control).L298N_PWM_DUTY_B = percent;
    }
}

void L298N_SetPWMDir(l298n_control_t *control, l298n_channel_t channel, int8_t percent)
{
    if (percent >= 0)
    {
        L298N_SetDirection(control, channel, L298N_DIRECTION_CCW);
        L298N_SetPWMDuty(control, channel, percent);
    }
    else
    {
        L298N_SetDirection(control, channel, L298N_DIRECTION_CW);
        L298N_SetPWMDuty(control, channel, -percent);
    }
}

void L298N_Stop(l298n_control_t *control, l298n_channel_t channel)
{
    L298N_SetPWMDuty(control, channel, 0);
    L298N_SetDirection(control, channel, L298N_DIRECTION_HL);
}

void L298N_Brake(l298n_control_t *control, l298n_channel_t channel)
{
    L298N_SetPWMDuty(control, channel, 100);
    L298N_SetDirection(control, channel, L298N_DIRECTION_HL);
}