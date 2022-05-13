#ifndef __L298N_LIBRARY_H__
#define __L298N_LIBRARY_H__

#define L298N_PWM_FREQ (5 * 1000)
#define L298N_TIMER LEDC_TIMER_0
#define L298N_PWM_CHANNEL_A LEDC_CHANNEL_0
#define L298N_PWM_CHANNEL_B LEDC_CHANNEL_1
#define L298N_PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define L298N_PWM_RES LEDC_TIMER_13_BIT
#define L298N_PWM_MAX_DUTY ((8192) - 1)
#define L298N_PWM_DUTY_INIT_A 0
#define L298N_PWM_DUTY_INIT_B 0

#define L298N_EN_A_PIN -1
#define L298N_EN_B_PIN 5
#define L298N_IN1_A_PIN -1
#define L298N_IN2_A_PIN -1
#define L298N_IN3_B_PIN 19
#define L298N_IN4_B_PIN 18

typedef enum
{
    L298N_DIRECTION_CW,  // Clockwise direction
    L298N_DIRECTION_CCW, // Counter-clockwise direction
    L298N_DIRECTION_HH,  // Hold high
    L298N_DIRECTION_HL,  // Hold low
} l298n_direction_t;

typedef enum
{
    L298N_CHANNEL_A, // Clockwise direction
    L298N_CHANNEL_B, // Counter-clockwise direction
} l298n_channel_t;

typedef struct
{
    uint16_t L298N_PWM_DUTY_A;
    uint16_t L298N_PWM_DUTY_B;
    l298n_direction_t L298N_DIRECTION_A;
    l298n_direction_t L298N_DIRECTION_B;
} l298n_control_t;

void L298N_Init(l298n_control_t *control);
void L298N_SetDirection(l298n_control_t *control, l298n_channel_t channel, l298n_direction_t dir);
void L298N_SetPWMDuty(l298n_control_t *control, l298n_channel_t channel, uint8_t percent);
void L298N_SetPWMDir(l298n_control_t *control, l298n_channel_t channel, int8_t percent);
void L298N_Stop(l298n_control_t *control, l298n_channel_t channel);
void L298N_Brake(l298n_control_t *control, l298n_channel_t channel);

#endif
