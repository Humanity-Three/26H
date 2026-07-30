#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_MAX                       (2500)
#define MOTOR_LEFT_FORWARD_REVERSED         (1)
#define MOTOR_RIGHT_FORWARD_REVERSED        (1)

/* Official 26-magnet encoder: 13 pulses * quadrature x4 * gearbox 30. */
#define ENCODER_COUNTS_PER_REV              (1560)
#define ENCODER_LEFT_REVERSED               (0)
#define ENCODER_RIGHT_REVERSED              (1)

typedef struct
{
    int16_t left_pwm;
    int16_t right_pwm;
    bool enabled;
} Motor_Status;

void Motor_Init(void);
void Motor_Enable(bool enable);
void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm);
void Motor_Coast(void);
void Motor_Brake(void);
Motor_Status Motor_GetStatus(void);

typedef struct
{
    int32_t left_count;
    int32_t right_count;
    int16_t left_delta_10ms;
    int16_t right_delta_10ms;
    int16_t left_rpm;
    int16_t right_rpm;
} Encoder_Status;

void Encoder_Init(void);
void Encoder_IRQHandler(void);
void Encoder_Update10ms(void);
void Encoder_Clear(void);
Encoder_Status Encoder_GetStatus(void);

#endif
