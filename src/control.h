#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "encoder_motor.h"

/*
 * 8 路灰度单环循迹控制。
 * PID 的输入是黑线相对传感器中心的偏差，输出是左右轮 PWM 差速量。
 */
typedef struct
{
    int16_t base_pwm;
    int16_t kp;
    int16_t ki;
    int16_t kd;
    int16_t max_turn_pwm;
} LineFollow_Config;

typedef struct
{
    uint8_t sensor_mask;
    uint8_t black_count;
    int16_t error;
    int16_t turn_pwm;
    bool line_lost;
    bool cross_line;
} LineFollow_Status;

void LineFollow_Init(void);
void LineFollow_Reset(void);
void LineFollow_SetConfig(const LineFollow_Config *config);
void LineFollow_Update10ms(void);
LineFollow_Status LineFollow_GetStatus(void);

#endif
