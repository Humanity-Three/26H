#ifndef STEP_CONTROL_H
#define STEP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define STEP_CONTROL_LEVEL_POSITION_TENTHS (908L)

typedef struct
{
    int16_t raw_dx;
    int16_t target_offset;
    int16_t error;
    int16_t output_rpm;
    int32_t integral;
    bool link_valid;
} StepControl_Status;

void StepControl_Init(void);
void StepControl_Enter(void);
void StepControl_SetTargetOffsetPixels(int16_t offset_pixels);
void StepControl_EnableVelocityProfile(bool enable);
bool StepControl_SetOpenLoopOutputTenths(int16_t output_tenths);
bool StepControl_SetOpenLoopPositionTenths(int32_t position_tenths);
void StepControl_Update10ms(void);
void StepControl_Exit(void);
StepControl_Status StepControl_GetStatus(void);

#endif
