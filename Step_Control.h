#ifndef STEP_CONTROL_H
#define STEP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t error;
    int16_t output_rpm;
    int32_t integral;
    bool link_valid;
} StepControl_Status;

void StepControl_Init(void);
void StepControl_Enter(void);
void StepControl_Update10ms(void);
void StepControl_Exit(void);
StepControl_Status StepControl_GetStatus(void);

#endif
