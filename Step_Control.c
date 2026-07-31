#include "Step_Control.h"

#include "K230_link.h"
#include "X_V2.h"
#include "zdt_can_port.h"

#define STEP_MOTOR_ADDRESS          (1U)
#define STEP_MOTOR_POSITION_RPM     (60.0f)
#define STEP_MOTOR_POSITION_SLEW    (15L)
#define STEP_MOTOR_POSITION_TICKS   (1U)
#define STEP_CONTROL_DEAD_ZONE      (4)
#define STEP_CONTROL_MAX_RPM        (60)
#define STEP_CONTROL_INTEGRAL_MAX   (2000)
#define STEP_CONTROL_INTEGRAL_ZONE  (96)
#define STEP_CONTROL_LOST_HOLD_TICKS (8U)
#define STEP_CONTROL_POSITION_EPSILON (2)
#define STEP_CONTROL_ERROR_JUMP_MAX (160)
#define STEP_CONTROL_STICTION_DERIVATIVE_MAX (2)
#define STEP_CONTROL_STICTION_OUTPUT_TENTHS (25)
#define STEP_LEVEL_OFFSET_TENTH_DEG (611L)
#define STEP_POSITION_FULL_OUTPUT_TENTHS (300L)

/*
 * Firm, strongly damped balance baseline for the ~23 FPS camera:
 * Kp=0.10, Ki=0.002, Kd=1.20.
 * Kp remains above the original soft baseline, while the stronger derivative
 * term and conservative position slew suppress oscillatory settling. A very
 * weak, center-only integral removes the remaining static position bias.
 */
#define STEP_PID_KP_NUM             (100)
#define STEP_PID_KI_NUM             (2)
#define STEP_PID_KD_NUM             (1200)
#define STEP_PID_GAIN_DEN           (1000)

static StepControl_Status g_step_status;
static int16_t g_previous_error;
static int16_t g_filtered_error;
static int16_t g_filtered_derivative;
static int16_t g_target_offset_pixels;
static int16_t g_output_tenths;
static uint8_t g_lost_target_ticks;
static uint32_t g_last_frame_count;
static int32_t g_last_target_position;
static int32_t g_command_position;
static uint8_t g_position_command_ticks;
static bool g_command_position_valid;
static bool g_filter_valid;
static bool g_target_position_valid;
static bool g_loss_stop_sent;
static bool g_open_loop_active;
static bool g_active;

static void StepControl_SendPosition(int32_t target_tenth_degree)
{
    uint8_t direction =
        (target_tenth_degree < 0) ? 1U : 0U;
    float position_degree =
        (target_tenth_degree < 0) ?
        (float)-target_tenth_degree / 10.0f :
        (float)target_tenth_degree / 10.0f;

    X_V2_Bypass_Pos_LV_Control(
        STEP_MOTOR_ADDRESS, direction,
        STEP_MOTOR_POSITION_RPM, position_degree, 1U, false);
}

static void StepControl_UpdateMotorPosition(void)
{
    int32_t current_position;
    int32_t previous_command;
    uint32_t position_sequence;

    if (!g_target_position_valid)
    {
        return;
    }

    if (!g_command_position_valid)
    {
        if (!ZDT_CAN_GetMotorPosition(
                STEP_MOTOR_ADDRESS, &current_position,
                &position_sequence))
        {
            return;
        }
        (void)position_sequence;
        g_command_position = current_position;
        g_command_position_valid = true;
    }

    g_position_command_ticks++;
    if (g_position_command_ticks < STEP_MOTOR_POSITION_TICKS)
    {
        return;
    }
    g_position_command_ticks = 0U;
    previous_command = g_command_position;

    if (g_last_target_position >
        g_command_position + STEP_MOTOR_POSITION_SLEW)
    {
        g_command_position += STEP_MOTOR_POSITION_SLEW;
    }
    else if (g_last_target_position <
             g_command_position - STEP_MOTOR_POSITION_SLEW)
    {
        g_command_position -= STEP_MOTOR_POSITION_SLEW;
    }
    else
    {
        g_command_position = g_last_target_position;
    }
    if (g_command_position != previous_command)
    {
        StepControl_SendPosition(g_command_position);
    }
}

static int16_t StepControl_Clamp(int32_t value, int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

void StepControl_Init(void)
{
    g_active = false;
    g_step_status.error = 0;
    g_step_status.output_rpm = 0;
    g_step_status.integral = 0;
    g_step_status.link_valid = false;
    g_previous_error = 0;
    g_filtered_error = 0;
    g_filtered_derivative = 0;
    g_target_offset_pixels = 0;
    g_output_tenths = 0;
    g_lost_target_ticks = 0U;
    g_last_frame_count = 0U;
    g_last_target_position = 0;
    g_command_position = 0;
    g_position_command_ticks = 0U;
    g_command_position_valid = false;
    g_filter_valid = false;
    g_target_position_valid = false;
    g_loss_stop_sent = false;
    g_open_loop_active = false;
}

void StepControl_Enter(void)
{
    StepControl_Init();
    g_active = true;
    X_V2_En_Control(STEP_MOTOR_ADDRESS, true, false);
}

void StepControl_SetTargetOffsetPixels(int16_t offset_pixels)
{
    g_open_loop_active = false;
    if (offset_pixels == g_target_offset_pixels)
    {
        return;
    }

    g_target_offset_pixels = offset_pixels;
    g_step_status.error = 0;
    g_step_status.integral = 0;
    g_previous_error = 0;
    g_filtered_error = 0;
    g_filtered_derivative = 0;
    g_output_tenths = 0;
    g_filter_valid = false;
}

bool StepControl_SetOpenLoopOutputTenths(int16_t output_tenths)
{
    int32_t minimum_position;
    int32_t maximum_position;
    int32_t level_position;
    int32_t target;

    if (!g_active ||
        !ZDT_CAN_GetMotorLimits(
            STEP_MOTOR_ADDRESS, &minimum_position, &maximum_position))
    {
        return false;
    }

    output_tenths = StepControl_Clamp(
        output_tenths, STEP_CONTROL_MAX_RPM * 10);
    level_position = minimum_position + STEP_LEVEL_OFFSET_TENTH_DEG;
    if (level_position < minimum_position)
    {
        level_position = minimum_position;
    }
    else if (level_position > maximum_position)
    {
        level_position = maximum_position;
    }

    if (output_tenths >= 0)
    {
        target = level_position -
            ((int32_t)output_tenths *
             (level_position - minimum_position)) /
            STEP_POSITION_FULL_OUTPUT_TENTHS;
    }
    else
    {
        target = level_position +
            ((int32_t)-output_tenths *
             (maximum_position - level_position)) /
            STEP_POSITION_FULL_OUTPUT_TENTHS;
    }

    if (target < minimum_position)
    {
        target = minimum_position;
    }
    else if (target > maximum_position)
    {
        target = maximum_position;
    }

    g_open_loop_active = true;
    g_last_target_position = target;
    g_target_position_valid = true;
    g_step_status.link_valid = true;
    g_step_status.error = 0;
    g_step_status.output_rpm = output_tenths / 10;
    g_step_status.integral = 0;
    g_output_tenths = output_tenths;
    return true;
}

bool StepControl_SetOpenLoopPositionTenths(int32_t position_tenths)
{
    int32_t minimum_position;
    int32_t maximum_position;

    if (!g_active ||
        !ZDT_CAN_GetMotorLimits(
            STEP_MOTOR_ADDRESS, &minimum_position, &maximum_position))
    {
        return false;
    }

    if (position_tenths < minimum_position)
    {
        position_tenths = minimum_position;
    }
    else if (position_tenths > maximum_position)
    {
        position_tenths = maximum_position;
    }

    g_open_loop_active = true;
    g_last_target_position = position_tenths;
    g_target_position_valid = true;
    g_step_status.link_valid = true;
    g_step_status.error = 0;
    g_step_status.output_rpm = 0;
    g_step_status.integral = 0;
    g_output_tenths = 0;
    return true;
}

void StepControl_Update10ms(void)
{
    const K230_LinkData *link;
    int16_t error;
    int16_t derivative;
    int32_t candidate_integral;
    int32_t unsaturated_output;
    int32_t output;
    bool control_updated = false;

    if (!g_active)
    {
        return;
    }

    if (g_open_loop_active)
    {
        StepControl_UpdateMotorPosition();
        return;
    }

    link = K230_Link_GetData();
    if (K230_Link_IsValid(500U) && link->detected)
    {
        g_step_status.link_valid = true;
        g_lost_target_ticks = 0U;
        g_loss_stop_sent = false;

        if (link->frame_count != g_last_frame_count)
        {
            int16_t raw_error =
                (int16_t)(link->dx - g_target_offset_pixels);
            g_last_frame_count = link->frame_count;

            if (!g_filter_valid)
            {
                g_filtered_error = raw_error;
                g_filter_valid = true;
            }
            else
            {
                if (raw_error >
                    g_filtered_error + STEP_CONTROL_ERROR_JUMP_MAX)
                {
                    raw_error =
                        g_filtered_error + STEP_CONTROL_ERROR_JUMP_MAX;
                }
                else if (raw_error <
                         g_filtered_error - STEP_CONTROL_ERROR_JUMP_MAX)
                {
                    raw_error =
                        g_filtered_error - STEP_CONTROL_ERROR_JUMP_MAX;
                }
                /*
                 * Keep only 25% history and accept 75% of the newest camera
                 * sample. The old 75% history filter added several camera
                 * frames of phase lag and made the ball oscillate.
                 */
                g_filtered_error = (int16_t)(
                    ((int32_t)g_filtered_error + 3L * raw_error) / 4L);
            }

            error = g_filtered_error;
            if ((error <= STEP_CONTROL_DEAD_ZONE) &&
                (error >= -STEP_CONTROL_DEAD_ZONE))
            {
                error = 0;
            }

            /*
             * The derivative is especially sensitive to detector jitter.
             * Average it with its previous value before applying Kd so
             * alternating one-frame coordinate noise cannot reverse the
             * motor command abruptly.
             */
            derivative = (int16_t)(error - g_previous_error);
            /*
             * Retain only 25% derivative history. Earlier braking is more
             * important than heavy smoothing for the underdamped ball motion.
             */
            g_filtered_derivative = (int16_t)(
                ((int32_t)g_filtered_derivative +
                 3L * derivative) / 4L);
            derivative = g_filtered_derivative;
            if (error == 0)
            {
                /*
                 * Keep the learned level correction in the dead zone. Leaking
                 * it here makes the ball drift back to the same biased point.
                 */
            }
            else if ((error <= STEP_CONTROL_INTEGRAL_ZONE) &&
                     (error >= -STEP_CONTROL_INTEGRAL_ZONE))
            {
                /*
                 * If the ball crosses the center, unload the old correction
                 * quickly instead of letting it push the ball through again.
                 */
                if (((error > 0) && (g_step_status.integral < 0)) ||
                    ((error < 0) && (g_step_status.integral > 0)))
                {
                    g_step_status.integral =
                        g_step_status.integral * 3L / 4L;
                }
                candidate_integral =
                    g_step_status.integral + error;
                if (candidate_integral > STEP_CONTROL_INTEGRAL_MAX)
                {
                    candidate_integral = STEP_CONTROL_INTEGRAL_MAX;
                }
                else if (candidate_integral <
                         -STEP_CONTROL_INTEGRAL_MAX)
                {
                    candidate_integral = -STEP_CONTROL_INTEGRAL_MAX;
                }

                unsaturated_output =
                    (int32_t)STEP_PID_KP_NUM * error +
                    (int32_t)STEP_PID_KI_NUM * candidate_integral +
                    (int32_t)STEP_PID_KD_NUM * derivative;
                unsaturated_output /= STEP_PID_GAIN_DEN;
                if (!((unsaturated_output >= STEP_CONTROL_MAX_RPM &&
                       error > 0) ||
                      (unsaturated_output <= -STEP_CONTROL_MAX_RPM &&
                       error < 0)))
                {
                    g_step_status.integral = candidate_integral;
                }
            }

            /*
             * Keep one decimal place of controller output. With integer-only
             * output, a one-count PID change moved the motor target by about
             * one degree, producing visibly stepped motion.
             */
            output = (int32_t)STEP_PID_KP_NUM * error +
                     (int32_t)STEP_PID_KI_NUM *
                         g_step_status.integral +
                     (int32_t)STEP_PID_KD_NUM * derivative;
            output /= (STEP_PID_GAIN_DEN / 10);

            /*
             * Overcome static friction only when the ball is nearly stopped
             * outside the position dead zone. Once motion is detected, the
             * normal PID output takes over immediately.
             */
            if ((derivative <= STEP_CONTROL_STICTION_DERIVATIVE_MAX) &&
                (derivative >= -STEP_CONTROL_STICTION_DERIVATIVE_MAX))
            {
                if ((error > 0) &&
                    (output < STEP_CONTROL_STICTION_OUTPUT_TENTHS))
                {
                    output = STEP_CONTROL_STICTION_OUTPUT_TENTHS;
                }
                else if ((error < 0) &&
                         (output > -STEP_CONTROL_STICTION_OUTPUT_TENTHS))
                {
                    output = -STEP_CONTROL_STICTION_OUTPUT_TENTHS;
                }
            }
            g_step_status.error = error;
            g_output_tenths = StepControl_Clamp(
                output, STEP_CONTROL_MAX_RPM * 10);
            g_step_status.output_rpm = g_output_tenths / 10;
            g_previous_error = error;
            control_updated = true;
        }
    }
    else if (K230_Link_IsValid(500U) &&
             (g_lost_target_ticks < STEP_CONTROL_LOST_HOLD_TICKS))
    {
        /*
         * Match the proven STM32 controller: retain the previous command
         * across one or two missed camera frames (80 ms maximum).
         */
        g_lost_target_ticks++;
        g_step_status.link_valid = true;
    }
    else
    {
        g_step_status.link_valid = false;
        g_step_status.error = 0;
        g_step_status.output_rpm = 0;
        g_step_status.integral = 0;
        g_previous_error = 0;
        g_filtered_derivative = 0;
        g_output_tenths = 0;
        g_filter_valid = false;
        g_lost_target_ticks = STEP_CONTROL_LOST_HOLD_TICKS;
    }

    if (g_step_status.link_valid && control_updated)
    {
        int32_t minimum_position;
        int32_t maximum_position;

        if (ZDT_CAN_GetMotorLimits(
                STEP_MOTOR_ADDRESS, &minimum_position,
                &maximum_position))
        {
            int32_t span = maximum_position - minimum_position;
            int32_t level_position =
                minimum_position + STEP_LEVEL_OFFSET_TENTH_DEG;
            int32_t target;

            if (level_position < minimum_position)
            {
                level_position = minimum_position;
            }
            else if (level_position > maximum_position)
            {
                level_position = maximum_position;
            }

            if (g_output_tenths >= 0)
            {
                target = level_position -
                    ((int32_t)g_output_tenths *
                     (level_position - minimum_position)) /
                    STEP_POSITION_FULL_OUTPUT_TENTHS;
            }
            else
            {
                target = level_position +
                    ((int32_t)-g_output_tenths *
                     (maximum_position - level_position)) /
                    STEP_POSITION_FULL_OUTPUT_TENTHS;
            }

            if (target < minimum_position)
            {
                target = minimum_position;
            }
            else if (target > maximum_position)
            {
                target = maximum_position;
            }

            if (!g_target_position_valid ||
                (target - g_last_target_position >=
                 STEP_CONTROL_POSITION_EPSILON) ||
                (g_last_target_position - target >=
                 STEP_CONTROL_POSITION_EPSILON))
            {
                g_last_target_position = target;
                g_target_position_valid = true;
            }
        }
    }
    else if (!g_step_status.link_valid && !g_loss_stop_sent)
    {
        X_V2_Stop_Now(STEP_MOTOR_ADDRESS, false);
        g_loss_stop_sent = true;
        g_target_position_valid = false;
        g_command_position_valid = false;
        g_position_command_ticks = 0U;
    }

    if (g_step_status.link_valid && g_target_position_valid)
    {
        StepControl_UpdateMotorPosition();
    }
}

void StepControl_Exit(void)
{
    g_active = false;
    X_V2_Stop_Now(STEP_MOTOR_ADDRESS, false);
    StepControl_Init();
}

StepControl_Status StepControl_GetStatus(void)
{
    return g_step_status;
}
