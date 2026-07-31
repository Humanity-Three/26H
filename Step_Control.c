#include "Step_Control.h"

#include "K230_link.h"
#include "X_V2.h"
#include "zdt_can_port.h"

#define STEP_MOTOR_ADDRESS          (1U)
#define STEP_MOTOR_POSITION_RPM     (60.0f)
#define STEP_MOTOR_POSITION_SLEW    (16L)
#define STEP_MOTOR_POSITION_TICKS   (1U)
#define STEP_CONTROL_DEAD_ZONE      (4)
#define STEP_CONTROL_MAX_RPM        (60)
#define STEP_CONTROL_INTEGRAL_MAX   (2000)
#define STEP_CONTROL_INTEGRAL_ZONE  (96)
#define STEP_CONTROL_INTEGRAL_LEAK_DIVISOR (64L)
#define STEP_CONTROL_LOST_HOLD_TICKS (8U)
#define STEP_CONTROL_POSITION_EPSILON (2)
#define STEP_CONTROL_ERROR_JUMP_MAX (160)
#define STEP_CONTROL_STICTION_DERIVATIVE_MAX (2)
#define STEP_CONTROL_STICTION_OUTPUT_TENTHS (26)
#define STEP_POSITION_FULL_OUTPUT_TENTHS (300L)

/*
 * Transient-stability tuning baseline for the ~23 FPS camera:
 * Kp=0.13, Ki=0.002, Kd=2.20.
 * Keeping the derivative below its frequent saturation region avoids
 * bang-bang reversals. A weak integral removes residual static position bias
 * without materially changing the transient response.
 */
#define STEP_PID_KP_NUM             (130)
#define STEP_PID_KI_NUM             (2)
#define STEP_PID_KD_NUM             (2200)
#define STEP_PID_GAIN_DEN           (1000)

/*
 * Experimental cascaded controller:
 * outer position PI -> target ball velocity,
 * inner velocity PID -> rail/motor output.
 * Set to 0 to restore the proven single-loop controller below.
 */
#define STEP_CONTROL_USE_CASCADE    (1)
#define STEP_CASCADE_POSITION_KP_NUM (8L)
#define STEP_CASCADE_POSITION_KI_DIV (500L)
#define STEP_CASCADE_POSITION_INTEGRAL_ZONE (32)
#define STEP_CASCADE_TARGET_VELOCITY_X10_MAX (80)
#define STEP_CASCADE_SPEED_KP_NUM   (100L)
#define STEP_CASCADE_SPEED_KI_NUM   (0L)
#define STEP_CASCADE_SPEED_KD_NUM   (90L)
#define STEP_CASCADE_SPEED_GAIN_DEN (100L)
#define STEP_CASCADE_SPEED_INTEGRAL_MAX (2000L)
#define STEP_CASCADE_OUTPUT_MAX_TENTHS (300L)

/* Global closed-loop velocity/position coupled capture profile. */
#define STEP_PROFILE_KP_AWAY_NUM    (170)
#define STEP_PROFILE_KD_BRAKE_NUM   (3300)
#define STEP_PROFILE_BRAKE_ACCEL    (3L)
#define STEP_PROFILE_NEAR_PIXELS    (20)
#define STEP_PROFILE_MIDDLE_PIXELS  (48)
#define STEP_PROFILE_FAR_PIXELS     (96)
#define STEP_PROFILE_NEAR_LIMIT_TENTHS   (100)
#define STEP_PROFILE_MIDDLE_LIMIT_TENTHS (180)
#define STEP_PROFILE_FAR_LIMIT_TENTHS    (300)
#define STEP_PROFILE_MAX_LIMIT_TENTHS    (450)

static StepControl_Status g_step_status;
static int16_t g_previous_error;
static int16_t g_filtered_error;
static int16_t g_filtered_derivative;
static int16_t g_target_offset_pixels;
static int16_t g_output_tenths;
static int16_t g_feedforward_tenths;
static int32_t g_speed_integral_x10;
static int16_t g_previous_speed_error_x10;
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
static bool g_velocity_profile_enabled;
static bool g_active;

static int32_t StepControl_CalculateCascade(
    int16_t error, int16_t velocity_pixels_per_frame)
{
    int32_t target_velocity_x10;
    int32_t speed_error_x10;
    int32_t speed_derivative_x10;
    int32_t output_tenths;

    if (error == 0)
    {
        if (g_step_status.integral > 0)
        {
            g_step_status.integral -=
                (g_step_status.integral +
                 STEP_CONTROL_INTEGRAL_LEAK_DIVISOR - 1L) /
                STEP_CONTROL_INTEGRAL_LEAK_DIVISOR;
        }
        else if (g_step_status.integral < 0)
        {
            g_step_status.integral +=
                (-g_step_status.integral +
                 STEP_CONTROL_INTEGRAL_LEAK_DIVISOR - 1L) /
                STEP_CONTROL_INTEGRAL_LEAK_DIVISOR;
        }
    }
    else if ((error <= STEP_CASCADE_POSITION_INTEGRAL_ZONE) &&
             (error >= -STEP_CASCADE_POSITION_INTEGRAL_ZONE) &&
             (velocity_pixels_per_frame <= 2) &&
             (velocity_pixels_per_frame >= -2))
    {
        g_step_status.integral += error;
        if (g_step_status.integral > STEP_CONTROL_INTEGRAL_MAX)
            g_step_status.integral = STEP_CONTROL_INTEGRAL_MAX;
        else if (g_step_status.integral < -STEP_CONTROL_INTEGRAL_MAX)
            g_step_status.integral = -STEP_CONTROL_INTEGRAL_MAX;
    }

    /* Positive position error requires a negative target ball velocity. */
    target_velocity_x10 =
        -((STEP_CASCADE_POSITION_KP_NUM * error) / 10L +
          g_step_status.integral / STEP_CASCADE_POSITION_KI_DIV);
    if (target_velocity_x10 > STEP_CASCADE_TARGET_VELOCITY_X10_MAX)
        target_velocity_x10 = STEP_CASCADE_TARGET_VELOCITY_X10_MAX;
    else if (target_velocity_x10 <
             -STEP_CASCADE_TARGET_VELOCITY_X10_MAX)
        target_velocity_x10 = -STEP_CASCADE_TARGET_VELOCITY_X10_MAX;

    /* Positive speed error requires a positive corrective rail output. */
    speed_error_x10 =
        (int32_t)velocity_pixels_per_frame * 10L - target_velocity_x10;
    speed_derivative_x10 =
        speed_error_x10 - g_previous_speed_error_x10;
    g_previous_speed_error_x10 = (int16_t)speed_error_x10;

    g_speed_integral_x10 += speed_error_x10;
    if (g_speed_integral_x10 > STEP_CASCADE_SPEED_INTEGRAL_MAX)
        g_speed_integral_x10 = STEP_CASCADE_SPEED_INTEGRAL_MAX;
    else if (g_speed_integral_x10 < -STEP_CASCADE_SPEED_INTEGRAL_MAX)
        g_speed_integral_x10 = -STEP_CASCADE_SPEED_INTEGRAL_MAX;

    output_tenths =
        (STEP_CASCADE_SPEED_KP_NUM * speed_error_x10 +
         STEP_CASCADE_SPEED_KI_NUM * g_speed_integral_x10 +
         STEP_CASCADE_SPEED_KD_NUM * speed_derivative_x10) /
        STEP_CASCADE_SPEED_GAIN_DEN;
    if (output_tenths > STEP_CASCADE_OUTPUT_MAX_TENTHS)
        output_tenths = STEP_CASCADE_OUTPUT_MAX_TENTHS;
    else if (output_tenths < -STEP_CASCADE_OUTPUT_MAX_TENTHS)
        output_tenths = -STEP_CASCADE_OUTPUT_MAX_TENTHS;
    return output_tenths + g_feedforward_tenths;
}

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
    g_step_status.raw_dx = 0;
    g_step_status.target_offset = 0;
    g_step_status.error = 0;
    g_step_status.output_rpm = 0;
    g_step_status.integral = 0;
    g_step_status.link_valid = false;
    g_previous_error = 0;
    g_filtered_error = 0;
    g_filtered_derivative = 0;
    g_speed_integral_x10 = 0;
    g_previous_speed_error_x10 = 0;
    g_target_offset_pixels = 0;
    g_output_tenths = 0;
    g_feedforward_tenths = 0;
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
    /*
     * Use the proven velocity/position capture curve in every closed-loop
     * mode (BALANCE, AB and ABCDA). Open-loop transfer commands bypass this
     * path, so the two fast BALANCE travel stages remain unchanged.
     */
    g_velocity_profile_enabled = true;
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
    g_step_status.target_offset = offset_pixels;
    g_step_status.error = 0;
    g_step_status.integral = 0;
    g_previous_error = 0;
    g_filtered_error = 0;
    g_filtered_derivative = 0;
    g_speed_integral_x10 = 0;
    g_previous_speed_error_x10 = 0;
    g_output_tenths = 0;
    g_filter_valid = false;
}

void StepControl_EnableVelocityProfile(bool enable)
{
    g_velocity_profile_enabled = enable;
}

void StepControl_SetFeedforwardTenths(int16_t output_tenths)
{
    g_feedforward_tenths = StepControl_Clamp(
        output_tenths, STEP_CONTROL_MAX_RPM * 10);
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
    level_position = STEP_CONTROL_LEVEL_POSITION_TENTHS;
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
    g_velocity_profile_enabled = false;
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
    g_velocity_profile_enabled = false;
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
    int32_t position_gain_num;
    int32_t derivative_gain_num;
    int16_t profile_output_limit_tenths;
    bool control_updated = false;
    bool first_filtered_sample = false;

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
        g_step_status.raw_dx = link->dx;
        g_step_status.target_offset = g_target_offset_pixels;
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
                first_filtered_sample = true;
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
            /*
             * Do not interpret the initial position error as ball velocity.
             * Otherwise entering closed loop produces a large derivative kick,
             * especially with the higher damping gain.
             */
            derivative = first_filtered_sample ? 0 :
                (int16_t)(error - g_previous_error);
            /*
             * Retain only 25% derivative history. Earlier braking is more
             * important than heavy smoothing for the underdamped ball motion.
             */
            g_filtered_derivative = (int16_t)(
                ((int32_t)g_filtered_derivative +
                 3L * derivative) / 4L);
            derivative = g_filtered_derivative;

            /*
             * Once position is inside the dead zone, discard derivative
             * history immediately. Letting filtered velocity decay over
             * several frames keeps moving the rail after the ball is already
             * centered and produces visible small-amplitude jitter.
             */
            if ((error == 0) &&
                ((STEP_CONTROL_USE_CASCADE == 0) ||
                 ((derivative <= STEP_CONTROL_STICTION_DERIVATIVE_MAX) &&
                  (derivative >= -STEP_CONTROL_STICTION_DERIVATIVE_MAX))))
            {
                g_filtered_derivative = 0;
                derivative = 0;
            }

            position_gain_num = STEP_PID_KP_NUM;
            derivative_gain_num = STEP_PID_KD_NUM;
            profile_output_limit_tenths = STEP_CONTROL_MAX_RPM * 10;
            if (g_velocity_profile_enabled)
            {
                int32_t absolute_error =
                    (error < 0) ? -(int32_t)error : (int32_t)error;
                int32_t absolute_velocity =
                    (derivative < 0) ?
                    -(int32_t)derivative : (int32_t)derivative;
                int32_t stopping_distance =
                    (absolute_velocity * absolute_velocity) /
                    (2L * STEP_PROFILE_BRAKE_ACCEL);

                if (((error > 0) && (derivative > 0)) ||
                    ((error < 0) && (derivative < 0)))
                {
                    /* The ball is moving away: apply a firmer recovery pull. */
                    position_gain_num = STEP_PROFILE_KP_AWAY_NUM;
                }
                else if ((((error > 0) && (derivative < 0)) ||
                          ((error < 0) && (derivative > 0))) &&
                         (stopping_distance >= absolute_error))
                {
                    /* Approaching too fast: brake before crossing the target. */
                    derivative_gain_num = STEP_PROFILE_KD_BRAKE_NUM;
                }

                if (absolute_error <= STEP_PROFILE_NEAR_PIXELS)
                {
                    profile_output_limit_tenths =
                        STEP_PROFILE_NEAR_LIMIT_TENTHS;
                }
                else if (absolute_error <= STEP_PROFILE_MIDDLE_PIXELS)
                {
                    profile_output_limit_tenths =
                        STEP_PROFILE_MIDDLE_LIMIT_TENTHS;
                }
                else if (absolute_error <= STEP_PROFILE_FAR_PIXELS)
                {
                    profile_output_limit_tenths =
                        STEP_PROFILE_FAR_LIMIT_TENTHS;
                }
                else
                {
                    profile_output_limit_tenths =
                        STEP_PROFILE_MAX_LIMIT_TENTHS;
                }
            }
#if STEP_CONTROL_USE_CASCADE
            output = StepControl_CalculateCascade(error, derivative);
#else
            if (error == 0)
            {
                /*
                 * Slowly release the learned tilt while the ball is inside
                 * the dead zone. This lets the rail approach mechanical level
                 * instead of permanently holding a friction-supported ball at
                 * an old integral tilt. Any renewed drift rebuilds the needed
                 * correction after the error leaves the dead zone.
                 */
                if (g_step_status.integral > 0)
                {
                    g_step_status.integral -=
                        (g_step_status.integral +
                         STEP_CONTROL_INTEGRAL_LEAK_DIVISOR - 1L) /
                        STEP_CONTROL_INTEGRAL_LEAK_DIVISOR;
                }
                else if (g_step_status.integral < 0)
                {
                    g_step_status.integral +=
                        (-g_step_status.integral +
                         STEP_CONTROL_INTEGRAL_LEAK_DIVISOR - 1L) /
                        STEP_CONTROL_INTEGRAL_LEAK_DIVISOR;
                }
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
                    position_gain_num * error +
                    (int32_t)STEP_PID_KI_NUM * candidate_integral +
                    derivative_gain_num * derivative;
                unsaturated_output /= STEP_PID_GAIN_DEN;
                unsaturated_output += g_feedforward_tenths / 10;
                if (!((unsaturated_output >=
                       profile_output_limit_tenths / 10 &&
                       error > 0) ||
                      (unsaturated_output <=
                       -profile_output_limit_tenths / 10 &&
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
            output = position_gain_num * error +
                     (int32_t)STEP_PID_KI_NUM *
                         g_step_status.integral +
                     derivative_gain_num * derivative;
            output /= (STEP_PID_GAIN_DEN / 10);
            output += g_feedforward_tenths;
#endif

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
            output = StepControl_Clamp(
                output, profile_output_limit_tenths);
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
        g_speed_integral_x10 = 0;
        g_previous_speed_error_x10 = 0;
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
            int32_t level_position =
                STEP_CONTROL_LEVEL_POSITION_TENTHS;
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
