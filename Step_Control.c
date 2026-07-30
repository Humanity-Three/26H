#include "Step_Control.h"

#include "K230_link.h"
#include "X_V2.h"
#include "zdt_can_port.h"

#define STEP_MOTOR_ADDRESS          (1U)
#define STEP_MOTOR_ACCEL_RPM_S      (4000U)
#define STEP_MOTOR_MAX_POSITION_RPM (60.0f)
#define STEP_CONTROL_DEAD_ZONE      (4)
#define STEP_CONTROL_MAX_RPM        (60)
#define STEP_CONTROL_INTEGRAL_MAX   (600)
#define STEP_CONTROL_LOST_HOLD_TICKS (8U)
#define STEP_CONTROL_POSITION_EPSILON (12)
#define STEP_CONTROL_ERROR_JUMP_MAX (160)
#define STEP_LEVEL_OFFSET_TENTH_DEG (611L)
#define STEP_CONTROL_REACHED_WINDOW (10L)

/*
 * Damped balance baseline for the ~23 FPS camera:
 * Kp=0.14, Ki=0, Kd=0.32.
 * Integral is disabled while dynamic balance is tuned because accumulated
 * bias keeps tilting the track after the ball has already crossed center.
 */
#define STEP_PID_KP_NUM             (140)
#define STEP_PID_KI_NUM             (0)
#define STEP_PID_KD_NUM             (320)
#define STEP_PID_GAIN_DEN           (1000)

static StepControl_Status g_step_status;
static int16_t g_previous_error;
static int16_t g_filtered_error;
static int16_t g_filtered_derivative;
static uint8_t g_lost_target_ticks;
static uint32_t g_last_frame_count;
static int32_t g_last_target_position;
static int32_t g_pending_target_position;
static bool g_filter_valid;
static bool g_target_position_valid;
static bool g_pending_target_valid;
static bool g_loss_stop_sent;
static bool g_active;

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

static void StepControl_SendPosition(int32_t target_tenth_degree)
{
    uint8_t direction =
        (target_tenth_degree < 0) ? 1U : 0U;
    float position_degree =
        (target_tenth_degree < 0) ?
        (float)-target_tenth_degree / 10.0f :
        (float)target_tenth_degree / 10.0f;

    X_V2_Traj_Pos_Control(
        STEP_MOTOR_ADDRESS, direction,
        STEP_MOTOR_ACCEL_RPM_S, STEP_MOTOR_ACCEL_RPM_S,
        STEP_MOTOR_MAX_POSITION_RPM, position_degree,
        1U, false);
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
    g_lost_target_ticks = 0U;
    g_last_frame_count = 0U;
    g_last_target_position = 0;
    g_pending_target_position = 0;
    g_filter_valid = false;
    g_target_position_valid = false;
    g_pending_target_valid = false;
    g_loss_stop_sent = false;
}

void StepControl_Enter(void)
{
    StepControl_Init();
    g_active = true;
    X_V2_En_Control(STEP_MOTOR_ADDRESS, true, false);
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

    link = K230_Link_GetData();
    if (K230_Link_IsValid(500U) && link->detected)
    {
        g_step_status.link_valid = true;
        g_lost_target_ticks = 0U;
        g_loss_stop_sent = false;

        if (link->frame_count != g_last_frame_count)
        {
            int16_t raw_error = link->dx;
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
            g_filtered_derivative = (int16_t)(
                ((int32_t)g_filtered_derivative + derivative) / 2L);
            derivative = g_filtered_derivative;
            if (error == 0)
            {
                /* Retain bias smoothly instead of discontinuously clearing it. */
                g_step_status.integral =
                    g_step_status.integral * 99L / 100L;
            }
            else
            {
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

            output = (int32_t)STEP_PID_KP_NUM * error +
                     (int32_t)STEP_PID_KI_NUM *
                         g_step_status.integral +
                     (int32_t)STEP_PID_KD_NUM * derivative;
            output /= STEP_PID_GAIN_DEN;
            g_step_status.error = error;
            g_step_status.output_rpm =
                StepControl_Clamp(output, STEP_CONTROL_MAX_RPM);
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

            if (g_step_status.output_rpm >= 0)
            {
                target = level_position -
                    ((int32_t)g_step_status.output_rpm *
                     (level_position - minimum_position)) /
                    STEP_CONTROL_MAX_RPM;
            }
            else
            {
                target = level_position +
                    ((int32_t)-g_step_status.output_rpm *
                     (maximum_position - level_position)) /
                    STEP_CONTROL_MAX_RPM;
            }

            if (target < minimum_position)
            {
                target = minimum_position;
            }
            else if (target > maximum_position)
            {
                target = maximum_position;
            }

            if (!g_target_position_valid)
            {
                StepControl_SendPosition(target);
                g_last_target_position = target;
                g_target_position_valid = true;
            }
            else if ((target - g_last_target_position >=
                      STEP_CONTROL_POSITION_EPSILON) ||
                     (g_last_target_position - target >=
                      STEP_CONTROL_POSITION_EPSILON))
            {
                int32_t current_position;
                uint32_t position_sequence;
                int32_t position_error;

                /*
                 * Keep only the newest requested target while the current FD
                 * trajectory is running. Replacing FD on every camera frame
                 * repeatedly restarts the motor's acceleration planner.
                 */
                g_pending_target_position = target;
                g_pending_target_valid = true;

                if (ZDT_CAN_GetMotorPosition(
                        STEP_MOTOR_ADDRESS, &current_position,
                        &position_sequence))
                {
                    (void)position_sequence;
                    position_error =
                        current_position - g_last_target_position;
                    if ((position_error <=
                         STEP_CONTROL_REACHED_WINDOW) &&
                        (position_error >=
                         -STEP_CONTROL_REACHED_WINDOW))
                    {
                        StepControl_SendPosition(
                            g_pending_target_position);
                        g_last_target_position =
                            g_pending_target_position;
                        g_pending_target_valid = false;
                    }
                }
            }
            else
            {
                /* The newest demand returned to the active target. */
                g_pending_target_valid = false;
            }
        }
    }
    else if (!g_loss_stop_sent)
    {
        X_V2_Stop_Now(STEP_MOTOR_ADDRESS, false);
        g_loss_stop_sent = true;
        g_target_position_valid = false;
        g_pending_target_valid = false;
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
