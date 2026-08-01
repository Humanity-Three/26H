#include "Step_Control.h"

#include "K230_link.h"
#include "X_V2.h"
#include "zdt_can_port.h"

#define STEP_MOTOR_ADDRESS          (1U)
#define STEP_MOTOR_POSITION_RPM     (60.0f)
#define STEP_MOTOR_POSITION_SLEW    (20L)
#define STEP_MOTOR_OPEN_LOOP_POSITION_SLEW (60L)
#define STEP_MOTOR_POSITION_TICKS   (1U)
#define STEP_CONTROL_DEAD_ZONE      (4)
#define STEP_CONTROL_ENDPOINT_DEAD_ZONE (2)
#define STEP_CONTROL_MAX_RPM        (60)
#define STEP_CONTROL_INTEGRAL_MAX   (2000)
#define STEP_CONTROL_INTEGRAL_ZONE  (96)
#define STEP_CONTROL_INTEGRAL_LEAK_DIVISOR (64L)
#define STEP_CONTROL_LOST_HOLD_TICKS (8U)
#define STEP_CONTROL_POSITION_EPSILON (2)
#define STEP_CONTROL_TARGET_HOLD_RELEASE_PIXELS (8)
#define STEP_CONTROL_TARGET_HOLD_VELOCITY_MAX (1)
#define STEP_CONTROL_ERROR_JUMP_MAX (160)
#define STEP_CONTROL_STICTION_DERIVATIVE_MAX (2)
#define STEP_CONTROL_STICTION_RELEASE_DERIVATIVE (4)
#define STEP_CONTROL_STICTION_MIN_TENTHS (20)
#define STEP_CONTROL_STICTION_MAX_TENTHS (36)
#define STEP_CONTROL_POSITIVE_TARGET_STICTION_MIN_TENTHS (60)
#define STEP_CONTROL_POSITIVE_TARGET_STICTION_MAX_TENTHS (100)
#define STEP_CONTROL_STICTION_RAMP_TENTHS (2)
#define STEP_CONTROL_STICTION_RELEASE_TENTHS (4)
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
#define STEP_CASCADE_POSITION_KP_NUM (12L)
#define STEP_CASCADE_POSITION_KP_NEAR_NUM (2L)
#define STEP_CASCADE_POSITION_KP_POSITIVE_APPROACH_NUM (20L)
#define STEP_CASCADE_POSITION_KP_POSITIVE_NEAR_NUM (15L)
#define STEP_CASCADE_POSITION_NEAR_PIXELS (18)
#define STEP_CASCADE_POSITION_LEAD_FRAMES (2L)
#define STEP_CASCADE_POSITION_KI_DIV (120L)
#define STEP_CASCADE_POSITION_INTEGRAL_ZONE (32)
#define STEP_CASCADE_TARGET_VELOCITY_X10_MAX (100)
#define STEP_CASCADE_NEGATIVE_APPROACH_VELOCITY_X10_MAX (25)
#define STEP_CASCADE_SPEED_KP_NUM   (110L)
#define STEP_CASCADE_SPEED_KP_AWAY_NUM (220L)
#define STEP_CASCADE_SPEED_KI_NUM   (0L)
#define STEP_CASCADE_SPEED_KD_NUM   (140L)
#define STEP_CASCADE_SPEED_GAIN_DEN (100L)
#define STEP_CASCADE_SPEED_INTEGRAL_MAX (2000L)
#define STEP_CASCADE_OUTPUT_MAX_TENTHS (360L)

/* Global closed-loop velocity/position coupled capture profile. */
#define STEP_PROFILE_KP_AWAY_NUM    (170)
#define STEP_PROFILE_KD_BRAKE_NUM   (3300)
#define STEP_PROFILE_BRAKE_ACCEL    (3L)
#define STEP_PROFILE_NEAR_PIXELS    (20)
#define STEP_PROFILE_MIDDLE_PIXELS  (48)
#define STEP_PROFILE_FAR_PIXELS     (96)
#define STEP_PROFILE_NEAR_LIMIT_TENTHS   (80)
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
static int16_t g_stiction_tenths;
static int8_t g_stiction_direction;
static bool g_stiction_active;
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
static bool g_target_hold_active;
static bool g_suppress_speed_derivative_once;
static bool g_active;

static int32_t StepControl_ApplyStiction(
    int32_t output, int16_t error, int16_t velocity_pixels_per_frame)
{
    int8_t requested_direction = 0;
    int16_t absolute_velocity = velocity_pixels_per_frame;
    int16_t stiction_min_tenths = STEP_CONTROL_STICTION_MIN_TENTHS;
    int16_t stiction_max_tenths = STEP_CONTROL_STICTION_MAX_TENTHS;

    if (absolute_velocity < 0)
        absolute_velocity = (int16_t)-absolute_velocity;

    if ((g_target_offset_pixels > 0) && (error < 0))
    {
        stiction_min_tenths =
            STEP_CONTROL_POSITIVE_TARGET_STICTION_MIN_TENTHS;
        stiction_max_tenths =
            STEP_CONTROL_POSITIVE_TARGET_STICTION_MAX_TENTHS;
    }

    /*
     * Once the ball is inside the position dead zone, release the dynamic
     * breakaway boost immediately. Letting it decay over several camera
     * frames keeps pushing the single-ended lift through center and creates
     * a small limit cycle. The learned position integral remains untouched.
     */
    if (error == 0)
    {
        g_stiction_tenths = 0;
        g_stiction_direction = 0;
        g_stiction_active = false;
        return output;
    }

    if (error != 0)
    {
        if (output > 0)
            requested_direction = 1;
        else if (output < 0)
            requested_direction = -1;
        else
            requested_direction = (error > 0) ? 1 : -1;
    }

    if ((requested_direction != 0) &&
        (absolute_velocity <= STEP_CONTROL_STICTION_DERIVATIVE_MAX))
    {
        if ((g_stiction_direction != 0) &&
            (requested_direction != g_stiction_direction) &&
            (g_stiction_tenths > 0))
        {
            /*
             * Never add the old-direction breakaway boost to a newly
             * reversed controller output. Clear it immediately; the new
             * direction may rebuild compensation on the next camera frame.
             */
            g_stiction_active = false;
            g_stiction_tenths = 0;
            g_stiction_direction = 0;
        }
        else
        {
            g_stiction_direction = requested_direction;
            g_stiction_active = true;
            if (g_stiction_tenths < stiction_min_tenths)
            {
                g_stiction_tenths = stiction_min_tenths;
            }
            else if (g_stiction_tenths < stiction_max_tenths)
            {
                g_stiction_tenths +=
                    STEP_CONTROL_STICTION_RAMP_TENTHS;
                if (g_stiction_tenths > stiction_max_tenths)
                {
                    g_stiction_tenths = stiction_max_tenths;
                }
            }
        }
    }
    else if ((error == 0) ||
             (absolute_velocity >=
              STEP_CONTROL_STICTION_RELEASE_DERIVATIVE))
    {
        g_stiction_active = false;
    }

    if (!g_stiction_active && (g_stiction_tenths > 0))
    {
        g_stiction_tenths -= STEP_CONTROL_STICTION_RELEASE_TENTHS;
        if (g_stiction_tenths <= 0)
        {
            g_stiction_tenths = 0;
            g_stiction_direction = 0;
        }
    }

    return output +
        (int32_t)g_stiction_direction * g_stiction_tenths;
}

static int32_t StepControl_CalculateCascade(
    int16_t error, int16_t velocity_pixels_per_frame)
{
    int32_t target_velocity_x10;
    int32_t target_velocity_limit_x10;
    int32_t position_kp_num;
    int32_t predicted_error;
    int32_t speed_error_x10;
    int32_t speed_derivative_x10;
    int32_t speed_kp_num;
    int32_t output_tenths;

    /*
     * Hold the learned position integral inside the dead zone. This mechanism
     * has one hinged end and one motor-lifted end, so its true level position
     * generally needs a persistent static offset. Leaking the integral at
     * zero error repeatedly removed that offset and biased the limit cycle.
     */
    if ((error != 0) &&
        (error <= STEP_CASCADE_POSITION_INTEGRAL_ZONE) &&
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

    /*
     * Predict where the ball will be after the camera/actuator delay. When an
     * approaching ball would cross center within the lead horizon, the
     * predicted error changes sign and asks the velocity loop to brake before
     * the measured position itself crosses center. This preserves strong
     * far-field capture without relying on a very soft near-center gain.
     */
    predicted_error = (int32_t)error +
        STEP_CASCADE_POSITION_LEAD_FRAMES *
            velocity_pixels_per_frame;

    position_kp_num = STEP_CASCADE_POSITION_KP_NUM;
    if ((g_target_offset_pixels > 0) && (error < 0))
    {
        /*
         * The lifted-end direction needs more rail angle to overcome its
         * mechanical bias. Apply this gain through the complete +5 cm
         * approach; limiting it to the near zone can leave the ball stopped
         * roughly 2 cm short of the endpoint.
         */
        position_kp_num =
            STEP_CASCADE_POSITION_KP_POSITIVE_APPROACH_NUM;
    }
    if ((predicted_error <= STEP_CASCADE_POSITION_NEAR_PIXELS) &&
        (predicted_error >= -STEP_CASCADE_POSITION_NEAR_PIXELS))
    {
        if ((g_target_offset_pixels > 0) && (error < 0))
        {
            position_kp_num =
                STEP_CASCADE_POSITION_KP_POSITIVE_NEAR_NUM;
        }
        else
        {
            position_kp_num = STEP_CASCADE_POSITION_KP_NEAR_NUM;
        }
    }

    /* Positive position error requires a negative target ball velocity. */
    target_velocity_x10 =
        -((position_kp_num * predicted_error) / 10L +
          g_step_status.integral / STEP_CASCADE_POSITION_KI_DIV);
    target_velocity_limit_x10 = STEP_CASCADE_TARGET_VELOCITY_X10_MAX;
    if ((g_target_offset_pixels < 0) && (error > 0))
    {
        /*
         * While approaching a negative endpoint from the center side, limit
         * commanded ball speed directly instead of changing actuator gains.
         * This reduces arrival energy without depending on motor polarity.
         * Once the ball crosses the target, full recovery speed is restored.
         */
        target_velocity_limit_x10 =
            STEP_CASCADE_NEGATIVE_APPROACH_VELOCITY_X10_MAX;
    }
    if (target_velocity_x10 > target_velocity_limit_x10)
        target_velocity_x10 = target_velocity_limit_x10;
    else if (target_velocity_x10 < -target_velocity_limit_x10)
        target_velocity_x10 = -target_velocity_limit_x10;

    /* Positive speed error requires a positive corrective rail output. */
    speed_error_x10 =
        (int32_t)velocity_pixels_per_frame * 10L - target_velocity_x10;
    if (g_suppress_speed_derivative_once)
    {
        speed_derivative_x10 = 0;
        g_suppress_speed_derivative_once = false;
    }
    else
    {
        speed_derivative_x10 =
            speed_error_x10 - g_previous_speed_error_x10;
    }
    g_previous_speed_error_x10 = (int16_t)speed_error_x10;

    speed_kp_num = STEP_CASCADE_SPEED_KP_NUM;
    if (((error > 0) && (velocity_pixels_per_frame > 0)) ||
        ((error < 0) && (velocity_pixels_per_frame < 0)))
    {
        speed_kp_num = STEP_CASCADE_SPEED_KP_AWAY_NUM;
    }
    g_speed_integral_x10 += speed_error_x10;
    if (g_speed_integral_x10 > STEP_CASCADE_SPEED_INTEGRAL_MAX)
        g_speed_integral_x10 = STEP_CASCADE_SPEED_INTEGRAL_MAX;
    else if (g_speed_integral_x10 < -STEP_CASCADE_SPEED_INTEGRAL_MAX)
        g_speed_integral_x10 = -STEP_CASCADE_SPEED_INTEGRAL_MAX;

    output_tenths =
         (speed_kp_num * speed_error_x10 +
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
    int32_t position_slew;
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
    position_slew = g_open_loop_active ?
        STEP_MOTOR_OPEN_LOOP_POSITION_SLEW : STEP_MOTOR_POSITION_SLEW;

    if (g_last_target_position >
        g_command_position + position_slew)
    {
        g_command_position += position_slew;
    }
    else if (g_last_target_position <
             g_command_position - position_slew)
    {
        g_command_position -= position_slew;
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
    g_stiction_tenths = 0;
    g_stiction_direction = 0;
    g_stiction_active = false;
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
    g_target_hold_active = false;
    g_suppress_speed_derivative_once = true;
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
    g_stiction_tenths = 0;
    g_stiction_direction = 0;
    g_stiction_active = false;
    g_output_tenths = 0;
    g_filter_valid = false;
    g_target_hold_active = false;
    g_suppress_speed_derivative_once = true;
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
    int16_t control_dead_zone;
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
                 * Accept 75% of the newest camera sample. The remaining 25%
                 * history rejects enough coordinate jitter for the higher
                 * cascade gains without adding several frames of phase lag.
                 */
                g_filtered_error = (int16_t)(
                    ((int32_t)g_filtered_error + 3L * raw_error) / 4L);
            }

            error = g_filtered_error;
            control_dead_zone = (g_target_offset_pixels == 0) ?
                STEP_CONTROL_DEAD_ZONE :
                STEP_CONTROL_ENDPOINT_DEAD_ZONE;
            if ((error <= control_dead_zone) &&
                (error >= -control_dead_zone))
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

            /*
             * Once any commanded target is reached at very low speed, freeze
             * the current motor position. Hysteresis prevents camera jitter
             * and rack backlash from restarting alternating corrections. No
             * frame-count delay is required; a real displacement immediately
             * outside the release band wakes the cascade controller.
             */
            if (g_target_hold_active)
            {
                if ((g_filtered_error >
                     STEP_CONTROL_TARGET_HOLD_RELEASE_PIXELS) ||
                    (g_filtered_error <
                     -STEP_CONTROL_TARGET_HOLD_RELEASE_PIXELS))
                {
                    g_target_hold_active = false;
                    g_filtered_derivative = 0;
                    derivative = 0;
                    g_previous_error = error;
                    g_previous_speed_error_x10 = 0;
                    g_suppress_speed_derivative_once = true;
                }
            }
            /*
             * Do not latch endpoint targets. BALANCE keeps running its
             * closed loop after the stopwatch has stopped; freezing the
             * actuator at the instant the ball is manually placed on +5 cm
             * preserves the old biased rail angle and lets the ball roll
             * back to that equilibrium point. Continuous endpoint control
             * instead maintains the requested ball coordinate.
             */

            if (g_target_hold_active)
            {
                g_filtered_derivative = 0;
                g_previous_error = error;
                g_previous_speed_error_x10 = 0;
                g_step_status.error = 0;
                g_step_status.output_rpm = 0;
                if (g_target_position_valid)
                {
                    StepControl_UpdateMotorPosition();
                }
                return;
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

            output = StepControl_ApplyStiction(
                output, error, derivative);
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
        g_stiction_tenths = 0;
        g_stiction_direction = 0;
        g_stiction_active = false;
        g_target_hold_active = false;
        g_suppress_speed_derivative_once = true;
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
