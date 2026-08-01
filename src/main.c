#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "X_V2.h"
#include "control.h"
#include "grayscale_sensor.h"
#include "encoder_motor.h"
#include "zdt_can_port.h"
#include "K230_link.h"
#include "Step_Control.h"
#include "MPU6050.h"

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm(".global __ARM_use_no_argv\n");
#endif

/*
 * Main framework only:
 * - 10 ms foreground task
 * - independent key debounce
 * - mode selection and state transitions
 * - stepper-motor actions issued once on state entry/exit
 *
 * Grayscale, encoder and other application-level behavior are intentionally
 * not connected here yet.
 */

#define OLED_HEARTBEAT_TICKS        (50U)
#define CAL_MOTOR_ADDRESS           (1U)
#define CAL_QUERY_TICKS             (2U)
#define POSITION_REFRESH_TICKS      (2U)
#define CAL_POSITION_SPAN           (1222L)
/*
 * Physical targets measured on the 800x480 CanMV display are converted to
 * the 640x480 detector/UART coordinate system. X scales by 640/800 while Y
 * is unchanged: center (385,190) -> (308,190), -5 cm (243,190) ->
 * (194,190), and +5 cm (527,190) -> (422,190).
 */
#define BALANCE_NEGATIVE_TARGET_PIXELS (-114)
#define BALANCE_POSITIVE_TARGET_PIXELS (114)
/*
 * -5 cm is display x=243 (detector x=194, dx=-114). Two centimetres before
 * that point toward center is display x~=300 (detector x~=240, dx=-68).
 */
#define BALANCE_NEGATIVE_REVERSE_DX           (-68)
/* 5 cm corresponds to 114 pixels, so 1 cm is approximately 23 pixels. */
#define BALANCE_FINAL_ARRIVAL_TOLERANCE_PIXELS (23)
#define BALANCE_FINAL_STILL_DELTA_PIXELS       (3)
#define BALANCE_FIRST_SWITCH_LEAD_PIXELS  (110)
#define BALANCE_SECOND_SWITCH_LEAD_PIXELS (120)
#define BALANCE_FINAL_ARRIVAL_FRAMES (4U)
/*
 * BALANCE open-loop transfer plus closed-loop endpoint holding.
 * Position/tilt units are 0.1 motor degree; time units are 10 ms.
 * Swap the signs of the two tilt offsets if the first travel direction is
 * opposite to the required -5 cm direction.
 */
#define BALANCE_FIRST_TILT_TENTHS     (-60L)
#define BALANCE_FIRST_TIME_TICKS      (60U)
#define BALANCE_SECOND_TILT_TENTHS    (200L)
#define BALANCE_SECOND_TIME_TICKS     (120U)
#define CIRCLE_RIGHT5_MASK          (0xF8U)
#define CIRCLE_RIGHT3_MASK          (0xE0U)
#define CIRCLE_DEPART_TICKS         (10U)
#define CIRCLE_FINISH_TICKS         (1U)
#define AB_CRUISE_PWM               (960)
#define AB_ACCEL_SLEW_X2            (7)
#define AB_DECEL_SLEW_X2            (16)
#define AB_STOP_PWM_STEP            (4)
#define ABCDA_STRAIGHT_PWM          (800)
#define ABCDA_CURVE_PWM             (540)
#define ABCDA_ACCEL_SLEW            (3)
#define ABCDA_DECEL_SLEW            (8)
#define ABCDA_TURN_SLEW             (35)
#define ABCDA_STOP_PWM_STEP         (2)
#define ABCDA_CURVE_ENTER_ERROR     (2)
#define ABCDA_CURVE_EXIT_ERROR      (1)
#define ABCDA_FINISH_CREEP_PWM      (400)
#define ABCDA_FINISH_CREEP_COUNTS   (2200L)
/* Calibrate this from A to B: average quadrature counts of both wheels. */
#define AB_TARGET_ENCODER_COUNTS    (11180L)
#define AB_DECEL_START_COUNTS       (6240L)
#define AB_APPROACH_PWM             (320)
#define AB_CONTINUOUS_TRACKING      (1)
#define AB_BRAKE_ENCODER_COUNTS     (2000L)
#define AB_STOP_RPM                 (3)
#define AB_STOP_STABLE_TICKS        (10U)
#define AB_STOP_TIMEOUT_TICKS       (150U)
#define AB_FEEDFORWARD_SIGN         (1L)
#define AB_PWM_ACCEL_FF_ACCEL_NUM   (7L)
#define AB_PWM_ACCEL_FF_BRAKE_NUM   (11L)
#define AB_ENCODER_ACCEL_FF_NUM     (11L)
#define AB_ENCODER_ACCEL_DEAD_ZONE  (1)
#define AB_FEEDFORWARD_LIMIT_TENTHS (220)
#define AB_FEEDFORWARD_SLEW_TENTHS  (4)
#define ABCDA_GYRO_FF_SIGN           (1L)
#define ABCDA_GYRO_FF_DEADZONE_X10  (18L)
#define ABCDA_GYRO_FF_GAIN_NUM       (4L)
#define ABCDA_GYRO_FF_GAIN_DEN       (10L)
#define ABCDA_GYRO_FF_LIMIT_TENTHS   (100L)
#define ABCDA_FEEDFORWARD_LIMIT_TENTHS (300L)
#define ABCDA_FEEDFORWARD_SLEW_TENTHS  (7L)

enum CAR_STATE
{
    CAR_STATE_IDLE = 0U,
    CAR_STATE_MOVECIRCLE = 1U,
    CAR_STATE_BALANCE = 2U,
    CAR_STATE_ABCDA_BALANCE_CENTER = 3U,
    CAR_STATE_AB_BALANCE = 4U,
    CAR_STATE_STOPPED = 5U
};

static enum CAR_STATE g_selected_state = CAR_STATE_MOVECIRCLE;
static enum CAR_STATE g_current_state = CAR_STATE_IDLE;
static enum CAR_STATE g_next_state = CAR_STATE_IDLE;

/* KEY1..KEY4 press events map to bit0..bit3. */
static uint8_t g_key_pressed_event;
static volatile uint8_t g_task_10ms_pending;
volatile uint32_t g_sys_tick_10ms;
static uint8_t g_oled_heartbeat;
static int32_t g_calibration_ccw_position;
static int32_t g_calibration_cw_position;
static uint32_t g_calibration_position_sequence;
static uint8_t g_calibration_query_ticks;
static bool g_initial_limit_ready;
static uint8_t g_balance_stage;
static uint8_t g_balance_arrival_frames;
static uint32_t g_balance_last_frame;
static uint32_t g_balance_stage_start_ticks;
static int16_t g_balance_previous_dx;
static bool g_balance_previous_dx_valid;
static uint32_t g_timer_10ms_ticks;
static bool g_timer_running;
static uint8_t g_circle_depart_ticks;
static uint8_t g_circle_finish_ticks;
static bool g_circle_departed;
static bool g_circle_finish_requested;
static bool g_ab_started;
static bool g_ab_stopping;
static uint8_t g_ab_stop_stable_ticks;
static uint16_t g_ab_stop_ticks;
static bool g_ab_stopped;
static int16_t g_ab_previous_average_pwm;
static int16_t g_ab_previous_average_rpm;
static int16_t g_ab_filtered_pwm_accel;
static int16_t g_ab_filtered_encoder_accel;
static int16_t g_ab_feedforward_tenths;
static int16_t g_abcda_filtered_gyro_z_x10;
static bool g_mpu6050_ready;
static bool g_abcda_curve_active;
static bool g_abcda_finish_creep_active;
static bool g_abcda_finish_creep_done;
static int32_t g_abcda_finish_creep_start_count;

static void Key_Init(void);
static void Key_Update10ms(void);
static void State_Update(void);
static void State_Enter(enum CAR_STATE state);
static void State_Exit(enum CAR_STATE state);
static void State_Operation(void);
static void Display_Update10ms(void);
static const char *Display_StateName(enum CAR_STATE state);
static void InitialLimit_Update10ms(void);
static bool State_UsesStepControl(enum CAR_STATE state);
static void Balance_TaskUpdate(void);

static bool State_UsesStepControl(enum CAR_STATE state)
{
    return (state == CAR_STATE_BALANCE) ||
           (state == CAR_STATE_ABCDA_BALANCE_CENTER) ||
           (state == CAR_STATE_AB_BALANCE);
}

static void Balance_TaskUpdate(void)
{
    const K230_LinkData *link = K230_Link_GetData();
    int16_t target;
    int16_t target_error;
    int16_t frame_delta;

    if ((g_balance_stage == 0U) || (g_balance_stage >= 5U))
    {
        return;
    }

    if (!K230_Link_IsValid(500U) || !link->detected ||
        (link->frame_count == g_balance_last_frame))
    {
        return;
    }
    g_balance_last_frame = link->frame_count;

    frame_delta = 0;
    if (g_balance_previous_dx_valid)
    {
        frame_delta = (int16_t)(link->dx - g_balance_previous_dx);
    }
    g_balance_previous_dx = link->dx;
    g_balance_previous_dx_valid = true;

    target = ((g_balance_stage == 1U) ||
              (g_balance_stage == 2U)) ?
        BALANCE_NEGATIVE_TARGET_PIXELS :
        BALANCE_POSITIVE_TARGET_PIXELS;
    target_error = (int16_t)(link->dx - target);

    if (g_balance_stage == 1U)
    {
        if ((link->dx <=
             (BALANCE_NEGATIVE_TARGET_PIXELS +
              BALANCE_FIRST_SWITCH_LEAD_PIXELS)) ||
            ((g_timer_10ms_ticks - g_balance_stage_start_ticks) >=
             BALANCE_FIRST_TIME_TICKS))
        {
            g_balance_stage = 2U;
            g_balance_arrival_frames = 0U;
            g_balance_stage_start_ticks = g_timer_10ms_ticks;
            StepControl_SetTargetOffsetPixels(
                BALANCE_NEGATIVE_TARGET_PIXELS);
            StepControl_EnableVelocityProfile(true);
        }
    }
    else if (g_balance_stage == 2U)
    {
        /*
         * -5 cm is a pass-through reversal point, not a settling point.
         * Reverse as soon as the ball enters the target window from center;
         * do not wait for low velocity or additional camera frames.
         */
        if (link->dx <= BALANCE_NEGATIVE_REVERSE_DX)
        {
            g_balance_stage = 3U;
            g_balance_arrival_frames = 0U;
            g_balance_stage_start_ticks = g_timer_10ms_ticks;
            (void)StepControl_SetOpenLoopPositionTenths(
                STEP_CONTROL_LEVEL_POSITION_TENTHS +
                BALANCE_SECOND_TILT_TENTHS);
        }
    }
    else if (g_balance_stage == 3U)
    {
        if ((link->dx >=
             (BALANCE_POSITIVE_TARGET_PIXELS -
              BALANCE_SECOND_SWITCH_LEAD_PIXELS)) ||
            ((g_timer_10ms_ticks - g_balance_stage_start_ticks) >=
             BALANCE_SECOND_TIME_TICKS))
        {
            g_balance_stage = 4U;
            g_balance_arrival_frames = 0U;
            g_balance_stage_start_ticks = g_timer_10ms_ticks;
            StepControl_SetTargetOffsetPixels(
                BALANCE_POSITIVE_TARGET_PIXELS);
            StepControl_EnableVelocityProfile(true);
        }
    }
    else
    {
        if (g_balance_previous_dx_valid &&
            (target_error <=
             BALANCE_FINAL_ARRIVAL_TOLERANCE_PIXELS) &&
            (target_error >=
             -BALANCE_FINAL_ARRIVAL_TOLERANCE_PIXELS) &&
            (frame_delta <= BALANCE_FINAL_STILL_DELTA_PIXELS) &&
            (frame_delta >= -BALANCE_FINAL_STILL_DELTA_PIXELS))
        {
            if (g_balance_arrival_frames <
                BALANCE_FINAL_ARRIVAL_FRAMES)
            {
                g_balance_arrival_frames++;
            }
        }
        else
        {
            g_balance_arrival_frames = 0U;
        }

        if (g_balance_arrival_frames >=
            BALANCE_FINAL_ARRIVAL_FRAMES)
        {
            g_balance_stage = 5U;
            g_timer_running = false;
        }
    }
}

void TIMG7_IRQHandler(void)
{
    static uint8_t divider;

    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) ==
        DL_TIMER_IIDX_ZERO)
    {
        divider++;
        if (divider >= 2U)
        {
            divider = 0U;
            g_task_10ms_pending = 1U;
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Grayscale_Sensor_Init();
    LineFollow_Init();
    K230_Link_Init();
    StepControl_Init();
    g_mpu6050_ready = (MPU6050_Init() != 0U);
    g_initial_limit_ready = false;
    g_calibration_query_ticks = 0U;
    g_calibration_position_sequence = 0U;
    X_V2_Read_Sys_Params(CAL_MOTOR_ADDRESS, S_CPOS);
    /* Give the OLED supply and charge pump time to settle after power-up. */
    delay_cycles(8000000U);
    OLED_Init();
    Display_Update10ms();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    __enable_irq();
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1)
    {
        if (g_task_10ms_pending != 0U)
        {
            __disable_irq();
            g_task_10ms_pending = 0U;
            __enable_irq();

            g_sys_tick_10ms++;

            Key_Update10ms();
            Encoder_Update10ms();
            K230_Link_Update10ms();
            ZDT_CAN_PollRx();
            InitialLimit_Update10ms();
            State_Update();
            State_Operation();
            if (g_timer_running && (g_timer_10ms_ticks < 9999U))
            {
                g_timer_10ms_ticks++;
            }
            Display_Update10ms();
        }
        else
        {
            __WFI();
        }
    }
}

static void Key_Init(void)
{
    /*
     * Re-assert the four active-low key inputs after the generated GPIO
     * initialization.  This follows the proven 25E initialization sequence
     * and prevents a later pin configuration from leaving a key floating.
     */
    DL_GPIO_initDigitalInputFeatures(
        KEY_KEY_1_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        KEY_KEY_2_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        KEY_KEY_3_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        KEY_KEY_4_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void Key_Update10ms(void)
{
    static uint8_t stable_level[4] = {0U, 0U, 0U, 0U};
    static uint8_t previous_level[4] = {0U, 0U, 0U, 0U};
    static uint8_t debounce_count[4] = {0U, 0U, 0U, 0U};
    uint8_t raw_level[4];
    uint8_t i;

    /*
     * Read every key independently, exactly as in the proven 25E project.
     * KEY1 and KEY2 then pass through the same debounce loop below.
     */
    raw_level[0] =
        (DL_GPIO_readPins(KEY_PORT, KEY_KEY_1_PIN) == 0U) ? 1U : 0U;
    raw_level[1] =
        (DL_GPIO_readPins(KEY_PORT, KEY_KEY_2_PIN) == 0U) ? 1U : 0U;
    raw_level[2] =
        (DL_GPIO_readPins(KEY_PORT, KEY_KEY_3_PIN) == 0U) ? 1U : 0U;
    raw_level[3] =
        (DL_GPIO_readPins(KEY_PORT, KEY_KEY_4_PIN) == 0U) ? 1U : 0U;

    g_key_pressed_event = 0U;
    for (i = 0U; i < 4U; i++)
    {
        if (raw_level[i] == stable_level[i])
        {
            debounce_count[i] = 0U;
        }
        else
        {
            debounce_count[i]++;
            if (debounce_count[i] >= 3U)
            {
                stable_level[i] = raw_level[i];
                debounce_count[i] = 0U;
            }
        }

        if ((stable_level[i] != 0U) && (previous_level[i] == 0U))
        {
            g_key_pressed_event |= (uint8_t)(1U << i);
        }
        previous_level[i] = stable_level[i];
    }
}

static void State_Update(void)
{
    enum CAR_STATE old_state = g_current_state;

    if ((uint32_t)g_current_state > (uint32_t)CAR_STATE_STOPPED)
    {
        g_current_state = CAR_STATE_IDLE;
        old_state = CAR_STATE_IDLE;
    }
    if (((uint32_t)g_selected_state <
         (uint32_t)CAR_STATE_MOVECIRCLE) ||
        ((uint32_t)g_selected_state >
         (uint32_t)CAR_STATE_AB_BALANCE))
    {
        g_selected_state = CAR_STATE_MOVECIRCLE;
    }

    g_next_state = g_current_state;

    /*
     * KEY3 clears the timer except in a completed AB run. In BALANCE it also restarts the complete
     * -5 cm to +5 cm task from the current, center-balanced condition. The
     * first transfer starts directly in closed loop so the controller can
     * regulate approach velocity before the ball reaches -5 cm.
     */
    if ((g_key_pressed_event & 0x04U) != 0U)
    {
        if (((g_current_state == CAR_STATE_AB_BALANCE) ||
             (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER)) &&
            !g_ab_started)
        {
            g_timer_10ms_ticks = 0U;
            g_timer_running = true;
            g_ab_started = true;
            g_ab_stopping = false;
            g_ab_stop_stable_ticks = 0U;
            g_ab_stop_ticks = 0U;
            g_ab_previous_average_pwm = 0;
            g_ab_previous_average_rpm = 0;
            g_ab_filtered_pwm_accel = 0;
            g_ab_filtered_encoder_accel = 0;
            g_ab_feedforward_tenths = 0;
            Encoder_Clear();
            LineFollow_ResetSoft();
            g_abcda_finish_creep_active = false;
            g_abcda_finish_creep_done = false;
            g_abcda_finish_creep_start_count = 0;
            if (g_current_state == CAR_STATE_AB_BALANCE)
            {
                LineFollow_SetBasePWMSlewX2(
                    AB_ACCEL_SLEW_X2, AB_DECEL_SLEW_X2);
                LineFollow_SetTurnPWMSlew(600);
            }
            else
            {
                LineFollow_SetBasePWMSlew(
                    ABCDA_ACCEL_SLEW, ABCDA_DECEL_SLEW);
                LineFollow_SetTurnPWMSlew(ABCDA_TURN_SLEW);
            }
            LineFollow_SetTargetBasePWM(ABCDA_STRAIGHT_PWM);
            if (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER)
            {
                g_abcda_curve_active = false;
            }
            Motor_Enable(true);
            if (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER)
            {
                g_circle_depart_ticks = 0U;
                g_circle_finish_ticks = 0U;
                g_circle_departed = false;
            }
        }
        else if ((g_current_state != CAR_STATE_AB_BALANCE) &&
                 (g_current_state != CAR_STATE_ABCDA_BALANCE_CENTER))
        {
            g_timer_10ms_ticks = 0U;
            g_timer_running = false;
            if (g_current_state == CAR_STATE_BALANCE)
            {
                g_balance_stage = 2U;
                g_balance_arrival_frames = 0U;
                g_balance_last_frame = K230_Link_GetData()->frame_count;
                g_balance_stage_start_ticks = 0U;
                g_balance_previous_dx_valid = false;
                StepControl_SetTargetOffsetPixels(
                    BALANCE_NEGATIVE_TARGET_PIXELS);
                StepControl_EnableVelocityProfile(true);
                g_timer_running = true;
            }
            else if ((g_current_state == CAR_STATE_MOVECIRCLE) &&
                     !g_circle_finish_requested)
            {
                g_timer_running = true;
            }
        }
    }

    switch (g_current_state)
    {
        case CAR_STATE_IDLE:
            if ((g_key_pressed_event & 0x02U) != 0U)
            {
                g_next_state = g_selected_state;
            }
            else if ((g_key_pressed_event & 0x01U) != 0U)
            {
                switch (g_selected_state)
                {
                    case CAR_STATE_MOVECIRCLE:
                        g_selected_state = CAR_STATE_BALANCE;
                        break;
                    case CAR_STATE_BALANCE:
                        g_selected_state =
                            CAR_STATE_ABCDA_BALANCE_CENTER;
                        break;
                    case CAR_STATE_ABCDA_BALANCE_CENTER:
                        g_selected_state = CAR_STATE_AB_BALANCE;
                        break;
                    case CAR_STATE_AB_BALANCE:
                    default:
                        g_selected_state = CAR_STATE_MOVECIRCLE;
                        break;
                }
            }
            break;

        case CAR_STATE_MOVECIRCLE:
            if (g_circle_finish_requested)
            {
                g_next_state = CAR_STATE_IDLE;
            }
            else if ((g_key_pressed_event & 0x08U) != 0U)
            {
                g_next_state = CAR_STATE_IDLE;
            }
            break;
        case CAR_STATE_BALANCE:
        case CAR_STATE_ABCDA_BALANCE_CENTER:
        case CAR_STATE_AB_BALANCE:
            if ((g_key_pressed_event & 0x08U) != 0U)
            {
                g_next_state = CAR_STATE_IDLE;
            }
            break;

        case CAR_STATE_STOPPED:
        default:
            g_next_state = CAR_STATE_IDLE;
            break;
    }

    if (g_next_state != old_state)
    {
        State_Exit(old_state);
        g_current_state = g_next_state;
        State_Enter(g_current_state);
    }
    else
    {
        g_current_state = g_next_state;
    }
}

static void State_Enter(enum CAR_STATE state)
{
    switch (state)
    {
        case CAR_STATE_IDLE:
            break;
        case CAR_STATE_STOPPED:
            break;
        case CAR_STATE_MOVECIRCLE:
            g_circle_depart_ticks = 0U;
            g_circle_finish_ticks = 0U;
            g_circle_departed = false;
            g_circle_finish_requested = false;
            g_timer_10ms_ticks = 0U;
            g_timer_running = true;
            LineFollow_SetBasePWMSlew(10, 10);
            LineFollow_SetTurnPWMSlew(600);
            LineFollow_Reset();
            Motor_Enable(true);
            break;
        case CAR_STATE_AB_BALANCE:
            g_ab_started = false;
            g_ab_stopping = false;
            g_ab_stop_stable_ticks = 0U;
            g_ab_stop_ticks = 0U;
            g_ab_stopped = false;
            g_ab_previous_average_pwm = 0;
            g_ab_previous_average_rpm = 0;
            g_ab_filtered_pwm_accel = 0;
            g_ab_filtered_encoder_accel = 0;
            g_ab_feedforward_tenths = 0;
            g_timer_running = false;
            Motor_Coast();
            LineFollow_ResetSoft();
            StepControl_Enter();
            StepControl_SetTargetOffsetPixels(0);
            StepControl_SetFeedforwardTenths(0);
            break;
        case CAR_STATE_ABCDA_BALANCE_CENTER:
            g_ab_started = false;
            g_ab_stopping = false;
            g_ab_stop_stable_ticks = 0U;
            g_ab_stop_ticks = 0U;
            g_ab_stopped = false;
            g_ab_previous_average_pwm = 0;
            g_ab_previous_average_rpm = 0;
            g_ab_filtered_pwm_accel = 0;
            g_ab_filtered_encoder_accel = 0;
            g_ab_feedforward_tenths = 0;
            g_abcda_curve_active = false;
            g_abcda_finish_creep_active = false;
            g_abcda_finish_creep_done = false;
            g_abcda_finish_creep_start_count = 0;
            g_timer_running = false;
            Motor_Coast();
            LineFollow_ResetSoft();
            StepControl_Enter();
            StepControl_SetTargetOffsetPixels(0);
            StepControl_SetFeedforwardTenths(0);
            break;
        case CAR_STATE_BALANCE:
            g_balance_stage = 0U;
            g_balance_arrival_frames = 0U;
            g_balance_last_frame = K230_Link_GetData()->frame_count;
            g_balance_previous_dx_valid = false;
            StepControl_Enter();
            StepControl_SetTargetOffsetPixels(0);
            break;
        default:
            break;
    }
}

static void State_Exit(enum CAR_STATE state)
{
    switch (state)
    {
        case CAR_STATE_MOVECIRCLE:
            g_timer_running = false;
            if (g_circle_finish_requested)
            {
                Motor_Brake();
            }
            else
            {
                Motor_Coast();
            }
            LineFollow_Reset();
            break;
        case CAR_STATE_IDLE:
        case CAR_STATE_STOPPED:
            break;
        case CAR_STATE_AB_BALANCE:
            g_timer_running = false;
            Motor_Coast();
            LineFollow_ResetSoft();
            StepControl_SetFeedforwardTenths(0);
            StepControl_Exit();
            break;
        case CAR_STATE_ABCDA_BALANCE_CENTER:
            g_timer_running = false;
            Motor_Coast();
            LineFollow_ResetSoft();
            StepControl_SetFeedforwardTenths(0);
            StepControl_Exit();
            break;
        case CAR_STATE_BALANCE:
            g_timer_running = false;
            StepControl_Exit();
            break;
        default:
            break;
    }
}

static void State_Operation(void)
{
    switch (g_current_state)
    {
        case CAR_STATE_IDLE:
            /* TODO: idle action */
            break;
        case CAR_STATE_MOVECIRCLE:
        {
            LineFollow_Status line_status;
            bool stop_line_detected;
            uint8_t right_black_count;
            uint8_t right_bits;

            LineFollow_Update10ms();
            line_status = LineFollow_GetStatus();
            right_bits =
                (uint8_t)(line_status.sensor_mask & CIRCLE_RIGHT5_MASK);
            right_black_count = 0U;
            if ((right_bits & 0x08U) != 0U)
            {
                right_black_count++;
            }
            if ((right_bits & 0x10U) != 0U)
            {
                right_black_count++;
            }
            if ((right_bits & 0x20U) != 0U)
            {
                right_black_count++;
            }
            if ((right_bits & 0x40U) != 0U)
            {
                right_black_count++;
            }
            if ((right_bits & 0x80U) != 0U)
            {
                right_black_count++;
            }
            /*
             * Stop for four of the five right-side channels, all three
             * rightmost channels, or at least five sensors overall.
             */
            stop_line_detected =
                (right_black_count >= 4U) ||
                ((line_status.sensor_mask & CIRCLE_RIGHT3_MASK) ==
                 CIRCLE_RIGHT3_MASK) ||
                (line_status.black_count >= 5U);

            if (!g_circle_departed)
            {
                if (!stop_line_detected)
                {
                    if (g_circle_depart_ticks < CIRCLE_DEPART_TICKS)
                    {
                        g_circle_depart_ticks++;
                    }
                    if (g_circle_depart_ticks >= CIRCLE_DEPART_TICKS)
                    {
                        g_circle_departed = true;
                    }
                }
                else
                {
                    g_circle_depart_ticks = 0U;
                }
            }
            else if (stop_line_detected)
            {
                if (g_circle_finish_ticks < CIRCLE_FINISH_TICKS)
                {
                    g_circle_finish_ticks++;
                }
                if (g_circle_finish_ticks >= CIRCLE_FINISH_TICKS)
                {
                    Motor_Brake();
                    g_timer_running = false;
                    g_circle_finish_requested = true;
                }
            }
            else
            {
                g_circle_finish_ticks = 0U;
            }
            break;
        }
        case CAR_STATE_ABCDA_BALANCE_CENTER:
        case CAR_STATE_AB_BALANCE:
        {
            Encoder_Status encoder = Encoder_GetStatus();
            Motor_Status motor = Motor_GetStatus();
            int32_t left_count = encoder.left_count;
            int32_t right_count = encoder.right_count;
            int32_t travelled;
            int32_t remaining;
            int32_t average_pwm;
            int32_t average_rpm;
            int32_t pwm_accel;
            int32_t encoder_accel;
            int32_t pwm_feedforward_gain;
            int32_t feedforward_tenths;
            int32_t gyro_z_x10;
            int32_t gyro_feedforward_tenths;
            int32_t feedforward_delta;
            int16_t target_pwm;
            LineFollow_Status line_status;
            bool stop_line_detected;
            uint8_t right_black_count;
            uint8_t right_bits;
            int16_t absolute_line_error;

            average_pwm =
                ((int32_t)motor.left_pwm + motor.right_pwm) / 2L;
            average_rpm =
                ((int32_t)encoder.left_rpm + encoder.right_rpm) / 2L;
            pwm_accel = average_pwm - g_ab_previous_average_pwm;
            encoder_accel = average_rpm - g_ab_previous_average_rpm;
            g_ab_previous_average_pwm = (int16_t)average_pwm;
            g_ab_previous_average_rpm = (int16_t)average_rpm;
            g_ab_filtered_pwm_accel = (int16_t)(
                (g_ab_filtered_pwm_accel + pwm_accel) / 2L);
            g_ab_filtered_encoder_accel = (int16_t)(
                (3L * g_ab_filtered_encoder_accel + encoder_accel) / 4L);

            if ((g_ab_filtered_encoder_accel <=
                 AB_ENCODER_ACCEL_DEAD_ZONE) &&
                (g_ab_filtered_encoder_accel >=
                 -AB_ENCODER_ACCEL_DEAD_ZONE))
            {
                g_ab_filtered_encoder_accel = 0;
            }
            pwm_feedforward_gain =
                (g_ab_filtered_pwm_accel < 0) ?
                AB_PWM_ACCEL_FF_BRAKE_NUM :
                AB_PWM_ACCEL_FF_ACCEL_NUM;
            feedforward_tenths = AB_FEEDFORWARD_SIGN *
                (pwm_feedforward_gain * g_ab_filtered_pwm_accel +
                 AB_ENCODER_ACCEL_FF_NUM *
                     g_ab_filtered_encoder_accel);
            gyro_feedforward_tenths = 0;
            if ((g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) &&
                g_abcda_curve_active && g_mpu6050_ready &&
                !g_abcda_finish_creep_active)
            {
                MPU6050_Read_Gyro();
                gyro_z_x10 = (int32_t)(gyro_z * 10.0f);
                g_abcda_filtered_gyro_z_x10 = (int16_t)(
                    (g_abcda_filtered_gyro_z_x10 + gyro_z_x10) / 2L);
                if ((g_abcda_filtered_gyro_z_x10 >
                     ABCDA_GYRO_FF_DEADZONE_X10) ||
                    (g_abcda_filtered_gyro_z_x10 <
                     -ABCDA_GYRO_FF_DEADZONE_X10))
                {
                    gyro_feedforward_tenths = ABCDA_GYRO_FF_SIGN *
                        ABCDA_GYRO_FF_GAIN_NUM *
                        g_abcda_filtered_gyro_z_x10 /
                        ABCDA_GYRO_FF_GAIN_DEN;
                    if (gyro_feedforward_tenths >
                        ABCDA_GYRO_FF_LIMIT_TENTHS)
                    {
                        gyro_feedforward_tenths =
                            ABCDA_GYRO_FF_LIMIT_TENTHS;
                    }
                    else if (gyro_feedforward_tenths <
                             -ABCDA_GYRO_FF_LIMIT_TENTHS)
                    {
                        gyro_feedforward_tenths =
                            -ABCDA_GYRO_FF_LIMIT_TENTHS;
                    }
                }
            }
            else
            {
                g_abcda_filtered_gyro_z_x10 = 0;
            }
            feedforward_tenths += gyro_feedforward_tenths;
            if (feedforward_tenths >
                ((g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) ?
                 ABCDA_FEEDFORWARD_LIMIT_TENTHS :
                 AB_FEEDFORWARD_LIMIT_TENTHS))
            {
                feedforward_tenths =
                    (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) ?
                    ABCDA_FEEDFORWARD_LIMIT_TENTHS :
                    AB_FEEDFORWARD_LIMIT_TENTHS;
            }
            else if (feedforward_tenths <
                     -((g_current_state ==
                        CAR_STATE_ABCDA_BALANCE_CENTER) ?
                       ABCDA_FEEDFORWARD_LIMIT_TENTHS :
                       AB_FEEDFORWARD_LIMIT_TENTHS))
            {
                feedforward_tenths =
                    (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) ?
                    -ABCDA_FEEDFORWARD_LIMIT_TENTHS :
                    -AB_FEEDFORWARD_LIMIT_TENTHS;
            }
            if (!g_ab_started || g_ab_stopped)
            {
                feedforward_tenths = 0;
            }
            feedforward_delta =
                feedforward_tenths - g_ab_feedforward_tenths;
            if (feedforward_delta >
                ((g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) ?
                 ABCDA_FEEDFORWARD_SLEW_TENTHS :
                 AB_FEEDFORWARD_SLEW_TENTHS))
            {
                feedforward_delta =
                    (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) ?
                    ABCDA_FEEDFORWARD_SLEW_TENTHS :
                    AB_FEEDFORWARD_SLEW_TENTHS;
            }
            else if (feedforward_delta <
                     -((g_current_state ==
                        CAR_STATE_ABCDA_BALANCE_CENTER) ?
                       ABCDA_FEEDFORWARD_SLEW_TENTHS :
                       AB_FEEDFORWARD_SLEW_TENTHS))
            {
                feedforward_delta =
                    (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) ?
                    -ABCDA_FEEDFORWARD_SLEW_TENTHS :
                    -AB_FEEDFORWARD_SLEW_TENTHS;
            }
            g_ab_feedforward_tenths = (int16_t)(
                g_ab_feedforward_tenths + feedforward_delta);
            StepControl_SetFeedforwardTenths(
                g_ab_feedforward_tenths);
            StepControl_Update10ms();
            if (g_ab_stopped)
            {
                break;
            }
            if (left_count < 0) left_count = -left_count;
            if (right_count < 0) right_count = -right_count;
            travelled = (left_count + right_count) / 2L;

            if (!g_ab_started)
            {
                break;
            }

#if AB_CONTINUOUS_TRACKING
            /* Keep following the line until the user changes mode. */
            if ((g_current_state == CAR_STATE_AB_BALANCE) &&
                g_ab_stopping)
            {
                target_pwm = (average_pwm > AB_STOP_PWM_STEP) ?
                    (int16_t)(average_pwm - AB_STOP_PWM_STEP) : 0;
                Motor_SetPWM(target_pwm, target_pwm);
                if ((encoder.left_rpm <= AB_STOP_RPM) &&
                    (encoder.left_rpm >= -AB_STOP_RPM) &&
                    (encoder.right_rpm <= AB_STOP_RPM) &&
                    (encoder.right_rpm >= -AB_STOP_RPM))
                {
                    Motor_Coast();
                    g_timer_running = false;
                    g_ab_stopped = true;
                }
                break;
            }
            if ((g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) &&
                g_abcda_finish_creep_done)
            {
                target_pwm = (average_pwm > ABCDA_STOP_PWM_STEP) ?
                    (int16_t)(average_pwm - ABCDA_STOP_PWM_STEP) : 0;
                Motor_SetPWM(target_pwm, target_pwm);
                if ((encoder.left_rpm <= AB_STOP_RPM) &&
                    (encoder.left_rpm >= -AB_STOP_RPM) &&
                    (encoder.right_rpm <= AB_STOP_RPM) &&
                    (encoder.right_rpm >= -AB_STOP_RPM))
                {
                    Motor_Coast();
                    g_timer_running = false;
                    g_ab_stopped = true;
                }
                break;
            }
            if ((g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER) &&
                g_abcda_finish_creep_active)
            {
                LineFollow_SetTargetBasePWM(
                    ABCDA_FINISH_CREEP_PWM);
            }
            if (g_current_state == CAR_STATE_ABCDA_BALANCE_CENTER)
            {
                line_status = LineFollow_GetStatus();
                absolute_line_error = (line_status.error < 0) ?
                    (int16_t)-line_status.error : line_status.error;
                if (g_abcda_finish_creep_active)
                {
                    LineFollow_SetTargetBasePWM(
                        ABCDA_FINISH_CREEP_PWM);
                }
                else if (!g_abcda_curve_active &&
                         (absolute_line_error >=
                          ABCDA_CURVE_ENTER_ERROR))
                {
                    g_abcda_curve_active = true;
                }
                else if (g_abcda_curve_active &&
                         (absolute_line_error <=
                          ABCDA_CURVE_EXIT_ERROR) &&
                         !line_status.line_lost)
                {
                    g_abcda_curve_active = false;
                }
                if (!g_abcda_finish_creep_active)
                {
                    LineFollow_SetTargetBasePWM(
                        g_abcda_curve_active ?
                        ABCDA_CURVE_PWM : ABCDA_STRAIGHT_PWM);
                }
            }
            else if (!g_ab_stopping &&
                     (travelled >= AB_DECEL_START_COUNTS))
            {
                int32_t decel_distance =
                    AB_TARGET_ENCODER_COUNTS -
                    AB_DECEL_START_COUNTS;
                int32_t decel_remaining =
                    AB_TARGET_ENCODER_COUNTS - travelled;
                if (decel_remaining < 0L) decel_remaining = 0L;
                target_pwm = (int16_t)(
                    AB_APPROACH_PWM +
                    ((int32_t)(ABCDA_STRAIGHT_PWM -
                               AB_APPROACH_PWM) *
                     decel_remaining) / decel_distance);
                LineFollow_SetTargetBasePWM(target_pwm);
            }
            LineFollow_Update10ms();
            if ((g_current_state ==
                 CAR_STATE_ABCDA_BALANCE_CENTER) &&
                g_abcda_finish_creep_active &&
                ((travelled - g_abcda_finish_creep_start_count) >=
                 ABCDA_FINISH_CREEP_COUNTS))
            {
                g_abcda_finish_creep_active = false;
                g_abcda_finish_creep_done = true;
                g_ab_stop_stable_ticks = 0U;
                break;
            }
            if (g_current_state == CAR_STATE_AB_BALANCE)
            {
                if (!g_ab_stopping &&
                    (travelled >= AB_TARGET_ENCODER_COUNTS))
                {
                    g_ab_stopping = true;
                    g_ab_stop_stable_ticks = 0U;
                }
            }
            else if (g_current_state ==
                     CAR_STATE_ABCDA_BALANCE_CENTER)
            {
                line_status = LineFollow_GetStatus();
                right_bits = (uint8_t)(
                    line_status.sensor_mask & CIRCLE_RIGHT5_MASK);
                right_black_count = 0U;
                if ((right_bits & 0x08U) != 0U) right_black_count++;
                if ((right_bits & 0x10U) != 0U) right_black_count++;
                if ((right_bits & 0x20U) != 0U) right_black_count++;
                if ((right_bits & 0x40U) != 0U) right_black_count++;
                if ((right_bits & 0x80U) != 0U) right_black_count++;
                stop_line_detected =
                    (right_black_count >= 4U) ||
                    ((line_status.sensor_mask & CIRCLE_RIGHT3_MASK) ==
                     CIRCLE_RIGHT3_MASK) ||
                    (line_status.black_count >= 5U);

                if (!g_circle_departed)
                {
                    if (!stop_line_detected)
                    {
                        if (g_circle_depart_ticks < CIRCLE_DEPART_TICKS)
                            g_circle_depart_ticks++;
                        if (g_circle_depart_ticks >= CIRCLE_DEPART_TICKS)
                            g_circle_departed = true;
                    }
                    else
                    {
                        g_circle_depart_ticks = 0U;
                    }
                }
                else if (stop_line_detected &&
                         !g_abcda_finish_creep_active &&
                         !g_abcda_finish_creep_done)
                {
                    if (g_circle_finish_ticks < CIRCLE_FINISH_TICKS)
                        g_circle_finish_ticks++;
                    if (g_circle_finish_ticks >= CIRCLE_FINISH_TICKS)
                    {
                        g_abcda_finish_creep_active = true;
                        g_abcda_finish_creep_start_count = travelled;
                        g_ab_stop_stable_ticks = 0U;
                        LineFollow_SetTargetBasePWM(
                            ABCDA_FINISH_CREEP_PWM);
                    }
                }
                else
                {
                    g_circle_finish_ticks = 0U;
                }
            }
            break;
#endif

            remaining = AB_TARGET_ENCODER_COUNTS - travelled;
            if (!g_ab_stopping && (remaining <= AB_BRAKE_ENCODER_COUNTS))
            {
                g_ab_stopping = true;
                g_ab_stop_ticks = 0U;
            }
            if (g_ab_stopping)
            {
                g_ab_stop_ticks++;
                if (remaining <= 0L)
                {
                    target_pwm = 0;
                }
                else
                {
                    target_pwm = (int16_t)(
                        ((int32_t)AB_CRUISE_PWM * remaining) /
                        AB_BRAKE_ENCODER_COUNTS);
                }
                LineFollow_SetTargetBasePWM(target_pwm);
            }

            LineFollow_Update10ms();
            if (g_ab_stopping &&
                (remaining <= 0L) &&
                (encoder.left_rpm <= AB_STOP_RPM) &&
                (encoder.left_rpm >= -AB_STOP_RPM) &&
                (encoder.right_rpm <= AB_STOP_RPM) &&
                (encoder.right_rpm >= -AB_STOP_RPM))
            {
                if (g_ab_stop_stable_ticks < AB_STOP_STABLE_TICKS)
                {
                    g_ab_stop_stable_ticks++;
                }
            }
            else
            {
                g_ab_stop_stable_ticks = 0U;
            }
            if ((g_ab_stop_stable_ticks >= AB_STOP_STABLE_TICKS) ||
                (g_ab_stop_ticks >= AB_STOP_TIMEOUT_TICKS))
            {
                Motor_Coast();
                StepControl_SetFeedforwardTenths(0);
                g_timer_running = false;
                g_ab_stopped = true;
            }
            break;
        }
        case CAR_STATE_BALANCE:
            StepControl_Update10ms();
            Balance_TaskUpdate();
            break;
        case CAR_STATE_STOPPED:
            /* TODO: stop action */
            break;
        default:
            break;
    }
}

static void InitialLimit_Update10ms(void)
{
    int32_t position;
    uint32_t sequence;

    if (g_initial_limit_ready)
    {
        g_calibration_query_ticks++;
        if (g_calibration_query_ticks >= POSITION_REFRESH_TICKS)
        {
            g_calibration_query_ticks = 0U;
            X_V2_Read_Sys_Params(CAL_MOTOR_ADDRESS, S_CPOS);
        }
        return;
    }

    if (ZDT_CAN_GetMotorPosition(
            CAL_MOTOR_ADDRESS, &position, &sequence) &&
        (sequence != g_calibration_position_sequence))
    {
        g_calibration_ccw_position = position;
        g_calibration_cw_position = position + CAL_POSITION_SPAN;
        ZDT_CAN_SetMotorLimits(
            CAL_MOTOR_ADDRESS, g_calibration_ccw_position,
            g_calibration_cw_position);
        g_calibration_position_sequence = sequence;
        g_initial_limit_ready = true;
        return;
    }

    g_calibration_query_ticks++;
    if (g_calibration_query_ticks >= CAL_QUERY_TICKS)
    {
        g_calibration_query_ticks = 0U;
        X_V2_Read_Sys_Params(CAL_MOTOR_ADDRESS, S_CPOS);
    }
}

static const char *Display_StateName(enum CAR_STATE state)
{
    switch (state)
    {
        case CAR_STATE_MOVECIRCLE:
            return "CIRCLE";
        case CAR_STATE_BALANCE:
            return "BAL";
        case CAR_STATE_ABCDA_BALANCE_CENTER:
            return "ABCDA";
        case CAR_STATE_AB_BALANCE:
            return "AB";
        default:
            return "-";
    }
}

static void Display_EncoderStatus(void)
{
    if (State_UsesStepControl(g_current_state))
    {
        StepControl_Status status = StepControl_GetStatus();

        OLED_ClearArea(0U, 16U, 128U, 8U);
        OLED_ShowString(0U, 16U, "DX:", OLED_6X8);
        OLED_ShowSignedNum(18U, 16U, status.raw_dx, 4U, OLED_6X8);
        OLED_ShowString(60U, 16U, "T:", OLED_6X8);
        OLED_ShowSignedNum(72U, 16U, status.target_offset, 4U, OLED_6X8);
        OLED_UpdateArea(0U, 16U, 128U, 8U);

        OLED_ClearArea(0U, 24U, 128U, 8U);
        OLED_ShowString(0U, 24U, "OUT:", OLED_6X8);
        OLED_ShowSignedNum(24U, 24U, status.output_rpm, 3U, OLED_6X8);
        OLED_UpdateArea(0U, 24U, 128U, 8U);
        return;
    }

    Encoder_Status encoder = Encoder_GetStatus();

    OLED_ClearArea(0U, 16U, 128U, 8U);
    OLED_ShowString(0U, 16U, "EL:", OLED_6X8);
    OLED_ShowSignedNum(18U, 16U, encoder.left_count, 7U, OLED_6X8);
    OLED_ShowString(72U, 16U, "R:", OLED_6X8);
    OLED_ShowSignedNum(84U, 16U, encoder.left_rpm, 4U, OLED_6X8);
    OLED_UpdateArea(0U, 16U, 128U, 8U);

    OLED_ClearArea(0U, 24U, 128U, 8U);
    OLED_ShowString(0U, 24U, "ER:", OLED_6X8);
    OLED_ShowSignedNum(18U, 24U, encoder.right_count, 7U, OLED_6X8);
    OLED_ShowString(72U, 24U, "R:", OLED_6X8);
    OLED_ShowSignedNum(84U, 24U, encoder.right_rpm, 4U, OLED_6X8);
    OLED_UpdateArea(0U, 24U, 128U, 8U);
}

static void Display_Update10ms(void)
{
    static uint8_t heartbeat_ticks;
    static uint8_t position_display_ticks;
    static uint8_t timer_display_ticks;
    static uint8_t encoder_display_ticks;
    static enum CAR_STATE last_state = CAR_STATE_STOPPED;
    static enum CAR_STATE last_selected_state = CAR_STATE_STOPPED;
    static uint8_t last_heartbeat = 0xFFU;
    uint8_t row_changed;
    uint8_t heartbeat_changed;
    const char *run_text;

    heartbeat_ticks++;
    position_display_ticks++;
    timer_display_ticks++;
    encoder_display_ticks++;
    if (heartbeat_ticks >= OLED_HEARTBEAT_TICKS)
    {
        heartbeat_ticks = 0U;
        g_oled_heartbeat ^= 1U;
    }

    row_changed = ((g_current_state != last_state) ||
                   (g_selected_state != last_selected_state)) ? 1U : 0U;
    heartbeat_changed =
        (g_oled_heartbeat != last_heartbeat) ? 1U : 0U;

    if (row_changed != 0U)
    {
        run_text = (g_current_state == CAR_STATE_IDLE) ? "IDLE" : "RUN ";
        OLED_ClearArea(0U, 0U, 120U, 8U);
        OLED_ShowString(0U, 0U, (char *)run_text, OLED_6X8);
        OLED_ShowString(30U, 0U, "SEL:", OLED_6X8);
        OLED_ShowString(
            54U, 0U, (char *)Display_StateName(g_selected_state), OLED_6X8);
        OLED_ShowChar(
            114U, 0U, (g_oled_heartbeat != 0U) ? '*' : ' ', OLED_6X8);
        OLED_UpdateArea(0U, 0U, 120U, 8U);
        last_state = g_current_state;
        last_selected_state = g_selected_state;
        last_heartbeat = g_oled_heartbeat;

        OLED_ClearArea(0U, 8U, 120U, 8U);
        if (State_UsesStepControl(g_current_state))
        {
            StepControl_Status status = StepControl_GetStatus();
            OLED_ShowString(0U, 8U, "LK:", OLED_6X8);
            OLED_ShowNum(
                18U, 8U, status.link_valid ? 1U : 0U, 1U, OLED_6X8);
            OLED_ShowString(36U, 8U, "E:", OLED_6X8);
            OLED_ShowSignedNum(48U, 8U, status.error, 3U, OLED_6X8);
            OLED_ShowString(72U, 8U, "I:", OLED_6X8);
            OLED_ShowSignedNum(84U, 8U, status.integral, 5U, OLED_6X8);
        }
        OLED_UpdateArea(0U, 8U, 120U, 8U);
        Display_EncoderStatus();
        encoder_display_ticks = 0U;
    }
    else if (heartbeat_changed != 0U)
    {
        /*
         * Refresh at the heartbeat rate only.  The complete row update also
         * avoids controller-specific issues with a six-column window.
         */
        OLED_ClearArea(114U, 0U, 6U, 8U);
        OLED_ShowChar(
            114U, 0U, (g_oled_heartbeat != 0U) ? '*' : ' ', OLED_6X8);
        OLED_UpdateArea(0U, 0U, 120U, 8U);
        if (State_UsesStepControl(g_current_state))
        {
            StepControl_Status status = StepControl_GetStatus();
            OLED_ClearArea(0U, 8U, 120U, 8U);
            OLED_ShowString(0U, 8U, "LK:", OLED_6X8);
            OLED_ShowNum(
                18U, 8U, status.link_valid ? 1U : 0U, 1U, OLED_6X8);
            OLED_ShowString(36U, 8U, "E:", OLED_6X8);
            OLED_ShowSignedNum(48U, 8U, status.error, 3U, OLED_6X8);
            OLED_ShowString(72U, 8U, "I:", OLED_6X8);
            OLED_ShowSignedNum(84U, 8U, status.integral, 5U, OLED_6X8);
            OLED_UpdateArea(0U, 8U, 120U, 8U);
        }
        last_heartbeat = g_oled_heartbeat;
    }

    if (encoder_display_ticks >= 10U)
    {
        encoder_display_ticks = 0U;
        Display_EncoderStatus();
    }

    if (position_display_ticks >= 10U)
    {
        int32_t current_position;
        int32_t initial_position;
        int32_t maximum_position;
        uint32_t position_sequence;

        position_display_ticks = 0U;
        OLED_ClearArea(0U, 32U, 128U, 8U);
        OLED_ShowString(0U, 32U, "STEP:", OLED_6X8);
        if (ZDT_CAN_GetMotorPosition(
                CAL_MOTOR_ADDRESS, &current_position,
                &position_sequence) &&
            ZDT_CAN_GetMotorLimits(
                CAL_MOTOR_ADDRESS, &initial_position,
                &maximum_position))
        {
            (void)position_sequence;
            (void)maximum_position;
            OLED_ShowSignedNum(
                30U, 32U, current_position - initial_position,
                8U, OLED_6X8);
        }
        else
        {
            OLED_ShowString(30U, 32U, "WAIT", OLED_6X8);
        }
        OLED_UpdateArea(0U, 32U, 128U, 8U);
    }

    if (timer_display_ticks >= 5U)
    {
        uint32_t seconds;
        uint32_t hundredths;

        timer_display_ticks = 0U;
        seconds = (g_timer_10ms_ticks / 100U) % 100U;
        hundredths = g_timer_10ms_ticks % 100U;
        OLED_ClearArea(0U, 40U, 128U, 8U);
        OLED_ShowString(0U, 40U, "TIME:", OLED_6X8);
        OLED_ShowNum(30U, 40U, seconds, 2U, OLED_6X8);
        OLED_ShowChar(42U, 40U, ':', OLED_6X8);
        OLED_ShowNum(48U, 40U, hundredths, 2U, OLED_6X8);
        OLED_UpdateArea(0U, 40U, 128U, 8U);
    }
}
