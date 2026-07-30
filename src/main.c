#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "X_V2.h"
#include "control.h"
#include "grayscale_sensor.h"
#include "encoder_motor.h"
#include "zdt_can_port.h"
#include "K230_link.h"
#include "Step_Control.h"

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
#define CAL_QUERY_TICKS             (10U)
#define POSITION_REFRESH_TICKS      (2U)
#define CAL_POSITION_SPAN           (1222L)
#define ABCDA_TEST_OFFSET_TENTH_DEG (300L)
#define ABCDA_TEST_ACCEL_RPM_S      (4000U)
#define ABCDA_TEST_SPEED_RPM        (60.0f)

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
static uint8_t g_oled_heartbeat;
static int32_t g_calibration_ccw_position;
static int32_t g_calibration_cw_position;
static uint32_t g_calibration_position_sequence;
static uint8_t g_calibration_query_ticks;
static bool g_initial_limit_ready;

static void Key_Init(void);
static void Key_Update10ms(void);
static void State_Update(void);
static void State_Enter(enum CAR_STATE state);
static void State_Exit(enum CAR_STATE state);
static void State_Operation(void);
static void Display_Update10ms(void);
static const char *Display_StateName(enum CAR_STATE state);
static void InitialLimit_Update10ms(void);
static void ABCDA_SendFixedPositionOnce(void);

static void ABCDA_SendFixedPositionOnce(void)
{
    int32_t minimum_position;
    int32_t maximum_position;
    int32_t target;
    uint8_t direction;
    float position_degree;

    if (!ZDT_CAN_GetMotorLimits(
            CAL_MOTOR_ADDRESS, &minimum_position, &maximum_position))
    {
        return;
    }

    target = minimum_position + ABCDA_TEST_OFFSET_TENTH_DEG;
    if (target > maximum_position)
    {
        target = maximum_position;
    }

    direction = (target < 0) ? 1U : 0U;
    position_degree =
        (target < 0) ? (float)-target / 10.0f : (float)target / 10.0f;

    X_V2_En_Control(CAL_MOTOR_ADDRESS, true, false);
    X_V2_Traj_Pos_Control(
        CAL_MOTOR_ADDRESS, direction,
        ABCDA_TEST_ACCEL_RPM_S, ABCDA_TEST_ACCEL_RPM_S,
        ABCDA_TEST_SPEED_RPM, position_degree, 1U, false);
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
    Grayscale_Sensor_Init();
    LineFollow_Init();
    K230_Link_Init();
    StepControl_Init();
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

            Key_Update10ms();
            K230_Link_Update10ms();
            ZDT_CAN_PollRx();
            InitialLimit_Update10ms();
            State_Update();
            State_Operation();
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
        case CAR_STATE_ABCDA_BALANCE_CENTER:
            ABCDA_SendFixedPositionOnce();
            break;
        case CAR_STATE_STOPPED:
            break;
        case CAR_STATE_MOVECIRCLE:
            LineFollow_Reset();
            Motor_Enable(true);
            break;
        case CAR_STATE_AB_BALANCE:
            StepControl_Enter();
            break;
        case CAR_STATE_BALANCE:
            /* Reserved for a future application feature. */
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
            Motor_Coast();
            LineFollow_Reset();
            break;
        case CAR_STATE_ABCDA_BALANCE_CENTER:
            X_V2_Stop_Now(CAL_MOTOR_ADDRESS, false);
            break;
        case CAR_STATE_IDLE:
        case CAR_STATE_STOPPED:
            break;
        case CAR_STATE_AB_BALANCE:
            StepControl_Exit();
            break;
        case CAR_STATE_BALANCE:
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
            LineFollow_Update10ms();
            break;
        case CAR_STATE_AB_BALANCE:
            StepControl_Update10ms();
            break;
        case CAR_STATE_BALANCE:
            /* Reserved for a future application feature. */
            break;
        case CAR_STATE_ABCDA_BALANCE_CENTER:
            /* Fixed-position test: intentionally send no repeated commands. */
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

static void Display_Update10ms(void)
{
    static uint8_t heartbeat_ticks;
    static uint8_t position_display_ticks;
    static enum CAR_STATE last_state = CAR_STATE_STOPPED;
    static enum CAR_STATE last_selected_state = CAR_STATE_STOPPED;
    static uint8_t last_heartbeat = 0xFFU;
    uint8_t row_changed;
    uint8_t heartbeat_changed;
    const char *run_text;

    heartbeat_ticks++;
    position_display_ticks++;
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
        if (g_current_state == CAR_STATE_AB_BALANCE)
        {
            StepControl_Status status = StepControl_GetStatus();
            OLED_ShowString(0U, 8U, "LK:", OLED_6X8);
            OLED_ShowNum(
                18U, 8U, status.link_valid ? 1U : 0U, 1U, OLED_6X8);
            OLED_ShowString(36U, 8U, "E:", OLED_6X8);
            OLED_ShowSignedNum(48U, 8U, status.error, 4U, OLED_6X8);
        }
        OLED_UpdateArea(0U, 8U, 120U, 8U);
        OLED_ClearArea(0U, 16U, 120U, 8U);
        if (g_current_state == CAR_STATE_AB_BALANCE)
        {
            StepControl_Status status = StepControl_GetStatus();
            OLED_ShowString(0U, 16U, "PID:", OLED_6X8);
            OLED_ShowSignedNum(
                24U, 16U, status.output_rpm, 3U, OLED_6X8);
            OLED_ShowString(48U, 16U, "B:", OLED_6X8);
            OLED_ShowNum(
                60U, 16U, K230_Link_GetRxByteCount() % 1000U,
                3U, OLED_6X8);
            OLED_ShowString(84U, 16U, "F:", OLED_6X8);
            OLED_ShowNum(
                96U, 16U,
                K230_Link_GetData()->frame_count % 1000U,
                3U, OLED_6X8);
        }
        OLED_UpdateArea(0U, 16U, 120U, 8U);
        OLED_ClearArea(0U, 24U, 120U, 8U);
        if (g_current_state == CAR_STATE_AB_BALANCE)
        {
            OLED_ShowString(0U, 24U, "D:", OLED_6X8);
            OLED_ShowNum(
                12U, 24U, K230_Link_GetData()->detected ? 1U : 0U,
                1U, OLED_6X8);
            OLED_ShowString(24U, 24U, "C:", OLED_6X8);
            OLED_ShowNum(
                36U, 24U, ZDT_CAN_GetLastStatus(), 1U, OLED_6X8);
            OLED_ShowString(48U, 24U, "L:", OLED_6X8);
            OLED_ShowNum(
                60U, 24U, ZDT_CAN_GetLastErrorCode(), 1U, OLED_6X8);
            OLED_ShowString(72U, 24U, "T:", OLED_6X8);
            OLED_ShowNum(
                84U, 24U, ZDT_CAN_GetTxErrorCount(), 3U, OLED_6X8);
            OLED_ShowString(108U, 24U, "B:", OLED_6X8);
            OLED_ShowNum(
                120U, 24U, ZDT_CAN_GetBusOffStatus(), 1U, OLED_6X8);
        }
        OLED_UpdateArea(0U, 24U, 128U, 8U);
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
        if (g_current_state == CAR_STATE_AB_BALANCE)
        {
            StepControl_Status status = StepControl_GetStatus();
            OLED_ClearArea(0U, 8U, 120U, 8U);
            OLED_ShowString(0U, 8U, "LK:", OLED_6X8);
            OLED_ShowNum(
                18U, 8U, status.link_valid ? 1U : 0U, 1U, OLED_6X8);
            OLED_ShowString(36U, 8U, "E:", OLED_6X8);
            OLED_ShowSignedNum(48U, 8U, status.error, 4U, OLED_6X8);
            OLED_UpdateArea(0U, 8U, 120U, 8U);
            OLED_ClearArea(0U, 16U, 120U, 8U);
            OLED_ShowString(0U, 16U, "PID:", OLED_6X8);
            OLED_ShowSignedNum(
                24U, 16U, status.output_rpm, 3U, OLED_6X8);
            OLED_ShowString(48U, 16U, "B:", OLED_6X8);
            OLED_ShowNum(
                60U, 16U, K230_Link_GetRxByteCount() % 1000U,
                3U, OLED_6X8);
            OLED_ShowString(84U, 16U, "F:", OLED_6X8);
            OLED_ShowNum(
                96U, 16U,
                K230_Link_GetData()->frame_count % 1000U,
                3U, OLED_6X8);
            OLED_UpdateArea(0U, 16U, 120U, 8U);
            OLED_ClearArea(0U, 24U, 128U, 8U);
            OLED_ShowString(0U, 24U, "D:", OLED_6X8);
            OLED_ShowNum(
                12U, 24U, K230_Link_GetData()->detected ? 1U : 0U,
                1U, OLED_6X8);
            OLED_ShowString(24U, 24U, "C:", OLED_6X8);
            OLED_ShowNum(
                36U, 24U, ZDT_CAN_GetLastStatus(), 1U, OLED_6X8);
            OLED_ShowString(48U, 24U, "L:", OLED_6X8);
            OLED_ShowNum(
                60U, 24U, ZDT_CAN_GetLastErrorCode(), 1U, OLED_6X8);
            OLED_ShowString(72U, 24U, "T:", OLED_6X8);
            OLED_ShowNum(
                84U, 24U, ZDT_CAN_GetTxErrorCount(), 3U, OLED_6X8);
            OLED_ShowString(108U, 24U, "B:", OLED_6X8);
            OLED_ShowNum(
                120U, 24U, ZDT_CAN_GetBusOffStatus(), 1U, OLED_6X8);
            OLED_UpdateArea(0U, 24U, 128U, 8U);
        }
        last_heartbeat = g_oled_heartbeat;
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
}
