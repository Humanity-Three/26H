#include "encoder_motor.h"
#include "ti_msp_dl_config.h"

static Motor_Status g_motor_status;
static volatile int32_t g_encoder_left_count;
static volatile int32_t g_encoder_right_count;
static volatile uint8_t g_encoder_left_state;
static volatile uint8_t g_encoder_right_state;
static Encoder_Status g_encoder_status;
static int32_t g_encoder_last_left;
static int32_t g_encoder_last_right;

/*
 * Quadrature transition table indexed by (old_state << 2) | new_state.
 * State bit1=A and bit0=B. Invalid/no-change transitions contribute zero.
 */
static const int8_t g_quadrature_delta[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static int16_t Motor_ClampPWM(int16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }
    if (pwm < -MOTOR_PWM_MAX)
    {
        return -MOTOR_PWM_MAX;
    }
    return pwm;
}

static uint32_t Motor_PWMToCompare(int16_t pwm)
{
    int32_t magnitude = pwm;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }
    return (uint32_t)(MOTOR_PWM_MAX - magnitude);
}

static void Motor_SetLeftDirection(int16_t pwm)
{
#if MOTOR_LEFT_FORWARD_REVERSED
    pwm = (int16_t)-pwm;
#endif

    if (pwm > 0)
    {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN_1_PIN);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN_2_PIN);
    }
    else if (pwm < 0)
    {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN_1_PIN);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN_2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MOTOR_PORT,
                          MOTOR_AIN_1_PIN | MOTOR_AIN_2_PIN);
    }
}

static void Motor_SetRightDirection(int16_t pwm)
{
#if MOTOR_RIGHT_FORWARD_REVERSED
    pwm = (int16_t)-pwm;
#endif

    if (pwm > 0)
    {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN_1_PIN);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN_2_PIN);
    }
    else if (pwm < 0)
    {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN_1_PIN);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN_2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MOTOR_PORT,
                          MOTOR_BIN_1_PIN | MOTOR_BIN_2_PIN);
    }
}

void Motor_Init(void)
{
    DL_GPIO_clearPins(MOTOR_PORT,
                      MOTOR_STBY_PIN |
                      MOTOR_AIN_1_PIN |
                      MOTOR_AIN_2_PIN |
                      MOTOR_BIN_1_PIN |
                      MOTOR_BIN_2_PIN);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, MOTOR_PWM_MAX, GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, MOTOR_PWM_MAX, GPIO_MOTOR_PWM_C1_IDX);
    DL_TimerG_startCounter(MOTOR_PWM_INST);

    g_motor_status.left_pwm = 0;
    g_motor_status.right_pwm = 0;
    g_motor_status.enabled = true;
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_STBY_PIN);
}

void Motor_Enable(bool enable)
{
    if (enable)
    {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_STBY_PIN);
        g_motor_status.enabled = true;
    }
    else
    {
        Motor_Coast();
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_STBY_PIN);
        g_motor_status.enabled = false;
    }
}

void Motor_SetPWM(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = Motor_ClampPWM(left_pwm);
    right_pwm = Motor_ClampPWM(right_pwm);

    if (!g_motor_status.enabled)
    {
        g_motor_status.left_pwm = 0;
        g_motor_status.right_pwm = 0;
        return;
    }

    Motor_SetLeftDirection(left_pwm);
    Motor_SetRightDirection(right_pwm);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, Motor_PWMToCompare(left_pwm),
        GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, Motor_PWMToCompare(right_pwm),
        GPIO_MOTOR_PWM_C1_IDX);
    g_motor_status.left_pwm = left_pwm;
    g_motor_status.right_pwm = right_pwm;
}

void Motor_Coast(void)
{
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, MOTOR_PWM_MAX, GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, MOTOR_PWM_MAX, GPIO_MOTOR_PWM_C1_IDX);
    DL_GPIO_clearPins(MOTOR_PORT,
                      MOTOR_AIN_1_PIN |
                      MOTOR_AIN_2_PIN |
                      MOTOR_BIN_1_PIN |
                      MOTOR_BIN_2_PIN);
    g_motor_status.left_pwm = 0;
    g_motor_status.right_pwm = 0;
}

void Motor_Brake(void)
{
    if (!g_motor_status.enabled)
    {
        return;
    }

    DL_GPIO_setPins(MOTOR_PORT,
                    MOTOR_AIN_1_PIN |
                    MOTOR_AIN_2_PIN |
                    MOTOR_BIN_1_PIN |
                    MOTOR_BIN_2_PIN);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, 0U, GPIO_MOTOR_PWM_C0_IDX);
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_INST, 0U, GPIO_MOTOR_PWM_C1_IDX);
    g_motor_status.left_pwm = 0;
    g_motor_status.right_pwm = 0;
}

Motor_Status Motor_GetStatus(void)
{
    return g_motor_status;
}

static uint8_t Encoder_ReadLeftState(void)
{
    uint32_t pins = DL_GPIO_readPins(
        ENCODER_READ_PORT,
        ENCODER_READ_ENCODER_A1_PIN | ENCODER_READ_ENCODER_B1_PIN);
    return (uint8_t)((((pins & ENCODER_READ_ENCODER_A1_PIN) != 0U) ? 2U : 0U) |
                     (((pins & ENCODER_READ_ENCODER_B1_PIN) != 0U) ? 1U : 0U));
}

static uint8_t Encoder_ReadRightState(void)
{
    uint32_t pins = DL_GPIO_readPins(
        ENCODER_READ_PORT,
        ENCODER_READ_ENCODER_A2_PIN | ENCODER_READ_ENCODER_B2_PIN);
    return (uint8_t)((((pins & ENCODER_READ_ENCODER_A2_PIN) != 0U) ? 2U : 0U) |
                     (((pins & ENCODER_READ_ENCODER_B2_PIN) != 0U) ? 1U : 0U));
}

void Encoder_Init(void)
{
    uint32_t pins = ENCODER_READ_ENCODER_A1_PIN |
                    ENCODER_READ_ENCODER_B1_PIN |
                    ENCODER_READ_ENCODER_A2_PIN |
                    ENCODER_READ_ENCODER_B2_PIN;

    Encoder_Clear();
    g_encoder_left_state = Encoder_ReadLeftState();
    g_encoder_right_state = Encoder_ReadRightState();

    DL_GPIO_setUpperPinsPolarity(
        GPIOB, DL_GPIO_PIN_23_EDGE_RISE_FALL |
               DL_GPIO_PIN_24_EDGE_RISE_FALL |
               DL_GPIO_PIN_26_EDGE_RISE_FALL |
               DL_GPIO_PIN_27_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, pins);
    DL_GPIO_enableInterrupt(GPIOB, pins);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Encoder_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        GPIOB, ENCODER_READ_ENCODER_A1_PIN |
               ENCODER_READ_ENCODER_B1_PIN |
               ENCODER_READ_ENCODER_A2_PIN |
               ENCODER_READ_ENCODER_B2_PIN);
    uint8_t new_state;
    int8_t delta;

    if ((pending & (ENCODER_READ_ENCODER_A1_PIN |
                    ENCODER_READ_ENCODER_B1_PIN)) != 0U)
    {
        new_state = Encoder_ReadLeftState();
        delta = g_quadrature_delta[
            (g_encoder_left_state << 2U) | new_state];
#if ENCODER_LEFT_REVERSED
        delta = (int8_t)-delta;
#endif
        g_encoder_left_count += delta;
        g_encoder_left_state = new_state;
    }

    if ((pending & (ENCODER_READ_ENCODER_A2_PIN |
                    ENCODER_READ_ENCODER_B2_PIN)) != 0U)
    {
        new_state = Encoder_ReadRightState();
        delta = g_quadrature_delta[
            (g_encoder_right_state << 2U) | new_state];
#if ENCODER_RIGHT_REVERSED
        delta = (int8_t)-delta;
#endif
        g_encoder_right_count += delta;
        g_encoder_right_state = new_state;
    }

    DL_GPIO_clearInterruptStatus(GPIOB, pending);
}

void GROUP1_IRQHandler(void)
{
    if (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) ==
        DL_INTERRUPT_GROUP1_IIDX_GPIOB)
    {
        Encoder_IRQHandler();
    }
}

void Encoder_Update10ms(void)
{
    int32_t left;
    int32_t right;
    int32_t left_delta;
    int32_t right_delta;

    __disable_irq();
    left = g_encoder_left_count;
    right = g_encoder_right_count;
    __enable_irq();

    left_delta = left - g_encoder_last_left;
    right_delta = right - g_encoder_last_right;
    g_encoder_last_left = left;
    g_encoder_last_right = right;

    g_encoder_status.left_count = left;
    g_encoder_status.right_count = right;
    g_encoder_status.left_delta_10ms = (int16_t)left_delta;
    g_encoder_status.right_delta_10ms = (int16_t)right_delta;
    g_encoder_status.left_rpm =
        (int16_t)((left_delta * 6000L) / ENCODER_COUNTS_PER_REV);
    g_encoder_status.right_rpm =
        (int16_t)((right_delta * 6000L) / ENCODER_COUNTS_PER_REV);
}

void Encoder_Clear(void)
{
    __disable_irq();
    g_encoder_left_count = 0;
    g_encoder_right_count = 0;
    __enable_irq();
    g_encoder_last_left = 0;
    g_encoder_last_right = 0;
    g_encoder_status.left_count = 0;
    g_encoder_status.right_count = 0;
    g_encoder_status.left_delta_10ms = 0;
    g_encoder_status.right_delta_10ms = 0;
    g_encoder_status.left_rpm = 0;
    g_encoder_status.right_rpm = 0;
}

Encoder_Status Encoder_GetStatus(void)
{
    return g_encoder_status;
}
