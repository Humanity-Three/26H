#include "control.h"
#include "grayscale_sensor.h"
#include "ti_msp_dl_config.h"

static LineFollow_Config g_line_config = {
    /* 与已在当前硬件验证的 25E 工程保持一致。 */
    .base_pwm = 960,
    .kp = 110,
    .ki = 0,
    .kd = 60,
    .max_turn_pwm = 600
};

static LineFollow_Status g_line_status;
static int16_t g_line_last_error;
static int32_t g_line_integral;
static int16_t g_line_current_base_pwm;
static int16_t g_line_target_base_pwm;
static int16_t g_line_acceleration_slew = 10;
static int16_t g_line_deceleration_slew = 10;
static int16_t g_line_current_turn_pwm;
static int16_t g_line_turn_slew = 600;
static bool g_line_acceleration_half_step;

static int16_t LineFollow_Clamp(int32_t value, int16_t limit)
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

void LineFollow_Init(void)
{
    LineFollow_Reset();
}

void LineFollow_Reset(void)
{
    g_line_last_error = 0;
    g_line_integral = 0;
    /*
     * 25E 实车的可靠起转值为 320。若从 0 缓升，前几十个控制周期
     * 始终落在电机死区内，看起来会像 KEY2 没有启动电机。
     */
    g_line_current_base_pwm = 640;
    g_line_target_base_pwm = g_line_config.base_pwm;
    g_line_current_turn_pwm = 0;
    g_line_status.sensor_mask = 0U;
    g_line_status.black_count = 0U;
    g_line_status.error = 0;
    g_line_status.turn_pwm = 0;
    g_line_status.line_lost = true;
    g_line_status.cross_line = false;
}

void LineFollow_ResetSoft(void)
{
    LineFollow_Reset();
    g_line_current_base_pwm = 0;
    g_line_target_base_pwm = 0;
}

void LineFollow_SetTargetBasePWM(int16_t base_pwm)
{
    if (base_pwm < 0)
    {
        base_pwm = 0;
    }
    else if (base_pwm > MOTOR_PWM_MAX)
    {
        base_pwm = MOTOR_PWM_MAX;
    }
    g_line_target_base_pwm = base_pwm;
}

void LineFollow_SetBasePWMSlew(
    int16_t acceleration_slew, int16_t deceleration_slew)
{
    if (acceleration_slew < 1) acceleration_slew = 1;
    if (deceleration_slew < 1) deceleration_slew = 1;
    g_line_acceleration_slew = acceleration_slew;
    g_line_deceleration_slew = deceleration_slew;
    g_line_acceleration_half_step = false;
}

void LineFollow_SetBasePWMSlewX2(
    int16_t acceleration_slew_x2, int16_t deceleration_slew_x2)
{
    if (acceleration_slew_x2 < 2) acceleration_slew_x2 = 2;
    if (deceleration_slew_x2 < 2) deceleration_slew_x2 = 2;
    g_line_acceleration_slew = (int16_t)(acceleration_slew_x2 / 2);
    g_line_deceleration_slew = (int16_t)(deceleration_slew_x2 / 2);
    g_line_acceleration_half_step =
        ((acceleration_slew_x2 & 1) != 0);
}

void LineFollow_SetTurnPWMSlew(int16_t turn_slew)
{
    if (turn_slew < 1) turn_slew = 1;
    g_line_turn_slew = turn_slew;
}

void LineFollow_SetConfig(const LineFollow_Config *config)
{
    if (config == 0)
    {
        return;
    }
    g_line_config = *config;
}

void LineFollow_Update10ms(void)
{
    static const int8_t weight[GRAYSCALE_SENSOR_CHANNELS] =
        {-7, -5, -3, -1, 1, 3, 5, 7};
    uint16_t sensor[GRAYSCALE_SENSOR_CHANNELS];
    int16_t weighted_sum = 0;
    int16_t error;
    int16_t derivative;
    int32_t pid_output;
    int16_t turn;
    int16_t turn_limit;
    uint8_t count = 0U;
    uint8_t mask = 0U;
    uint8_t i;

    /*
     * 进入循迹后的前 300 ms 先让两轮以 25E 已验证的起转值直行。
     * 这既越过电机死区，也避免首次灰度采样把一侧 PWM 立即压到 0。
     */
    Grayscale_Sensor_Read_All(sensor);
    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        if (sensor[i] != 0U)
        {
            mask |= (uint8_t)(1U << i);
            weighted_sum = (int16_t)(weighted_sum + weight[i]);
            count++;
        }
    }

    g_line_status.sensor_mask = mask;
    g_line_status.black_count = count;
    g_line_status.cross_line = (count >= 6U);

    if (count == 0U)
    {
        /*
         * 丢线时沿上一次偏差方向寻找黑线，避免误把偏差置零后直行。
         * 若传感器左右顺序相反，只需颠倒 weight[] 的符号。
         */
        g_line_status.line_lost = true;
        error = (g_line_last_error >= 0) ? 9 : -9;
        g_line_integral = 0;
    }
    else
    {
        g_line_status.line_lost = false;
        error = (int16_t)(weighted_sum / (int16_t)count);
        g_line_integral += error;
        if (g_line_integral > 100)
        {
            g_line_integral = 100;
        }
        else if (g_line_integral < -100)
        {
            g_line_integral = -100;
        }
    }

    derivative = (int16_t)(error - g_line_last_error);
    pid_output = (int32_t)g_line_config.kp * error +
                 (int32_t)g_line_config.ki * g_line_integral +
                 (int32_t)g_line_config.kd * derivative;
    /* 从可靠起转值平滑升至 25E 已验证的巡航值。 */
    if (g_line_current_base_pwm < g_line_target_base_pwm)
    {
        static bool add_half_step;
        int16_t acceleration_step = g_line_acceleration_slew;
        if (g_line_acceleration_half_step)
        {
            add_half_step = !add_half_step;
            if (add_half_step) acceleration_step++;
        }
        g_line_current_base_pwm =
            (int16_t)(g_line_current_base_pwm +
                      acceleration_step);
        if (g_line_current_base_pwm > g_line_target_base_pwm)
        {
            g_line_current_base_pwm = g_line_target_base_pwm;
        }
    }
    else if (g_line_current_base_pwm > g_line_target_base_pwm)
    {
        g_line_current_base_pwm =
            (int16_t)(g_line_current_base_pwm -
                      g_line_deceleration_slew);
        if (g_line_current_base_pwm < g_line_target_base_pwm)
        {
            g_line_current_base_pwm = g_line_target_base_pwm;
        }
    }

    turn_limit = g_line_config.max_turn_pwm;
    if (turn_limit > g_line_current_base_pwm)
    {
        turn_limit = g_line_current_base_pwm;
    }
    turn = LineFollow_Clamp(pid_output, turn_limit);

    if (turn > g_line_current_turn_pwm + g_line_turn_slew)
    {
        g_line_current_turn_pwm = (int16_t)(
            g_line_current_turn_pwm + g_line_turn_slew);
    }
    else if (turn < g_line_current_turn_pwm - g_line_turn_slew)
    {
        g_line_current_turn_pwm = (int16_t)(
            g_line_current_turn_pwm - g_line_turn_slew);
    }
    else
    {
        g_line_current_turn_pwm = turn;
    }

    /*
     * 正误差表示黑线位于传感器右侧：左轮加速、右轮减速。
     * 实车方向若相反，交换此处的加减号即可。
     */
    Motor_SetPWM(
        (int16_t)(g_line_current_base_pwm + g_line_current_turn_pwm),
        (int16_t)(g_line_current_base_pwm - g_line_current_turn_pwm));

    g_line_last_error = error;
    g_line_status.error = error;
    g_line_status.turn_pwm = g_line_current_turn_pwm;
}

LineFollow_Status LineFollow_GetStatus(void)
{
    return g_line_status;
}
