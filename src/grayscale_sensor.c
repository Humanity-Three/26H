#include "grayscale_sensor.h"

#define IR_FRAME_MAX_LENGTH       (64U)
#define IR_CONFIG_RETRY_READS     (100U)
#define IR_FRAME_TIMEOUT_READS    (20U)
#define IR_RECENT_BYTE_COUNT      (8U)

static volatile uint8_t g_ir_sensor_mask;
static volatile bool g_ir_frame_valid;
static volatile uint8_t g_ir_frame_age_reads;
static volatile uint32_t g_ir_rx_byte_count;
static volatile uint32_t g_ir_frame_count;
static uint8_t g_ir_frame[IR_FRAME_MAX_LENGTH];
static uint8_t g_ir_frame_length;
static volatile uint8_t g_ir_last_frame[IR_FRAME_MAX_LENGTH];
static volatile uint8_t g_ir_last_frame_length;
static volatile uint8_t g_ir_recent_bytes[IR_RECENT_BYTE_COUNT];
static volatile uint8_t g_ir_recent_write_index;
static volatile uint8_t g_ir_recent_count;
static uint16_t g_ir_config_retry_reads;

static void Grayscale_SendConfig(void)
{
    static const uint8_t command[] = "$0,0,1#";
    uint8_t i;

    for (i = 0U; i < (uint8_t)(sizeof(command) - 1U); i++)
    {
        while (DL_UART_Main_isTXFIFOFull(IR_UART_INST))
        {
        }
        DL_UART_Main_transmitData(IR_UART_INST, command[i]);
    }
}

static bool Grayscale_ParseFrame(void)
{
    uint8_t channel = 0U;
    uint8_t mask = 0U;
    uint8_t i;

    if ((g_ir_frame_length < 4U) || (g_ir_frame[0] != '$') ||
        (g_ir_frame[1] != 'D'))
    {
        return false;
    }

    /* Module format: $D,x1:0,x2:1,...,x8:1#; 0 means black. */
    for (i = 2U; (i + 1U < g_ir_frame_length) &&
                 (channel < GRAYSCALE_SENSOR_CHANNELS); i++)
    {
        if ((g_ir_frame[i] == ':') &&
            ((g_ir_frame[i + 1U] == '0') ||
             (g_ir_frame[i + 1U] == '1')))
        {
            if (g_ir_frame[i + 1U] == '0')
            {
                mask |= (uint8_t)(1U << channel);
            }
            channel++;
            i++;
        }
    }

    if (channel == GRAYSCALE_SENSOR_CHANNELS)
    {
        g_ir_sensor_mask = mask;
        g_ir_frame_valid = true;
        g_ir_frame_age_reads = 0U;
        g_ir_frame_count++;
        return true;
    }
    return false;
}

static void Grayscale_StoreRxByte(uint8_t value)
{
    uint8_t i;

    g_ir_rx_byte_count++;
    g_ir_recent_bytes[g_ir_recent_write_index] = value;
    g_ir_recent_write_index = (uint8_t)(
        (g_ir_recent_write_index + 1U) % IR_RECENT_BYTE_COUNT);
    if (g_ir_recent_count < IR_RECENT_BYTE_COUNT)
    {
        g_ir_recent_count++;
    }
    if (value == '$')
    {
        g_ir_frame[0] = value;
        g_ir_frame_length = 1U;
        return;
    }
    if (g_ir_frame_length == 0U)
    {
        return;
    }
    if (g_ir_frame_length >= IR_FRAME_MAX_LENGTH)
    {
        g_ir_frame_length = 0U;
        return;
    }
    g_ir_frame[g_ir_frame_length++] = value;
    /* Some module firmware terminates with CR/LF instead of '#'. */
    if (Grayscale_ParseFrame())
    {
        g_ir_last_frame_length = g_ir_frame_length;
        for (i = 0U; i < g_ir_frame_length; i++)
        {
            g_ir_last_frame[i] = g_ir_frame[i];
        }
        g_ir_frame_length = 0U;
        return;
    }
    if (value == '#')
    {
        g_ir_last_frame_length = g_ir_frame_length;
        for (i = 0U; i < g_ir_frame_length; i++)
        {
            g_ir_last_frame[i] = g_ir_frame[i];
        }
        g_ir_frame_length = 0U;
    }
}

void Grayscale_Sensor_Init(void)
{
    g_ir_sensor_mask = 0U;
    g_ir_frame_valid = false;
    g_ir_frame_age_reads = 0U;
    g_ir_frame_length = 0U;
    g_ir_last_frame_length = 0U;
    g_ir_recent_write_index = 0U;
    g_ir_recent_count = 0U;
    g_ir_config_retry_reads = 0U;
    g_ir_rx_byte_count = 0U;
    g_ir_frame_count = 0U;
    NVIC_ClearPendingIRQ(IR_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(IR_UART_INST_INT_IRQN);
    Grayscale_SendConfig();
}

void Grayscale_Sensor_Update10ms(void)
{
    if (!g_ir_frame_valid)
    {
        g_ir_config_retry_reads++;
        if (g_ir_config_retry_reads >= IR_CONFIG_RETRY_READS)
        {
            g_ir_config_retry_reads = 0U;
            Grayscale_SendConfig();
        }
    }
    else if (g_ir_frame_age_reads < IR_FRAME_TIMEOUT_READS)
    {
        g_ir_frame_age_reads++;
        if (g_ir_frame_age_reads >= IR_FRAME_TIMEOUT_READS)
        {
            /* Treat a 200 ms UART silence as line loss, never stale steering. */
            g_ir_frame_valid = false;
            g_ir_sensor_mask = 0U;
            g_ir_config_retry_reads = 0U;
        }
    }
}

uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS)
    {
        return 0U;
    }
    return (g_ir_sensor_mask & (uint8_t)(1U << channel)) ? 1U : 0U;
}

void Grayscale_Sensor_Read_All(uint16_t *sensor_values)
{
    uint8_t i;

    if (sensor_values == 0) {
        return;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        sensor_values[i] = Grayscale_Sensor_Read_Single(i);
    }
}

bool Grayscale_Sensor_IsValid(void)
{
    return g_ir_frame_valid;
}

uint8_t Grayscale_Sensor_GetMask(void)
{
    return g_ir_sensor_mask;
}

uint32_t Grayscale_Sensor_GetRxByteCount(void)
{
    return g_ir_rx_byte_count;
}

uint32_t Grayscale_Sensor_GetFrameCount(void)
{
    return g_ir_frame_count;
}

uint8_t Grayscale_Sensor_GetLastFrameLength(void)
{
    return g_ir_last_frame_length;
}

char Grayscale_Sensor_GetLastFrameChar(uint8_t index)
{
    if (index >= g_ir_last_frame_length)
    {
        return ' ';
    }
    if ((g_ir_last_frame[index] < 32U) ||
        (g_ir_last_frame[index] > 126U))
    {
        return '.';
    }
    return (char)g_ir_last_frame[index];
}

uint8_t Grayscale_Sensor_GetRecentByte(uint8_t index)
{
    uint8_t start;

    if (index >= g_ir_recent_count)
    {
        return 0U;
    }
    start = (uint8_t)((g_ir_recent_write_index + IR_RECENT_BYTE_COUNT -
                       g_ir_recent_count) % IR_RECENT_BYTE_COUNT);
    return g_ir_recent_bytes[(start + index) % IR_RECENT_BYTE_COUNT];
}

uint8_t Grayscale_Sensor_GetRecentByteCount(void)
{
    return g_ir_recent_count;
}

void IR_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(IR_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(IR_UART_INST))
            {
                Grayscale_StoreRxByte(
                    DL_UART_Main_receiveData(IR_UART_INST));
            }
            break;
        default:
            break;
    }
}

uint8_t Grayscale_Sensor_BuildMask(const uint16_t *sensor_values)
{
    uint8_t i;
    uint8_t mask = 0U;

    if (sensor_values == 0) {
        return 0U;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (sensor_values[i] != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}
