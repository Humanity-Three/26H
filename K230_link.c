#include "K230_link.h"

#include "ti_msp_dl_config.h"

#define K230_RX_BUFFER_SIZE       (128U)
#define K230_LINE_BUFFER_SIZE     (64U)

static volatile uint8_t g_rx_buffer[K230_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static char g_line_buffer[K230_LINE_BUFFER_SIZE + 2U];
static uint8_t g_line_length;
static K230_LinkData g_link_data;
static volatile uint32_t g_rx_byte_count;

static void K230_StoreRxByte(uint8_t value)
{
    uint16_t next =
        (uint16_t)((g_rx_head + 1U) % K230_RX_BUFFER_SIZE);

    g_rx_byte_count++;
    if (next != g_rx_tail)
    {
        g_rx_buffer[g_rx_head] = value;
        g_rx_head = next;
    }
}

static char K230_ToUpper(char value)
{
    if ((value >= 'a') && (value <= 'z'))
    {
        return (char)(value - ('a' - 'A'));
    }
    return value;
}

static int16_t K230_ParseNumber(const char *text)
{
    int32_t value = 0;
    int32_t sign = 1;

    if (*text == '-')
    {
        sign = -1;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        value = value * 10 + (*text - '0');
        if (value > 32767)
        {
            value = 32767;
        }
        text++;
    }
    return (int16_t)(value * sign);
}

static void K230_ParseLine(char *line)
{
    char *dx = 0;
    char *dy = 0;
    char *aligned = 0;
    char *detected = 0;
    uint8_t i;

    for (i = 0U; line[i] != '\0'; i++)
    {
        if ((K230_ToUpper(line[i]) == 'D') &&
            (K230_ToUpper(line[i + 1U]) == 'X') &&
            (line[i + 2U] == ':'))
        {
            dx = &line[i + 3U];
        }
        else if ((K230_ToUpper(line[i]) == 'D') &&
                 (K230_ToUpper(line[i + 1U]) == 'Y') &&
                 (line[i + 2U] == ':'))
        {
            dy = &line[i + 3U];
        }
        else if ((K230_ToUpper(line[i]) == 'X') &&
                 ((line[i + 1U] == '+') || (line[i + 1U] == '-')))
        {
            dx = &line[i + 1U];
        }
        else if ((K230_ToUpper(line[i]) == 'Y') &&
                 ((line[i + 1U] == '+') || (line[i + 1U] == '-')))
        {
            dy = &line[i + 1U];
        }
        else if ((K230_ToUpper(line[i]) == 'A') &&
                 ((line[i + 1U] == '0') || (line[i + 1U] == '1')))
        {
            aligned = &line[i + 1U];
        }
        else if ((K230_ToUpper(line[i]) == 'D') &&
                 ((line[i + 1U] == '0') || (line[i + 1U] == '1')))
        {
            detected = &line[i + 1U];
        }
    }

    /*
     * A control frame is valid only when position and detection state are
     * explicit. Never treat a truncated "DX" frame as a detected target.
     */
    if ((dx != 0) && (dy != 0) && (detected != 0))
    {
        g_link_data.dx = K230_ParseNumber(dx);
        g_link_data.dy = K230_ParseNumber(dy);
        g_link_data.aligned =
            (aligned != 0) ? (*aligned == '1') : false;
        g_link_data.detected = (*detected == '1');
        g_link_data.age_ms = 0U;
        g_link_data.frame_count++;
    }
}

void K230_Link_Init(void)
{
    /*
     * SYSCFG_DL_init() configures this UART before the final SYSPLL clock
     * switch.  Re-apply the generated UART setup here, after the clock tree
     * is stable, so the 80 MHz / 115200 baud divisors take effect against
     * the actual runtime bus clock.
     */
    DL_UART_Main_disable(K230_UART_INST);
    SYSCFG_DL_K230_UART_init();

    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_line_length = 0U;
    g_rx_byte_count = 0U;
    g_link_data.dx = 0;
    g_link_data.dy = 0;
    g_link_data.aligned = false;
    g_link_data.detected = false;
    g_link_data.age_ms = UINT32_MAX;
    g_link_data.frame_count = 0U;

    /*
     * A complete K230 text frame is longer than the hardware RX FIFO and is
     * transmitted as one burst. Drain it in the RX interrupt so polling every
     * 10 ms cannot lose the middle of a frame. Foreground polling remains as
     * a fallback for bytes already present before the interrupt is enabled.
     */
    DL_UART_Main_enableInterrupt(
        K230_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(K230_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_UART_INST_INT_IRQN);
}

void K230_Link_Update10ms(void)
{
    if (g_link_data.age_ms <= (UINT32_MAX - 10U))
    {
        g_link_data.age_ms += 10U;
    }

    while (g_rx_tail != g_rx_head)
    {
        uint8_t value = g_rx_buffer[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1U) %
                               K230_RX_BUFFER_SIZE);

        if ((value == '\r') || (value == '\n'))
        {
            if (g_line_length != 0U)
            {
                g_line_buffer[g_line_length] = '\0';
                g_line_buffer[g_line_length + 1U] = '\0';
                g_line_buffer[g_line_length + 2U] = '\0';
                K230_ParseLine(g_line_buffer);
                g_line_length = 0U;
            }
        }
        else if (g_line_length < (K230_LINE_BUFFER_SIZE - 1U))
        {
            g_line_buffer[g_line_length++] = (char)value;
        }
        else
        {
            g_line_length = 0U;
        }
    }
}

const K230_LinkData *K230_Link_GetData(void)
{
    return &g_link_data;
}

bool K230_Link_IsValid(uint32_t timeout_ms)
{
    return (g_link_data.frame_count != 0U) &&
           (g_link_data.age_ms <= timeout_ms);
}

uint32_t K230_Link_GetRxByteCount(void)
{
    return g_rx_byte_count;
}

void K230_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(K230_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST))
            {
                uint8_t value =
                    DL_UART_Main_receiveData(K230_UART_INST);
                K230_StoreRxByte(value);
            }
            break;
        default:
            break;
    }
}
