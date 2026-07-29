#include "ti_msp_dl_config.h"

volatile uint32_t g_sys_tick_10ms = 0U;

void TIMG7_IRQHandler(void)
{
    static uint8_t tick_divider = 0U;

    DL_TimerG_clearInterruptStatus(
        TIMER_0_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);

    tick_divider++;
    if (tick_divider >= 2U) {
        tick_divider = 0U;
        g_sys_tick_10ms++;
    }
}

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    while (1) {
        __WFI();
    }
}
