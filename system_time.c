#include "system_time.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t system_milliseconds;

void system_time_init(void)
{
    system_milliseconds = 0U;
    SysTick->LOAD = (CPUCLK_FREQ / 1000U) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

uint32_t system_time_ms(void)
{
    return system_milliseconds;
}

void SysTick_Handler(void)
{
    system_milliseconds++;
}
