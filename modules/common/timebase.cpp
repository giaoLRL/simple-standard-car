#include "modules/common/timebase.hpp"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis = 0;

void timebase_init(void)
{
    g_millis = 0;
    SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t timebase_millis(void)
{
    return g_millis;
}

extern "C" void SysTick_Handler(void)
{
    g_millis++;
}
