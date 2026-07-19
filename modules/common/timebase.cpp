#include "modules/common/timebase.hpp"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_millis = 0;

/* SysTick 调试心跳（CCS Expressions 可直接观察）*/
volatile uint32_t g_systick_cnt = 0;

void timebase_init(void)
{
    g_millis = 0;
    g_systick_cnt = 0;
    SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t timebase_millis(void)
{
    return g_millis;
}

extern "C" void SysTick_Handler(void)
{
    g_millis++;
    g_systick_cnt++;
}

/*
 * 微秒级时间戳：利用 SysTick 递减计数器 VAL 反推
 *   分辨率 = CPUCLK_FREQ / 1000000 (32MHz → 32 ticks/μs ≈ 0.03μs)
 *
 *   do-while 循环防跨毫秒翻转竞态：
 *   若读取 g_millis 和 VAL 之间发生了 SysTick 中断，
 *   则 g_millis 已更新但 VAL 已重载，导致 μs 值回退；
 *   此时重新读取确保 ms 和 val 来自同一个 SysTick 周期。
 */
uint32_t timebase_micros(void)
{
    uint32_t ms, val;
    do {
        ms  = g_millis;
        val = SysTick->VAL;
    } while (ms != g_millis);

    uint32_t load = CPUCLK_FREQ / 1000U;
    uint32_t us_per_tick = CPUCLK_FREQ / 1000000U;
    return ms * 1000U + (load - val) / us_per_tick;
}
