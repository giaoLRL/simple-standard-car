#include "modules/encoder/encoder.hpp"
#include "modules/common/config.hpp"
#include "modules/common/timebase.hpp"
#include "ti_msp_dl_config.h"

/* 全局单例 */
Encoder g_enc_left;
Encoder g_enc_right;

/*
 * GPIOA 中断初始化：PA15 + PA17 双边沿触发
 * syscfg 已配置为 INPUT，这里只需使能中断和设置触发方式
 */
void Encoder::init()
{
    /* 注意：init() 调用两次（左/右），用静态变量确保只初始化一次 */
    static bool gpio_int_inited = false;
    if (!gpio_int_inited) {
        gpio_int_inited = true;

        NVIC_SetPriority(GPIOA_INT_IRQn, 0);   /* 最高 NVIC 优先级 */
        NVIC_EnableIRQ(GPIOA_INT_IRQn);

        /* 清除上电残留中断标志 */
        DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_15 | DL_GPIO_PIN_17);

        /* PA15 在 POLARITY15_0 组，PA17 在 POLARITY31_16 组，分别设置双边沿 */
        DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_15_EDGE_RISE_FALL);
        DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_17_EDGE_RISE_FALL);

        DL_GPIO_enableInterrupt(GPIOA, DL_GPIO_PIN_15 | DL_GPIO_PIN_17);
    }
}

/* ISR 回调：记录时间戳，计算 RPM */
void Encoder::on_pulse(uint32_t now_us)
{
    pulse_count++;

    if (last_pulse_us != 0) {
        uint32_t period = now_us - last_pulse_us;
        if (period > 0) {
            pulse_period_us = period;
            /* 双边沿触发：每物理脉冲 2 个边沿，所以 rpm = 60M / (period_us × CPR × 2) */
            speed_rpm = 60000000.0f / (float)(period * ENC_CPR * 2U);
            last_rpm_update_us = now_us;
        }
    }
    last_pulse_us = now_us;
}

/*
 * 每 10ms 主循环调用：超时检测
 * 若超过 ENC_TIMEOUT_MS 没有脉冲 → 判定停车，RPM 归零
 */
void Encoder::update()
{
    uint32_t now_us = timebase_micros();
    if (last_rpm_update_us != 0 &&
        (now_us - last_rpm_update_us) >= (uint32_t)(ENC_TIMEOUT_MS * 1000U))
    {
        speed_rpm = 0.0f;
        last_rpm_update_us = 0;
    }
}

void Encoder::reset()
{
    pulse_count     = 0;
    speed_rpm       = 0.0f;
    last_pulse_us   = 0;
    pulse_period_us = 0;
    last_rpm_update_us = 0;
}

/* ============================================================
 *  GROUP1_IRQHandler — GPIOA 中断（编码器脉冲）
 *
 *  MSPM0G3507 SDK 2.x: GPIOA 通过 GROUP1 共享中断向量路由。
 *  GPIOA_INT_IRQn 是 GROUP1_IRQn 的别名。
 *  ISR 内先读 GROUP IIDX 确认是 GPIOA 触发，再读引脚 IIDX
 *  循环处理所有挂起的中断（防止同时触发丢失）。
 *
 *  PA15 = 左编码器 A 相 (E1A)
 *  PA17 = 右编码器 A 相 (E2A)
 * ============================================================ */
extern "C" void GROUP1_IRQHandler(void)
{
    uint32_t group_iidx = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    if (group_iidx == DL_INTERRUPT_GROUP1_IIDX_GPIOA) {
        uint32_t now_us = timebase_micros();   /* ★ 第一行抓时间戳 */

        DL_GPIO_IIDX pin_iidx;
        while ((pin_iidx = DL_GPIO_getPendingInterrupt(GPIOA))
               != DL_GPIO_IIDX_NO_INTR) {
            switch (pin_iidx) {
            case DL_GPIO_IIDX_DIO15:
                DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_15);
                g_enc_left.on_pulse(now_us);
                break;
            case DL_GPIO_IIDX_DIO17:
                DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_17);
                g_enc_right.on_pulse(now_us);
                break;
            default:
                /* 未预期的 GPIOA 中断引脚 — 清除标志，避免死循环 */
                DL_GPIO_clearInterruptStatus(GPIOA,
                    (1UL << (uint32_t)pin_iidx));
                break;
            }
        }
    }
}
