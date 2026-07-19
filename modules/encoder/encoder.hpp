#pragma once
/*
 * encoder.hpp / encoder.cpp — 双路霍尔编码器 T 法测速
 *
 * 硬件: MG310 霍尔编码器, 13 CPR（电机轴）, 自带上下拉整形
 * 引脚: E1A=PA15, E1B=PA16,  E2A=PA17, E2B=PB9（已由 syscfg 配置为 INPUT）
 *
 * 测速方式: T 法（GPIO 双边沿中断 + 微秒时间戳差值）
 *   motor_rpm = 60,000,000 / (pulse_period_us × 13 × 2)
 *
 * 中断: PA15/PA17 共享 GROUP1_IRQHandler，优先级 0（最高 NVIC）
 */

#include <cstdint>

struct Encoder {
    void init();                // 初始化 GPIO 双边沿中断
    void update();              // 每 10ms 调用，超时检测归零
    float get_speed_rpm() const { return reversed_ ? -speed_rpm : speed_rpm; }
    int32_t get_ticks()   const { return pulse_count; }
    void reset();               // 清零计数

    /* ISR 回调：由 GROUP1_IRQHandler 调用 */
    void on_pulse(uint32_t now_us);

    /* 运行时方向反转（正转 RPM 为负时置 true）*/
    bool reversed_ = false;

    /* ISR 与主循环共享字段，需 volatile 防编译器缓存 */
    volatile float    speed_rpm     = 0.0f;
    volatile int32_t  pulse_count   = 0;
    volatile uint32_t last_pulse_us = 0;    // 上次脉冲时间戳
    volatile uint32_t pulse_period_us = 0;  // 最近一次脉冲周期

private:
    volatile uint32_t last_rpm_update_us = 0; // 上次 RPM 更新时刻（超时检测用）
};

extern Encoder g_enc_left;   // 左轮编码器 (PA15)
extern Encoder g_enc_right;  // 右轮编码器 (PA17)
