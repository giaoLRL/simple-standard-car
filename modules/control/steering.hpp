#pragma once
/*
 * steering.hpp — 差速转向计算 (公共工具)
 *
 * 符号约定 (全链路一致):
 *   sensor error > 0 → 线偏右 → 需右转
 *   correction  > 0 → 右转修正 → 加到左轮 / 从右轮减去
 *
 * 控制律:
 *   raw_left  = base + correction
 *   raw_right = base - correction
 *
 * 饱和处理:
 *   当任一侧超出 [-PWM_MAX, PWM_MAX] 时等比例缩放两侧，
 *   保持 left:right 比值不变 (转向曲率不变)。
 */

#include <stdint.h>
#include <cmath>

/**
 * @brief 差速转向对称限幅
 * @param raw_left   左轮原始指令 (未限幅)
 * @param raw_right  右轮原始指令 (未限幅)
 * @param out_left   输出: 左轮限幅后指令
 * @param out_right  输出: 右轮限幅后指令
 * @param pwm_max    最大 PWM 占空比 (默认 1000)
 *
 * 保证: ① 双方均不超出 ±pwm_max  ② left:right 比值不变
 */
inline void apply_diff_steering(int32_t raw_left, int32_t raw_right,
                                int16_t *out_left, int16_t *out_right,
                                int32_t pwm_max = 1000)
{
    /* 找两侧绝对值最大者 */
    int32_t max_abs = (raw_left > 0) ? raw_left : -raw_left;
    int32_t tmp     = (raw_right > 0) ? raw_right : -raw_right;
    if (tmp > max_abs) {
        max_abs = tmp;
    }

    if (max_abs > pwm_max) {
        /* 等比例缩小: 保持 left:right 比值 → 转向曲率不变 */
        raw_left  = (raw_left  * pwm_max) / max_abs;
        raw_right = (raw_right * pwm_max) / max_abs;
    }

    *out_left  = (int16_t)raw_left;
    *out_right = (int16_t)raw_right;
}

/**
 * @brief float → int32_t 四舍五入 (无 libm 依赖)
 *
 * 替代 (int32_t) 截断或 lrintf()，消除 ±1 死区。
 * NaN/Inf 输入返回 0 (float→int 强转 NaN 为未定义行为)。
 */
inline int32_t round_to_i32(float x) {
    if (!std::isfinite(x)) {
        return 0;
    }
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}
