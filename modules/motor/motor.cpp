/*
 * motor.cpp — 双电机驱动模块实现
 *
 * 硬件: TIMA1 PWM + GPIO 方向控制
 * 依赖: ti_msp_dl_config.h
 *
 * 已知限制:
 *   1. PWM 更新为立即生效模式，高频更新可能产生毛刺
 *   2. 方向切换无硬件死区——连续正反转切换时需由上层控制时序。
 *      方向反转时，旧方向的 PWM 保持到新 PWM 值写入前，可能导致短暂
 *      桥臂直通电流尖峰。上层应避免在单个控制周期内做正反转跳变。
 */

#include "modules/motor/motor.hpp"

/* 全局单例 */
MotorDriver g_motor;

/* ============================================================
 *  构造与初始化
 * ============================================================ */

MotorDriver::MotorDriver()
    : left_command_{0}
    , right_command_{0}
{
}

void MotorDriver::init()
{
    /*
     * PWM 和 GPIO 已在 SYSCFG_DL_init() 中初始化，但计数器保持停止。
     * 先写入零占空比并清除方向，再启动 PWM，避免上电毛刺驱动电机。
     */
    stop();
    DL_TimerA_startCounter(MOTOR_PWM_INST);
}

/* ============================================================
 *  IMotorDriver 接口实现
 * ============================================================ */

void MotorDriver::set_speed(int16_t left, int16_t right)
{
    left_command_  = clamp_(left);
    right_command_ = clamp_(right);

    set_left_(left_command_);
    set_right_(right_command_);
}

void MotorDriver::stop()
{
    left_command_  = 0;
    right_command_ = 0;

    set_left_(0);
    set_right_(0);
}

/* ============================================================
 *  私有驱动函数
 * ============================================================ */

void MotorDriver::set_pwm_(DL_TIMER_CC_INDEX channel, uint16_t magnitude)
{
    /*
     * 边沿对齐模式，递减计数。
     * 写 (period - duty_counts) 获得目标占空比。
     */
    uint32_t duty_counts =
        ((uint32_t)magnitude * PWM_PERIOD_COUNTS) / PWM_MAX;
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST,
        PWM_PERIOD_COUNTS - duty_counts, channel);
}

void MotorDriver::set_left_(int16_t command)
{
    bool forward       = (command >= 0);
    uint16_t magnitude = (uint16_t)(forward ? command : -command);

    /* 方向反转开关（硬件接线反了时在 config.hpp 中置 1）*/
    forward = LEFT_MOTOR_REVERSED ? !forward : forward;

    /* 方向切换死区: 先关 PWM → 延时 → 翻转方向 → 恢复 PWM,
     * 防止 H 桥上下臂同时导通 (桥臂直通)。 */
    if (magnitude > 0U && forward != left_was_forward_) {
        set_pwm_(DL_TIMER_CC_0_INDEX, 0U);        /* 先关断 PWM */
        for (volatile uint8_t d = 0; d < HBRIDGE_DEADTIME_LOOPS; d++) { __NOP(); }
    }
    /* 仅在 magnitude>0 时更新方向记录, 停止时不覆写;
     *   避免 stop→reverse 跳变时因记录被强制改 true 而触发无谓死区延时。*/
    if (magnitude > 0U) {
        left_was_forward_ = forward;
    }

    if (magnitude == 0U) {
        /* 停止：关闭两路方向输出，防止短路 */
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12 | DL_GPIO_PIN_13);
    } else if (forward) {
        /* 正转：AIN1=1, AIN2=0 */
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12);
    } else {
        /* 反转：AIN1=0, AIN2=1 */
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12);
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13);
    }

    set_pwm_(DL_TIMER_CC_0_INDEX, magnitude);
}

void MotorDriver::set_right_(int16_t command)
{
    bool forward       = (command >= 0);
    uint16_t magnitude = (uint16_t)(forward ? command : -command);

    forward = RIGHT_MOTOR_REVERSED ? !forward : forward;

    /* 方向切换死区 (同上) */
    if (magnitude > 0U && forward != right_was_forward_) {
        set_pwm_(DL_TIMER_CC_1_INDEX, 0U);
        for (volatile uint8_t d = 0; d < HBRIDGE_DEADTIME_LOOPS; d++) { __NOP(); }
    }
    /* 同左电机: 停止时不覆写方向记录, 保持真实最后方向。*/
    if (magnitude > 0U) {
        right_was_forward_ = forward;
    }

    if (magnitude == 0U) {
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0 | DL_GPIO_PIN_8);
    } else if (forward) {
        /* 正转：BIN1=1, BIN2=0 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_0);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_8);
    } else {
        /* 反转：BIN1=0, BIN2=1 */
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_8);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_0);
    }

    set_pwm_(DL_TIMER_CC_1_INDEX, magnitude);
}

int16_t MotorDriver::clamp_(int32_t value)
{
    /* 防护: -INT16_MIN 为 UB, 但当前 value 范围受 PWM_MAX (±1000) 限制 */
    if (value > PWM_MAX) {
        return (int16_t)PWM_MAX;
    }
    if (value < -PWM_MAX) {
        return (int16_t)(-PWM_MAX);
    }
    return (int16_t)value;
}
