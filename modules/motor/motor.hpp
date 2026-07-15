#pragma once
/*
 * motor.hpp / motor.cpp — 双电机驱动模块
 *
 * 硬件:
 *   左电机 — PWM: TIMA1_CCP0 (PB4), 方向: AIN1=PA13 / AIN2=PA12
 *   右电机 — PWM: TIMA1_CCP1 (PB1), 方向: BIN1=PB0  / BIN2=PB8
 *
 * 依赖: IMotorDriver（实现接口）、config.hpp（参数）、ti_msp_dl_config.h（外设定义）
 *
 * 设计要点:
 *   1. 正值为前进、负值为后退，量纲与 PWM_MAX 统一
 *   2. 通过 LEFT_MOTOR_REVERSED / RIGHT_MOTOR_REVERSED 支持硬件接线反转
 *   3. 设速为 0 时同时关闭两路方向信号（节能 + 防意外短路）
 */

#include "ti_msp_dl_config.h"
#include "modules/common/config.hpp"
#include "modules/common/iinterfaces.hpp"

/* ============================================================
 *  MotorDriver — 双电机 PWM 驱动
 * ============================================================ */

class MotorDriver : public IMotorDriver {
public:
    MotorDriver();

    /* 硬件初始化（PWM 已在 SYSCFG_DL_init 中配置）*/
    void init();

    /* IMotorDriver 实现 */
    void set_speed(int16_t left, int16_t right) override;
    void stop() override;

    /* 调试访问器（const 成员函数，只读，不破坏封装）*/
    int16_t get_left_command()  const { return left_command_; }
    int16_t get_right_command() const { return right_command_; }

private:
    /* 设置单个 PWM 通道占空比 */
    void set_pwm_(DL_TIMER_CC_INDEX channel, uint16_t magnitude);

    /* 设置左/右电机（含方向引脚控制）*/
    void set_left_(int16_t command);
    void set_right_(int16_t command);

    /* 限幅到 [-PWM_MAX, PWM_MAX] */
    static int16_t clamp_(int32_t value);

    int16_t left_command_;
    int16_t right_command_;
};

extern MotorDriver g_motor;   /* 全局单例 */
