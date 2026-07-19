#pragma once
/*
 * speed_pid.hpp — 速度环 PID 双实例声明
 *
 * 复用 pid.hpp 的增量式 PID，左右轮各一个独立实例。
 * 速度环输出直接驱动 PWM，与方向环形成级联控制。
 *
 * 增量式 PID 特性:
 *   - 输出平滑，切换目标速度时无跳变
 *   - 内置 output_decay 防积分饱和
 *   - 支持 max_delta 步长限幅
 */

#include "modules/pid/pid.hpp"

extern PID g_spid_left;   /* 左轮速度 PID */
extern PID g_spid_right;  /* 右轮速度 PID */
