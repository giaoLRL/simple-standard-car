/**
 * @brief       陀螺仪闭环控制 — 全局声明
 * @details     陀螺 PID 实例 + 模式开关 + 调试变量。
 *              陀螺模式开启时完全替代循迹方向 PID，维持目标航向角。
 * @author      Haoqi Liu
 * @date        2026-07-26
 */

#pragma once

#include "modules/pid/pid.hpp"
#include "modules/gyro/mpu6050.hpp"
#include "modules/common/config.hpp"

#if ENABLE_GYRO

/* 陀螺方向 PID — 位置式
 * 输入: 角度误差 ()= target_angle - current_angle
 * 输出: PWM 修正量 (加到左轮/从右轮减) */
extern PID g_gyro_pid;

/* 模式开关 */
extern bool  g_gyro_mode_on;
extern float g_gyro_target_angle;

/* 运行时可调的 PID 参数 */
extern float g_gyro_kp;
extern float g_gyro_ki;
extern float g_gyro_kd;

/* 调试变量 (CCS Expressions 实时观察) */
extern volatile float   g_dbg_gyro_angle;
extern volatile float   g_dbg_gyro_dps;
extern volatile int32_t g_dbg_gyro_corr;

/*
 * I2C/初始化调试变量 (CCS Expressions 实时观察)
 *
 *  g_dbg_gyro_step:  0=进入init  1=等待MPU6050上电  2=读WHO_AM_I
 *                    3=唤醒  7=init完成
 *  g_dbg_gyro_who:   WHO_AM_I 读回值 (应为 0x68 或 0x70)
 *  g_dbg_i2c_msr:    I2C 控制器状态寄存器瞬时值
 *  g_i2c_dbg:        I2C 失败详情 (fail_step/reason/status)
 */
extern volatile uint8_t  g_dbg_gyro_step;
extern volatile uint8_t  g_dbg_gyro_ack;
extern volatile uint8_t  g_dbg_gyro_who;
extern volatile uint32_t g_dbg_i2c_msr;

#endif /* ENABLE_GYRO */
