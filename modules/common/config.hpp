#pragma once
/*
 * config.hpp — 编译期常量、特性开关与运行时开关
 *
 * 所有可调参数集中在此文件，方便统一管理和外部（串口/蓝牙）修改。
 *
 * 参考: 感为无MCU灰度传感器例程 No_Mcu_Ganv_Grayscale_Sensor_Config.h
 *
 * 依赖: 无（仅使用标准整数类型）
 */

#include <stdint.h>

/* ============================================================
 *  ADC 分辨率配置
 * ============================================================ */

#define ADC_RESOLUTION_BITS   (12U)    /* ADC 分辨率: 8/10/12/14 */
#define ADC_MAX_VALUE         (4096U)  /* 2^12 = 4096（与分辨率对应）*/

/* ============================================================
 *  传感器配置
 * ============================================================ */

#define SENSOR_COUNT              (8U) /* 循迹传感器数量 */
#define ADC_SAMPLES_PER_CHANNEL   (8U) /* 每通道 ADC 采样次数（均值滤波）*/
#define MUX_SETTLE_US             (1U) /* 模拟开关切换后稳定等待（µs）*/

/* 传感器物理位置（×1000 避免浮点），对称分布，左负右正 */
static const int16_t g_sensor_position[SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

/* 自动标定参数 */
#define WHITE_STARTUP_SCANS       (20U) /* 启动时快速白底采样次数 */
#define INITIAL_BLACK_SPAN        (120U)/* 初始黑线跨度（ADC 原始值）*/

/* ============================================================
 *  GPIO 地址切换宏（CD4051 模拟开关）
 *
 *  硬件连接:
 *    AD0 = PB20（地址位 0）
 *    AD1 = PB25（地址位 1）
 *    AD2 = PB24（地址位 2）
 *
 *  参照感为例程的宏抽象风格，屏蔽底层 GPIO 操作细节。
 * ============================================================ */

#define MUX_PORT  GPIOB
#define MUX_PIN_0 DL_GPIO_PIN_20  /* AD0 */
#define MUX_PIN_1 DL_GPIO_PIN_25  /* AD1 */
#define MUX_PIN_2 DL_GPIO_PIN_24  /* AD2 */
#define MUX_MASK  (MUX_PIN_0 | MUX_PIN_1 | MUX_PIN_2)

/* 设置地址线：传入通道号 0~7，自动解码为三根地址线 */
#define SWITCH_MUX_CHANNEL(ch) do {                                         \
    DL_GPIO_clearPins(MUX_PORT, MUX_MASK);                                 \
    if ((ch) & 0x01U) { DL_GPIO_setPins(MUX_PORT, MUX_PIN_0); }           \
    if ((ch) & 0x02U) { DL_GPIO_setPins(MUX_PORT, MUX_PIN_1); }           \
    if ((ch) & 0x04U) { DL_GPIO_setPins(MUX_PORT, MUX_PIN_2); }           \
} while(0)

/* ============================================================
 *  电机 / PWM 配置
 * ============================================================ */

#define PWM_PERIOD_COUNTS         (1600U) /* PWM 周期计数值（20 kHz at 32 MHz）*/
#define PWM_MAX                   (1000)  /* 最大 PWM 占空比（量纲统一）*/

/* 方向反转开关：电机实际转动方向与预期相反时置 1 */
#define LEFT_MOTOR_REVERSED       (0)
#define RIGHT_MOTOR_REVERSED      (0)

/* ============================================================
 *  控制参数
 * ============================================================ */

#define BASE_SPEED                (450)   /* 基础速度（PWM 占空比）*/
#define LINE_PRESENT_MIN          (300U)  /* 归一化暗度总和低于此值视为丢线 */

/* PID 参数（此处只用 P 项，I/D 预留为 0 以便后续扩展）*/
extern float g_pid_kp;
extern float g_pid_ki;
extern float g_pid_kd;

/* 运行时通过宏重定向，方便外部（串口/蓝牙）修改参数 */
#define PID_KP g_pid_kp
#define PID_KI g_pid_ki
#define PID_KD g_pid_kd

/* ============================================================
 *  时序配置
 * ============================================================ */

#define CONTROL_PERIOD_MS         (2U)     /* 主循环周期（ms）*/

/* ============================================================
 *  编译期特性开关
 * ============================================================ */

#ifndef ENABLE_ENCODER
#define ENABLE_ENCODER 0                    /* 编码器速度闭环（默认关闭）*/
#endif

/* ============================================================
 *  运行时特性开关
 * ============================================================ */

extern bool g_line_track_on;    /* 循迹使能 */
extern bool g_motor_on;         /* 电机使能 */
