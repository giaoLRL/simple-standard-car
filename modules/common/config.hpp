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
#define ADC_MAX_VALUE         (4095U)  /* 12 位 ADC 最大码值 = 2^12 - 1 = 4095 */

/* ============================================================
 *  传感器配置
 * ============================================================ */

#define SENSOR_COUNT              (8U) /* 循迹传感器数量 */
#define ADC_SAMPLES_PER_CHANNEL   (8U) /* 每通道 ADC 采样次数（均值滤波）*/

/* 传感器物理位置（×1000 避免浮点），对称分布，左负右正 */
static const int16_t g_sensor_position[SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

/* 标定参数。手册规定正常状态必须白大黑小。 */
#define WHITE_STARTUP_SCANS       (50U) /* 启动时快速白底采样次数 */
#define MIN_CALIBRATION_SPAN      (120U)/* 每路最小有效黑白跨度 */
#define MAX_CALIBRATION_RETRIES   (10U) /* 最大标定重试次数 (防硬件故障永久阻塞) */
#define ADC_DMA_TIMEOUT_LOOPS  (1600U) /* DMA 超时轮询次数 (~300 µs @32 MHz, 8 次 ADC 采样 ≈ 8 µs + 余量) */

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
#define MUX_PIN_1 DL_GPIO_PIN_24  /* AD1 (硬件 AD1/AD2 接反, 引脚对调) */
#define MUX_PIN_2 DL_GPIO_PIN_25  /* AD2 (硬件 AD1/AD2 接反, 引脚对调) */
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
#define LEFT_MOTOR_REVERSED       (1)
#define RIGHT_MOTOR_REVERSED      (1)

/* H 桥方向切换死区延时 (NOP 循环数, ~2 µs @32 MHz) */
#define HBRIDGE_DEADTIME_LOOPS    (64U)

/* ============================================================
 *  控制参数 — 运行时可通过串口/蓝牙修改
 * ============================================================ */

extern uint16_t g_base_speed;       /* 基础速度（PWM 占空比, 默认 200）*/
extern uint16_t g_turn_outer_speed; /* 转弯外轮速度（默认 170）*/
extern uint16_t g_turn_inner_speed; /* 转弯内轮速度（默认 80）*/

#define LINE_PRESENT_MIN          (300U)  /* 归一化暗度总和低于此值视为丢线 */

/* PID 参数（通过 g_pid 对象直接访问: g_pid.kp, g_pid.ki, g_pid.kd）*/

/* ============================================================
 *  直角弯状态机参数（编译期常量）
 * ============================================================ */

extern uint16_t g_turn_timeout_ms;  /* 转弯最大允许时间(ms)，超时判丢线，默认2000 */
extern uint16_t g_turn_advance_ms;  /* 转弯前直走延时(ms), 默认50 */

/* ============================================================
 *  时序配置
 * ============================================================ */

#define CONTROL_PERIOD_MS         (10U)    /* 主循环周期（ms, 100Hz 匹配电机 τm≈20-100ms）*/

/* ============================================================
 *  编译期特性开关
 * ============================================================ */

#ifndef ENABLE_ENCODER
#define ENABLE_ENCODER 1                    /* 编码器速度闭环（已启用）*/
#endif

/* ============================================================
 *  速度环参数（编译期默认值）
 * ============================================================ */

#define SPEED_KP_DEFAULT      0.3f           /* 速度环 KP 默认值（降低防爆冲）*/
#define SPEED_KI_DEFAULT      0.02f          /* 速度环 KI 默认值 */
#define SPEED_KD_DEFAULT      0.0f           /* 速度环 KD 默认值 */
#define SPEED_SUM_LIMIT       500.0f         /* 速度环积分限幅 */
#define SPEED_OUT_LIMIT       1000.0f        /* 速度环输出限幅 (=PWM_MAX) */
#define SPEED_MAX_DELTA       120.0f         /* 速度环单步增量限幅（≈12% PWM/周期）*/

/* ============================================================
 *  编码器参数（MG310 霍尔编码器）
 * ============================================================ */

#define ENC_CPR               13             /* 编码器线数（电机轴每转脉冲数）*/
#define ENC_GEAR_RATIO        20             /* 减速比（1:20）*/
#define WHEEL_DIAMETER_MM     48             /* 轮径（周长 ≈ 150.8mm）*/
#define ENC_TIMEOUT_MS        100            /* T 法超时：超过此时间无脉冲 → 判定停车 */

/* 编码器方向反转：正转时 RPM 读数为负 → 置 1（运行时可通过 ENCDIR 命令切换）*/
#define ENC_LEFT_REVERSED      (0)
#define ENC_RIGHT_REVERSED     (0)

/* ============================================================
 *  用户按键配置（接地上拉输入，用于标定触发）
 * ============================================================ */

#define KEY_USER_PORT  GPIOB
#define KEY_USER_PIN   DL_GPIO_PIN_21

/* ============================================================
 *  蜂鸣器配置（有源蜂鸣器，高电平响，用于标定提示）
 * ============================================================ */

#define BUZZER_PORT  GPIOB
#define BUZZER_PIN   DL_GPIO_PIN_17

/* ============================================================
 *  运行时特性开关
 * ============================================================ */

extern bool g_line_track_on;    /* 循迹使能 */
extern bool g_motor_on;         /* 电机使能 */

#if ENABLE_ENCODER
extern bool g_speed_loop_on;    /* 速度环使能（SEN 命令）*/
extern bool g_dir_pid_on;       /* 方向环使能（DEN 命令，关闭后 correction=0）*/
extern float g_speed_kp;        /* 速度环 KP */
extern float g_speed_ki;        /* 速度环 KI */
extern float g_speed_kd;        /* 速度环 KD */
extern bool g_enc_left_rev;     /* 左编码器方向反转（运行时可切换）*/
extern bool g_enc_right_rev;    /* 右编码器方向反转（运行时可切换）*/
extern volatile bool g_spd_need_warmstart; /* 速度环需热启动（刚复位后首次进入）*/
#endif
