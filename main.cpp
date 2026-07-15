/*
 * main.cpp — 循迹小车入口点 + 依赖组装 + 主循环
 *
 * 功能: 8 路红外传感器循迹，P 控制器驱动双电机差速转向。
 *
 * 参考: 感为无MCU灰度传感器例程 main.c
 *   - 结构: 初始化 → 校准 → 循环（传感器任务 → 取数据 → 控制）
 *   - 数据: 模拟量 / 数字量 / 归一化量三种访问方式
 *
 * 硬件:
 *   传感器: 8 路红外对管 → CD4051 → ADC0_CH3 (PA24)
 *   左电机: TIMA1_CCP0 (PB4) + AIN1/2 (PA13/PA12)
 *   右电机: TIMA1_CCP1 (PB1) + BIN1/2 (PB0/PB8)
 *
 * 架构:
 *   业务层通过抽象接口（iinterfaces.hpp）访问硬件，
 *   main() 负责创建实例并注入依赖。
 *
 *   组装:
 *     LineSensor  → ISensorArray
 *     MotorDriver → IMotorDriver
 *
 * 已知限制:
 *   1. 纯 P 控制，高速时可能振荡
 *   2. 丢线即停车，不记忆恢复
 *   3. 无编码器速度闭环
 */

#include "ti_msp_dl_config.h"
#include "modules/common/config.hpp"
#include "modules/common/iinterfaces.hpp"
#include "modules/line_sensor/line_sensor.hpp"
#include "modules/motor/motor.hpp"

/* ============================================================
 *  全局可调参数（外部可通过串口 / 蓝牙修改）
 * ============================================================ */

float g_pid_kp = 0.06f;   /* 比例增益（error × kp = correction）*/
float g_pid_ki = 0.0f;    /* 积分增益（预留）*/
float g_pid_kd = 0.0f;    /* 微分增益（预留）*/

/* ============================================================
 *  运行时特性开关
 * ============================================================ */

bool g_line_track_on = true;
bool g_motor_on      = true;

/* ============================================================
 *  调试变量（供 CCS Expressions / J-Scope 实时观察）
 * ============================================================ */

volatile int32_t  g_dbg_error       = 0;      /* 线偏差 */
volatile int16_t  g_dbg_left_cmd    = 0;      /* 左轮指令 */
volatile int16_t  g_dbg_right_cmd   = 0;      /* 右轮指令 */
volatile bool     g_dbg_line_found  = false;  /* 是否检测到线 */
volatile uint8_t  g_dbg_digital     = 0;      /* 数字量（8 通道二值化）*/
volatile uint16_t g_dbg_normal[SENSOR_COUNT]; /* 归一化值 */
volatile uint16_t g_dbg_analog[SENSOR_COUNT]; /* 原始模拟量 */

/* ============================================================
 *  差速转向计算
 *
 *  符号约定（全链路一致）:
 *    error > 0       → 线偏右 → 需右转
 *    correction > 0  → 右转修正量 → 加到左轮 / 从右轮减去
 *
 *  控制律:
 *     correction = error × kp
 *     raw_left   = base + correction
 *     raw_right  = base - correction
 *
 *  关键性质:
 *    raw_left + raw_right = 2 × base  （左右速度和恒定 = 保持匀速）
 *    raw_left - raw_right = 2 × correction （差速 = 转向力）
 *
 *  饱和处理:
 *    当任一侧超出 [-PWM_MAX, PWM_MAX] 时，等比例缩放两侧，
 *    保持 left:right 比值不变（即转向曲率不变），同时双方都不超限。
 *    此时匀速率下降，但转向精度优先保证。
 * ============================================================ */

static int32_t compute_correction(int32_t error)
{
    return (int32_t)((float)error * PID_KP);
}

/*
 * 将 raw_left / raw_right 做对称限幅。
 * 保证: ① 双方均不超出 ±PWM_MAX  ② left:right 比值不变
 */
static void apply_diff_steering(int32_t raw_left, int32_t raw_right,
                                int16_t *out_left, int16_t *out_right)
{
    /* 找两侧绝对值最大者 */
    int32_t max_abs = (raw_left > 0) ? raw_left : -raw_left;
    int32_t tmp     = (raw_right > 0) ? raw_right : -raw_right;
    if (tmp > max_abs) {
        max_abs = tmp;
    }

    if (max_abs > PWM_MAX) {
        /* 等比例缩小：保持 left:right 比值 → 转向曲率不变 */
        raw_left  = (raw_left  * PWM_MAX) / max_abs;
        raw_right = (raw_right * PWM_MAX) / max_abs;
    }

    *out_left  = (int16_t)raw_left;
    *out_right = (int16_t)raw_right;
}

/* ============================================================
 *  main — 入口点 + 主循环
 * ============================================================ */

int main(void)
{
    /* ===== 1. 平台初始化 ===== */
    SYSCFG_DL_init();
    NVIC_EnableIRQ(LINE_ADC_INST_INT_IRQN);

    /* 确保电机初始停转 */
    g_motor.stop();

    /* ===== 2. 各模块硬件初始化 ===== */
    g_line_sensor.init();
    g_motor.init();

    /* ===== 3. 传感器白底标定 + 系数预计算 ===== */
    g_line_sensor.calibrate_white();

    /* ===== 4. 主循环 ===== */
    while (1) {
        /* ---- 传感器任务（参照 No_Mcu_Ganv_Sensor_Task_Without_tick）---- */
        bool line_found = g_line_sensor.update();
        int32_t error   = g_line_sensor.get_error();

        /* 获取三种数据（供调试器观察）*/
        g_dbg_digital = g_line_sensor.get_digital();
        g_line_sensor.get_normalized((uint16_t *)g_dbg_normal);
        g_line_sensor.get_analog((uint16_t *)g_dbg_analog);

        g_dbg_error      = error;
        g_dbg_line_found = line_found;

        /* ---- 控制决策 ---- */
        if (line_found && g_line_track_on && g_motor_on) {
            int32_t correction = compute_correction(error);
            int16_t left_cmd, right_cmd;
            apply_diff_steering(
                (int32_t)BASE_SPEED + correction,
                (int32_t)BASE_SPEED - correction,
                &left_cmd, &right_cmd);

            g_motor.set_speed(left_cmd, right_cmd);

            g_dbg_left_cmd  = left_cmd;
            g_dbg_right_cmd = right_cmd;
        } else {
            /* 丢线或功能关闭 → 停车 */
            g_motor.stop();
            g_dbg_left_cmd  = 0;
            g_dbg_right_cmd = 0;
        }

        /* ---- 固定周期（2 ms → 500 Hz）---- */
        delay_cycles(CONTROL_PERIOD_MS * 32000U);
    }
}
