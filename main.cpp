/*
 * main.cpp — 循迹小车入口点 + 依赖组装 + 主循环
 *
 * 功能: 8 路红外传感器循迹，增量式 PID 驱动双电机差速转向。
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
 *     PID         → 差速转向 (steering.hpp)
 *
 * 已知限制:
 *   1. 增量式 PID 控制，已添加输出衰减和 D 项滤波；定点化待迁移
 *   2. 丢线即停车并蜂鸣器间歇报警；支持左/右直角弯状态机
 *   3. 无编码器速度闭环
 *   4. 传感器 ADC 配置为 repeat 模式，由 DMA burst 搬运 8 个样本
 *   5. MSPM0G3507 (Cortex-M0+) 无硬件 FPU，浮点 PID 由软浮点模拟，
 *      当前控制周期 10ms 可满足且裕量充足
 */

#include "ti_msp_dl_config.h"
#include "modules/common/config.hpp"
#include "modules/common/iinterfaces.hpp"
#include "modules/common/buzzer.hpp"
#include "modules/common/timebase.hpp"
#include "modules/control/turn_state_machine.hpp"
#include "modules/control/steering.hpp"
#include "modules/line_sensor/line_sensor.hpp"
#include "modules/motor/motor.hpp"
#include "modules/pid/pid.hpp"
#include "modules/common/uart_protocol.hpp"

/* ============================================================
 *  全局可调参数（外部可通过串口 / 蓝牙修改）
 * ============================================================ */

/* 位置式 PID 控制器
 *
 *   u = Kp×e + Ki×∫e + Kd×(e[k]−e[k−1])
 *   位置式特性: 输出与误差直接成比例，error→0 则输出→0，
 *   天然趋近零误差——无需 decay 衰减、无需增量步长限幅。
 *   积分限幅防饱和。
 *   参考: 国赛底盘控制 Kp=1.0 (bias ±3), 本项目等效:
 *     Kp=0.08 在 error=±3500 时 P 项输出 ≈ ±280 (钳位后 ±200)。 */
PID g_pid(PID::position_type, 0.05f, 0.0000f, 0.13f,
          100000.0f, -100000.0f,  /* sum_error 限幅, 防积分饱和 */
          (float)PWM_MAX, -(float)PWM_MAX,
          NAN,    /* deriv_lpf_alpha — 位置式不使用 */
          NAN,    /* output_decay — 位置式不使用 */
          NAN);   /* max_delta — 位置式不使用 */

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
 *     correction = positional_pid(error)   （位置式 PID）
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

/*
 * 差速转向计算 —— 已提取至 modules/control/steering.hpp
 * ============================================================ */

/*
 * 传感器误差死区: |error| < 此值时 PID 输入置零,
 * 减少直线段电机微振和功耗。单位为传感器位置量纲 (千分位)。
 */
static constexpr int32_t SENSOR_ERROR_DEADBAND = 5;  /* |error|<=5 置零, 减少直线微振; 修复合入不等式 */

/* ============================================================
 *  main — 入口点 + 主循环
 * ============================================================ */

int main(void)
{
    /* ===== 1. 平台初始化 ===== */
    SYSCFG_DL_init();
    /* 使能 DMA 完成中断（ADC 结果改由 DMA 搬运，不再用 ADC 中断） */
    NVIC_EnableIRQ(DMA_INT_IRQn);
    timebase_init();

    /* 确保电机初始停转 */
    g_motor.stop();

    /* ===== 2. 各模块硬件初始化 ===== */
    g_line_sensor.init();
    g_motor.init();
    buzzer_init();
    turn_fsm_init();
    proto_init();

    /* 启动短响：确认当前固件已运行，随后进入 PB21 白底标定等待。 */
    buzzer_beep(80U);

    /* ===== 3. 传感器黑白双标定 + 系数预计算 ===== */
    /*
     * 流程：上电后把 8 路探头全部放到白底，按 PB21 按键；
     *       再把 8 路探头全部放到黑线，再按一次 PB21 按键。
     */
    g_line_sensor.calibrate();

    /* 标定失败防护: 若所有重试耗尽仍不通过，死锁停车 + 持续报警，
     * 防止用全零标定数据盲开 (此时 error_ 恒为 0, 车会直冲)。 */
    if (!g_line_sensor.is_ready()) {
        g_motor.stop();
        while (1) {
            buzzer_beep(200U);
            delay_cycles(300U * (CPUCLK_FREQ / 1000U));
        }
    }

    /* ---- 电机方向自检 ----
     * 标定成功后, 两轮同时正转 500ms。观察小车:
     *   - 前进 ✓ → 方向正确
     *   - 后退   → 两轮接线都反了, config.hpp 中两个 REVERSED 都置 1
     *   - 原地转 → 某一轮方向反了, 单独置位该轮的 REVERSED 宏 */
    delay_cycles(800U * (CPUCLK_FREQ / 1000U));
    g_motor.set_speed(300, 300);
    {
        uint32_t t0 = timebase_millis();
        while ((int32_t)(timebase_millis() - t0) < 500) { __NOP(); }
    }
    g_motor.stop();
    buzzer_beep(50U);

    /* ===== 4. 主循环 ===== */
    uint32_t next_control_ms = timebase_millis();
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

        /* 遥测发送频率控制: 每帧 ~150 字节 @ 9600 baud ≈ 150ms,
         * 阻塞期间无控制，频率过高会导致小车失控。
         * 改为每 500ms 发一次，给控制循环留 350ms 连续运行窗口。 */
        {
            static uint32_t next_tlm_ms = 500;
            uint32_t now = timebase_millis();
            if ((int32_t)(now - next_tlm_ms) >= 0) {
                next_tlm_ms = now + 500;
                proto_send_telemetry();
            }
        }
        proto_poll_commands();

        /* ---- 直角弯状态机 ---- */
        uint8_t digital = g_line_sensor.get_digital();
        uint32_t now_ms = timebase_millis();
        turn_fsm_update(digital, line_found, now_ms);

        /* ---- 控制决策 ---- */
        if (!g_line_track_on || !g_motor_on) {
            /* 功能关闭 → 停车 */
            g_motor.stop();
            g_dbg_left_cmd  = 0;
            g_dbg_right_cmd = 0;
            buzzer_off();
        } else if (turn_fsm_is_straight()) {
            /* 正常循迹：位置式 PID 差速
             *
             * 符号链 (全链路一致):
             *   sensor error > 0 → 线偏右 → 需右转
             *   PID 内部: error_curr = target − input = 0 − (−sensor_error) = sensor_error
             *   位置式: u = Kp×e + Ki×∫e + Kd×(e[k]−e[k−1])
             *   误差越大输出越大，误差→0 输出→0，天然回中。
             *
             *   最终: left = base + correction, right = base - correction
             */

            /* 死区: 微小误差不驱动 PID, 减少电机微振
             *   修复: 用 >=/<= 替代 >/<, 使实际死区匹配常量名 (|error|<=5) */
            int32_t pid_input = error;
            if (pid_input >= -SENSOR_ERROR_DEADBAND &&
                pid_input <=  SENSOR_ERROR_DEADBAND) {
                pid_input = 0;
            }

            /* 传入 -error: 对齐 PID 输出与转向修正的符号 */
            int32_t correction = round_to_i32(
                g_pid.calc((float)(-pid_input)));

            /* correction 钳位 ±g_base_speed: 保证 raw_left/right 不超 PWM_MAX,
             *   从而 apply_diff_steering 永不触发等比例缩放,
             *   且内轮不会反转 (最低 0 = 停转, 已是极限差速)。 */
            if (correction >  (int32_t)g_base_speed) correction =  (int32_t)g_base_speed;
            if (correction < -(int32_t)g_base_speed) correction = -(int32_t)g_base_speed;

            int16_t left_cmd, right_cmd;
            apply_diff_steering(
                (int32_t)g_base_speed + correction,
                (int32_t)g_base_speed - correction,
                &left_cmd, &right_cmd,
                PWM_MAX);

            g_motor.set_speed(left_cmd, right_cmd);
            buzzer_off();

            g_dbg_left_cmd  = left_cmd;
            g_dbg_right_cmd = right_cmd;
        } else if (!turn_fsm_is_lost()) {
            /* 左/右直角弯：使用状态机输出的固定差速
             * 重置 PID 状态 — 转弯期间 PID 不被调用，旧状态已失效
             * 重新进入直线时 PID 从零开始，第一个 calc() 产生合理的首次响应 */
            g_pid.reset();
            int16_t left_cmd, right_cmd;
            turn_fsm_get_motor_commands(&left_cmd, &right_cmd);
            g_motor.set_speed(left_cmd, right_cmd);
            buzzer_off();

            g_dbg_left_cmd  = left_cmd;
            g_dbg_right_cmd = right_cmd;
        } else {
            /* LOST：丢线或转弯超时 → 停车 + 蜂鸣器 1 Hz 间歇报警 */
            g_pid.reset();
            g_motor.stop();
            g_dbg_left_cmd  = 0;
            g_dbg_right_cmd = 0;

            if ((now_ms / 500U) % 2U == 0U) {
                buzzer_on();
            } else {
                buzzer_off();
            }
        }

        /* ---- 真实毫秒时基控制周期；处理超时后不再追加整段延时。 ---- */
        next_control_ms += CONTROL_PERIOD_MS;
        uint32_t after_work_ms = timebase_millis();
        if ((int32_t)(next_control_ms - after_work_ms) > 0) {
            while ((int32_t)(next_control_ms - timebase_millis()) > 0) {
                __NOP();
            }
        } else {
            next_control_ms = after_work_ms;
        }
    }
}
