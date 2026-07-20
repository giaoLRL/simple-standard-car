/*
 * main.cpp — 循迹小车入口点 + 依赖组装 + 主循环
 *
 * 功能: 8 路红外传感器循迹，位置式 PID 驱动双电机差速转向。
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
 *   1. 位置式 PID 控制（无衰减、无 D 项 LPF）；定点化待迁移
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
#include "modules/common/uart_debug.hpp"
#if ENABLE_ENCODER
#include "modules/encoder/encoder.hpp"
#include "modules/speed_pid/speed_pid.hpp"
#endif

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
 *     Kp=0.05 在 error=±3500 时 P 项输出 ≈ ±175。 */
PID g_pid(PID::position_type, 0.05f, 0.0000f, 0.13f,
          100000.0f, -100000.0f,  /* sum_error 限幅, 防积分饱和 */
          (float)PWM_MAX, -(float)PWM_MAX,
          NAN,    /* deriv_lpf_alpha — 位置式不使用 */
          NAN,    /* output_decay — 位置式不使用 */
          NAN);   /* max_delta — 位置式不使用 */

#if ENABLE_ENCODER
/* 速度环 PID — 增量式，左右轮各独立
 *
 *   速度环跟踪方向环输出的目标速度，输出 PWM 驱动电机。
 *   增量式特性：输出平滑，切换目标速度时无跳变；
 *   内置 output_decay=0.999 防止积分饱和。 */
PID g_spid_left(PID::delta_type, SPEED_KP_DEFAULT, SPEED_KI_DEFAULT, SPEED_KD_DEFAULT,
                NAN, NAN,                                /* sum_error 限幅 — 增量式不使用 */
                SPEED_OUT_LIMIT, -SPEED_OUT_LIMIT,       /* 输出限幅 ±PWM_MAX */
                0.5f,    /* deriv_lpf_alpha */
                0.999f,  /* output_decay */
                SPEED_MAX_DELTA);                        /* 单步增量限幅 */

PID g_spid_right(PID::delta_type, SPEED_KP_DEFAULT, SPEED_KI_DEFAULT, SPEED_KD_DEFAULT,
                 NAN, NAN,
                 SPEED_OUT_LIMIT, -SPEED_OUT_LIMIT,
                 0.5f, 0.999f, SPEED_MAX_DELTA);
#endif

/* ============================================================
 *  运行时特性开关
 * ============================================================ */

bool g_line_track_on = true;
bool g_motor_on      = true;

/* ============================================================
 *  调试变量（供 CCS Expressions / J-Scope 实时观察）
 * ============================================================ */

volatile int32_t  g_dbg_error       = 0;      /* 线偏差 */
volatile int16_t  g_dbg_left_cmd    = 0;      /* 左轮最终 PWM 指令 */
volatile int16_t  g_dbg_right_cmd   = 0;      /* 右轮最终 PWM 指令 */
volatile int32_t  g_dbg_correction  = 0;      /* 方向 PID 修正量 */
volatile bool     g_dbg_line_found  = false;  /* 是否检测到线 */
volatile uint8_t  g_dbg_digital     = 0;      /* 数字量（8 通道二值化）*/
volatile uint16_t g_dbg_normal[SENSOR_COUNT]; /* 归一化值 */
volatile uint16_t g_dbg_analog[SENSOR_COUNT]; /* 原始模拟量 */
volatile uint32_t g_dbg_loop_cnt    = 0;      /* 主循环迭代计数（心跳）*/

#if ENABLE_ENCODER
volatile float   g_dbg_left_rpm  = 0.0f;   /* 左轮实际转速 RPM */
volatile float   g_dbg_right_rpm = 0.0f;   /* 右轮实际转速 RPM */
volatile int16_t g_dbg_left_target  = 0;   /* 速度环目标左轮 (PWM 量纲) */
volatile int16_t g_dbg_right_target = 0;   /* 速度环目标右轮 (PWM 量纲) */
volatile float   g_dbg_spd_err_l = 0.0f;   /* 速度环左轮误差 (RPM) */
volatile float   g_dbg_spd_err_r = 0.0f;   /* 速度环右轮误差 (RPM) */
volatile float   g_dbg_spd_out_l = 0.0f;   /* 速度环左轮输出 (累积) */
volatile float   g_dbg_spd_out_r = 0.0f;   /* 速度环右轮输出 (累积) */
#endif

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

#if ENABLE_ENCODER
/*
 * 速度环微调：方向环输出为基准 PWM，速度环在此基础上 ± 微调。
 * 不替换方向环差速——保留左右轮差速，只调整体幅值。
 */
static void speed_loop_trim(int16_t *left_cmd, int16_t *right_cmd)
{
    if (!g_speed_loop_on) return;

    /* 转弯/丢线后首次进入直线 → 重置速度 PID，防旧状态跳变 */
    if (g_spd_need_warmstart) {
        g_spid_left.reset();
        g_spid_right.reset();
        g_spd_need_warmstart = false;
    }

    int16_t base_l = *left_cmd;
    int16_t base_r = *right_cmd;
    int16_t sign_l = (base_l >= 0) ? 1 : -1;
    int16_t sign_r = (base_r >= 0) ? 1 : -1;

    float target_l = (float)(sign_l * base_l) * 400.0f / (float)PWM_MAX;
    float target_r = (float)(sign_r * base_r) * 400.0f / (float)PWM_MAX;

    g_dbg_left_target  = base_l;
    g_dbg_right_target = base_r;

    float actual_l = g_enc_left.get_speed_rpm();
    float actual_r = g_enc_right.get_speed_rpm();
    if (actual_l < 0.0f) actual_l = -actual_l;
    if (actual_r < 0.0f) actual_r = -actual_r;

    float err_l = actual_l - target_l;
    float err_r = actual_r - target_r;
    g_dbg_spd_err_l = err_l;
    g_dbg_spd_err_r = err_r;

    int16_t trim_l = round_to_i32(g_spid_left.calc(err_l));
    int16_t trim_r = round_to_i32(g_spid_right.calc(err_r));

    int16_t max_l = (base_l >= 0 ? base_l : -base_l) / 2;
    int16_t max_r = (base_r >= 0 ? base_r : -base_r) / 2;
    if (trim_l >  max_l) trim_l =  max_l;
    if (trim_l < -max_l) trim_l = -max_l;
    if (trim_r >  max_r) trim_r =  max_r;
    if (trim_r < -max_r) trim_r = -max_r;

    *left_cmd  = base_l + trim_l;
    *right_cmd = base_r + trim_r;
    if (*left_cmd  >  PWM_MAX) *left_cmd  =  PWM_MAX;
    if (*left_cmd  < -PWM_MAX) *left_cmd  = -PWM_MAX;
    if (*right_cmd >  PWM_MAX) *right_cmd =  PWM_MAX;
    if (*right_cmd < -PWM_MAX) *right_cmd = -PWM_MAX;

    g_dbg_spd_out_l = (float)trim_l;
    g_dbg_spd_out_r = (float)trim_r;
}
#endif

/*
 * 传感器误差死区: |error| < 此值时 PID 输入置零,
 * 减少直线段电机微振和功耗。单位为传感器位置量纲 (千分位)。
 */
static constexpr int32_t SENSOR_ERROR_DEADBAND = 5;  /* |error|<=5 置零, 减少直线微振; 修复合入不等式 */

/* ============================================================
 *  HardFault 异常处理 — 快速蜂鸣 3 长 3 短 后死循环
 *  若发送参数后听到此报警 = 硬件异常崩溃
 * ============================================================ */
extern "C" void HardFault_Handler(void)
{
    while (1) {
        /* 3 声长响 (200ms on, 200ms off) */
        for (int i = 0; i < 3; i++) {
            DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
            for (volatile uint32_t d = 0; d < (CPUCLK_FREQ / 1000U * 200U); d++) { __NOP(); }
            DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
            for (volatile uint32_t d = 0; d < (CPUCLK_FREQ / 1000U * 200U); d++) { __NOP(); }
        }
        /* 3 声短响 (80ms on, 120ms off) */
        for (int i = 0; i < 3; i++) {
            DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
            for (volatile uint32_t d = 0; d < (CPUCLK_FREQ / 1000U * 80U); d++) { __NOP(); }
            DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
            for (volatile uint32_t d = 0; d < (CPUCLK_FREQ / 1000U * 120U); d++) { __NOP(); }
        }
        /* 停顿 1.5 秒后重复 */
        for (volatile uint32_t d = 0; d < (CPUCLK_FREQ / 1000U * 1500U); d++) { __NOP(); }
    }
}

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
    uart_debug_init(9600);  /* 必须调用：配置 UART1 波特率 + 使能 RX 中断 */
    proto_init();

#if ENABLE_ENCODER
    /* 编码器初始化（GPIO 中断已在 Encoder::init 内使能）*/
    g_enc_left.init();
    g_enc_right.init();

    /* 编码器方向初始值（来自 config.hpp 编译期配置）*/
    g_enc_left.reversed_  = g_enc_left_rev;
    g_enc_right.reversed_ = g_enc_right_rev;

    /* 中断优先级：按方案 3.4 分配
     *   GROUP1（编码器）已在 Encoder::init 中设为 0（最高）
     *   DMA_INT → 1, UART1 → 2 */
    NVIC_SetPriority(DMA_INT_IRQn, 1);
    NVIC_SetPriority(UART1_INT_IRQn, 2);
#endif

    /* 启动短响：确认当前固件已运行，随后进入 PB21 白底标定等待。 */
    buzzer_beep(80U);

    /* 在标定前主动发 HELLO：小程序连接后只有 3 秒超时窗口，
     * 若等到标定完成（需按键两次）才发，大概率超时降级为 V2。
     * UART 已配好，提前发送确保小程序在 3 秒内收到握手。 */
    proto_send_hello();

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
     *   - 原地转 → 某一轮方向反了, 单独置位该轮的 REVERSED 宏
     *
     * 同时检查编码器：如果 500ms 后 pulse_count 仍为 0，
     * 说明编码器中断未触发（接线/供电/引脚配置问题）。 */
    delay_cycles(800U * (CPUCLK_FREQ / 1000U));
#if ENABLE_ENCODER
    int32_t enc_ticks_before_l = g_enc_left.get_ticks();
    int32_t enc_ticks_before_r = g_enc_right.get_ticks();
#endif
    g_motor.set_speed(300, 300);
    {
        uint32_t t0 = timebase_millis();
        while ((int32_t)(timebase_millis() - t0) < 500) { __NOP(); }
    }
    g_motor.stop();
#if ENABLE_ENCODER
    int32_t enc_ticks_after_l = g_enc_left.get_ticks();
    int32_t enc_ticks_after_r = g_enc_right.get_ticks();
    int32_t enc_delta_l = enc_ticks_after_l - enc_ticks_before_l;
    int32_t enc_delta_r = enc_ticks_after_r - enc_ticks_before_r;
    /* 编码器自检：500ms 正转至少应有若干脉冲。若无，长响 3 声报警。 */
    if (enc_delta_l == 0 || enc_delta_r == 0) {
        for (int beep = 0; beep < 3; beep++) {
            buzzer_beep(500U);
            delay_cycles(300U * (CPUCLK_FREQ / 1000U));
        }
    }
#endif
    buzzer_beep(50U);

    /* 标定/自检完成后主动发 HELLO：小程序在标定期间发的 HELLO
     * 因 calibrate() 阻塞未能处理，此处补发确保小程序识别 MCU 已就绪。 */
    proto_send_hello();


    /* ===== 4. 主循环 ===== */
    uint32_t next_control_ms = timebase_millis();
    while (1) {
        g_dbg_loop_cnt++;  /* 心跳：每次主循环迭代 +1 */

        /* ---- 心跳诊断：开启速度环时每100次主循环短响 ---- */
#if ENABLE_ENCODER
        if (g_speed_loop_on && (g_dbg_loop_cnt % 100U) == 0U) {
            buzzer_on();
            for (volatile uint32_t bz = 0; bz < (CPUCLK_FREQ / 1000U * 5U); bz++) { __NOP(); }
            buzzer_off();
        }
#endif

        /* ---- 传感器任务（参照 No_Mcu_Ganv_Sensor_Task_Without_tick）---- */
        bool line_found = g_line_sensor.update();
        int32_t error   = g_line_sensor.get_error();

        /* 获取三种数据（供调试器观察）*/
        g_dbg_digital = g_line_sensor.get_digital();
        g_line_sensor.get_normalized((uint16_t *)g_dbg_normal);
        g_line_sensor.get_analog((uint16_t *)g_dbg_analog);

        g_dbg_error      = error;
        g_dbg_line_found = line_found;

#if ENABLE_ENCODER
        /* 编码器超时检测 + RPM 更新（实际脉冲由 GPIO ISR 异步处理）*/
        g_enc_left.update();
        g_enc_right.update();

        g_dbg_left_rpm  = g_enc_left.get_speed_rpm();
        g_dbg_right_rpm = g_enc_right.get_speed_rpm();
#endif

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
        uart_debug_continue_tx();
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

            /* 死区: 微小误差不驱动 PID, 减少电机微振 */
            int32_t pid_input = error;
            if (pid_input >= -SENSOR_ERROR_DEADBAND &&
                pid_input <=  SENSOR_ERROR_DEADBAND) {
                pid_input = 0;
            }

            /* 传入 -error: 对齐 PID 输出与转向修正的符号 */
            int32_t correction = round_to_i32(
                g_pid.calc((float)(-pid_input)));
            g_dbg_correction = correction;

#if ENABLE_ENCODER
            /* DEN=0 → 方向环 bypass，correction 强制归零 */
            if (!g_dir_pid_on) {
                correction = 0;
            }
#endif

            /* correction 钳位 ±g_base_speed */
            if (correction >  (int32_t)g_base_speed) correction =  (int32_t)g_base_speed;
            if (correction < -(int32_t)g_base_speed) correction = -(int32_t)g_base_speed;

            int32_t raw_left  = (int32_t)g_base_speed + correction;
            int32_t raw_right = (int32_t)g_base_speed - correction;

            /* 等比例饱和：保持转向曲率 */
            int16_t left_cmd, right_cmd;
            apply_diff_steering(raw_left, raw_right, &left_cmd, &right_cmd, PWM_MAX);

#if ENABLE_ENCODER
            speed_loop_trim(&left_cmd, &right_cmd);
#endif

            g_motor.set_speed(left_cmd, right_cmd);
            buzzer_off();

            g_dbg_left_cmd  = left_cmd;
            g_dbg_right_cmd = right_cmd;
        } else if (!turn_fsm_is_lost()) {
            /* 左/右直角弯：使用状态机输出的固定差速
             * 重置 PID 状态 — 转弯期间 PID 不被调用，旧状态已失效
             * 重新进入直线时 PID 从零开始，第一个 calc() 产生合理的首次响应 */
            g_pid.reset();
#if ENABLE_ENCODER
            g_spid_left.reset();
            g_spid_right.reset();
            g_spd_need_warmstart = true;
#endif
            int16_t left_cmd, right_cmd;
            turn_fsm_get_motor_commands(&left_cmd, &right_cmd);

#if ENABLE_ENCODER
            speed_loop_trim(&left_cmd, &right_cmd);
#endif

            g_motor.set_speed(left_cmd, right_cmd);
            buzzer_off();

            g_dbg_left_cmd  = left_cmd;
            g_dbg_right_cmd = right_cmd;
        } else {
            /* LOST：丢线或转弯超时 → 停车 + 蜂鸣器 1 Hz 间歇报警 */
            g_pid.reset();
#if ENABLE_ENCODER
            g_spid_left.reset();
            g_spid_right.reset();
            g_spd_need_warmstart = true;
#endif
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
