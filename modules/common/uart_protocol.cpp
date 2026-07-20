#include "modules/common/uart_protocol.hpp"
#include "modules/common/uart_debug.hpp"
#include "modules/common/config.hpp"
#include "modules/common/timebase.hpp"
#include "modules/common/iinterfaces.hpp"
#include "modules/control/turn_state_machine.hpp"
#include "modules/pid/pid.hpp"
#if ENABLE_ENCODER
#include "modules/encoder/encoder.hpp"
#include "modules/speed_pid/speed_pid.hpp"
#endif
#include "ti_msp_dl_config.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

extern PID g_pid;
extern volatile int32_t  g_dbg_error;
extern volatile int16_t  g_dbg_left_cmd;
extern volatile int16_t  g_dbg_right_cmd;
extern volatile int32_t  g_dbg_correction;
extern volatile bool     g_dbg_line_found;
extern volatile uint8_t  g_dbg_digital;
extern volatile uint16_t g_dbg_normal[8];
extern volatile uint16_t g_dbg_analog[8];

#if ENABLE_ENCODER
extern PID g_spid_left;
extern PID g_spid_right;
extern volatile float   g_dbg_left_rpm;
extern volatile float   g_dbg_right_rpm;
#endif

static const char *const STATE_NAMES[] = {"直道", "转向延时", "左转", "右转", "丢线"};
static const uint8_t STATE_COUNT = 5;

#define HELLO_CAPS (0x0F)

static uint16_t tlm_seq = 0;
static bool hello_sent = false;

void proto_init(void)
{
    hello_sent = false;
    tlm_seq = 0;
    /*
     * 修复 (2026-07-18): 原实现在 proto_init 中立即调用 proto_send_hello(),
     * 发送约 300 字节 JSON 到 UART。若 UART 未连接 (ESP32 未通电), TX 环
     * 形缓冲区 256 字节被塞满后, uart_debug_send_char 自旋等待 ISR 腾空位,
     * 但 ISR 不会触发——导致永久阻塞, 程序卡在初始化阶段, 蜂鸣器和电机都不工作。
     *
     * 现在: proto_init 只标记"已初始化", 不主动发送 HELLO。
     * HELLO 仅在小程序发送 "HELLO" 命令时响应 (此时 UART 必然已建立连接),
     * 或由外部主动调用 proto_send_hello() 时发送。
     * telemetry 在 hello_sent=false 时不会发送, 不影响正常运行。
     *
     * 副作用: 不用 ESP32/小程序也能正常启动和标定。需要远程时连上后
     * 小程序会自动发送 HELLO 命令触发握手。
     */
    /* proto_send_hello(); — 移除启动时阻塞发送 */
}

/* ============================================================
 *  proto_send_hello — 非阻塞 HELLO 响应
 *
 *  修复 (2026-07-20): 原实现调用 uart_debug_printf 阻塞发送约 300 字节,
 *  在主循环内被 proto_poll_commands→parse_and_apply 触发时阻塞主循环
 *  ~312ms @9600baud。此期间无传感器读取、无电机控制、无喂狗, 车会
 *  短暂失控。更严重的是, 调用链深度 (main→poll→parse→send_hello→
 *  printf→send→send_char) 加 ISR 嵌套栈用超过默认栈 512 字节, 导致
 *  HardFault (默认 handler 静默死循环→蜂鸣器停响)。
 *
 *  现在: 用静态缓冲 + vsnprintf 格式化 (不占栈), 再用非阻塞
 *  uart_debug_send_bytes_nb 一次性写入 TX 环。若 TX 环空间不足则
 *  静默丢弃, 小程序 15 秒超时后降级重试。hello_sent 仅在发送成功
 *  时置位, 确保遥测不会在 HELLO 协商完成前启动。
 * ============================================================ */
void proto_send_hello(void)
{
    static char buf[640];
    int len;

    len = snprintf(buf, sizeof(buf),
        "$HELLO,0,{"
        "\"dev\":\"LineFollower\","
        "\"mcu\":\"MSPM0G3507\","
        "\"fw\":\"2.0\","
        "\"sensors\":%u,"
        "\"motors\":2,"
        "\"encoders\":%u,"
        "\"gyro\":false,"
        "\"caps\":%u,"
        "\"bin\":true,"
        "\"states\":[\"直道\",\"转向延时\",\"左转\",\"右转\",\"丢线\"],"
        "\"tlm\":["
        "\"seq\",\"tick\",\"state\",\"flags\","
        "\"error\",\"leftPwm\",\"rightPwm\",\"digital\","
        "\"n0\",\"n1\",\"n2\",\"n3\",\"n4\",\"n5\",\"n6\",\"n7\","
        "\"a0\",\"a1\",\"a2\",\"a3\",\"a4\",\"a5\",\"a6\",\"a7\""
#if ENABLE_ENCODER
        ",\"leftRpm\",\"rightRpm\""
#endif
        "],"
        "\"params\":{"
        "\"kp\":%d,\"ki\":%d,\"kd\":%d,"
        "\"baseSpeed\":%u,\"turnOuter\":%u,\"turnInner\":%u,"
        "\"pwmMax\":%d,\"adcMax\":%u"
#if ENABLE_ENCODER
        ",\"speedKp\":%d,\"speedKi\":%d,\"speedKd\":%d"
#endif
        "}"
        "}\n",
        (unsigned)SENSOR_COUNT,
#if ENABLE_ENCODER
        (unsigned)2,
#else
        (unsigned)0,
#endif
        (unsigned)HELLO_CAPS,
        (int)(g_pid.kp * 1000), (int)(g_pid.ki * 1000), (int)(g_pid.kd * 1000),
        g_base_speed, g_turn_outer_speed, g_turn_inner_speed,
        (int)PWM_MAX, (unsigned)ADC_MAX_VALUE
#if ENABLE_ENCODER
        , (int)(g_speed_kp * 1000), (int)(g_speed_ki * 1000), (int)(g_speed_kd * 1000)
#endif
    );

    if (len > 0 && len < (int)sizeof(buf)) {
        if (uart_debug_send_bytes_nb((const uint8_t *)buf, len)) {
            hello_sent = true;
        }
        /* 发送失败 (TX 环空间不足): hello_sent 保持 false,
         * 遥测不启动。小程序 HELLO 超时 15 秒后降级重试,
         * 或小程序重新发送 HELLO 命令时会再次触发本函数。 */
    }
}

/* ============================================================
 *  proto_send_telemetry — 二进制紧凑帧发送
 *
 *  帧格式: [0xAA][0x55][type=0x01][len=sizeof(TlmPayload)][payload][checksum]
 *  总长: 4 + len + 1 字节
 *  ISR 后台发送不阻塞主循环 (缓冲区满时可能短暂自旋等待)
 * ============================================================ */
void proto_send_telemetry(void)
{
    if (!hello_sent) return;

    /* 防止 TX 缓冲区溢出：二进制遥测帧 56 字节，9600 baud
     * 每 10ms 只能发 ~9.6 字节。若按主循环 10ms 周期每次发 56 字节，
     * TX 环形缓冲区 (256B) 在 ~57ms 内塞满，之后永久阻塞主循环。
     * 此处保守检查：剩余空间 < 64 字节时跳过本帧。 */
    if (uart_debug_tx_space() < 64) return;

    TurnState state = turn_fsm_get_state();
    uint8_t state_idx = (uint8_t)state;
    if (state_idx >= STATE_COUNT) state_idx = STATE_COUNT - 1;

    uint8_t flags = 0;
    if (!g_line_track_on)  flags |= 2;
    if (!g_motor_on)       flags |= 4;
#if ENABLE_ENCODER
    if (!g_speed_loop_on)  flags |= 8;
    if (!g_dir_pid_on)     flags |= 16;
#endif

    TlmPayload payload;
    payload.seq       = tlm_seq++;
    payload.tick      = timebase_millis();
    payload.state     = state_idx;
    payload.flags     = flags;
    payload.error     = (int16_t)g_dbg_error;
    payload.left_pwm  = g_dbg_left_cmd;
    payload.right_pwm = g_dbg_right_cmd;
    payload.digital   = g_dbg_digital;
    for (int i = 0; i < 8; i++) {
        payload.normal[i] = g_dbg_normal[i];
        payload.analog[i] = g_dbg_analog[i];
    }
#if ENABLE_ENCODER
    payload.left_rpm  = (int16_t)g_dbg_left_rpm;
    payload.right_rpm = (int16_t)g_dbg_right_rpm;
#endif

    /* 组装帧: header(2) + type(1) + len(1) + payload + checksum(1) */
    uint8_t frame[64];
    uint8_t *p = frame;
    *p++ = 0xAA;
    *p++ = 0x55;
    *p++ = 0x01;                     /* type: telemetry */
    *p++ = (uint8_t)sizeof(TlmPayload); /* payload length */
    (void)memcpy(p, &payload, sizeof(TlmPayload));
    p += sizeof(TlmPayload);

    /* checksum = XOR(type, len, payload bytes) */
    uint8_t checksum = frame[2] ^ frame[3];
    for (int i = 4; i < (int)(p - frame); i++) {
        checksum ^= frame[i];
    }
    *p++ = checksum;

    /* 非阻塞发送：写入 TX 环形缓冲区，ISR 后台发出。
     * 空间不足直接丢弃本帧，绝不阻塞。帧率由上游 500ms 节流控制。 */
    if (!uart_debug_send_bytes_nb(frame, (int)(p - frame))) {
        /* TX 缓冲不足，静默丢弃本帧。接收端可通过 tlm_seq 跳号检测丢帧。 */
    }
}
/* ============================================================
 *  命令解析（文本协议，保持不变）
 * ============================================================ */

static int parse_value(const char *s)
{
    if (strchr(s, '.')) {
        float f = (float)atof(s);
        return (int)(f * 1000.0f);
    }
    return atoi(s);
}

static bool cmd_is(const char *cmd, int cmd_len, const char *expect)
{
    int elen = (int)strlen(expect);
    if (cmd_len != elen) return false;
    for (int i = 0; i < elen; i++) {
        char c = cmd[i];
        if (c >= 'a' && c <= 'z') c -= 32;
        if (c != expect[i]) return false;
    }
    return true;
}

static void parse_and_apply(const char *cmd, int len)
{
    const char *eq = (const char *)memchr(cmd, '=', (size_t)len);
    if (!eq) {
        if (cmd_is(cmd, len, "STOP")) {
            g_motor_on = false;
            g_pid.reset();
#if ENABLE_ENCODER
            g_spid_left.reset();
            g_spid_right.reset();
            g_enc_left.reset();
            g_enc_right.reset();
            g_spd_need_warmstart = true;
#endif
        } else if (cmd_is(cmd, len, "GO")) {
            g_motor_on = true;
            g_line_track_on = true;
            g_pid.reset();
#if ENABLE_ENCODER
            g_spid_left.reset();
            g_spid_right.reset();
            g_enc_left.reset();
            g_enc_right.reset();
            g_spd_need_warmstart = true;
#endif
        } else if (cmd_is(cmd, len, "STP")) {
            g_motor_on = !g_motor_on;
        } else if (cmd_is(cmd, len, "RUN")) {
            g_line_track_on = !g_line_track_on;
        } else if (cmd_is(cmd, len, "CAL")) {
        } else if (cmd_is(cmd, len, "HELLO")) {
            proto_send_hello();
        }
#if ENABLE_ENCODER
        else if (cmd_is(cmd, len, "SPD?")) {
            char rsp[80];
            int n = snprintf(rsp, sizeof(rsp),
                "L rpm=%d ticks=%ld per=%luus R rpm=%d ticks=%ld per=%luus\n",
                (int)g_enc_left.get_speed_rpm(), (long)g_enc_left.get_ticks(),
                (unsigned long)g_enc_left.pulse_period_us,
                (int)g_enc_right.get_speed_rpm(), (long)g_enc_right.get_ticks(),
                (unsigned long)g_enc_right.pulse_period_us);
            if (n > 0 && n < (int)sizeof(rsp)) uart_debug_send(rsp);
        } else if (cmd_is(cmd, len, "ENC?")) {
            char rsp[48];
            int n = snprintf(rsp, sizeof(rsp), "L%d R%d\n",
                (int)g_enc_left.get_ticks(), (int)g_enc_right.get_ticks());
            if (n > 0 && n < (int)sizeof(rsp)) uart_debug_send(rsp);
        }
#endif
        else if (cmd_is(cmd, len, "RST")) {
            g_pid.kp = 0.05f;
            g_pid.ki = 0.0000f;
            g_pid.kd = 0.13f;
            g_base_speed = 200;
            g_turn_outer_speed = 170;
            g_turn_inner_speed = 80;
            g_pid.reset();
#if ENABLE_ENCODER
            g_speed_kp = SPEED_KP_DEFAULT;
            g_speed_ki = SPEED_KI_DEFAULT;
            g_speed_kd = SPEED_KD_DEFAULT;
            g_speed_loop_on = true;
            g_dir_pid_on    = true;
            g_spid_left.kp  = g_speed_kp;
            g_spid_left.ki  = g_speed_ki;
            g_spid_left.kd  = g_speed_kd;
            g_spid_right.kp = g_speed_kp;
            g_spid_right.ki = g_speed_ki;
            g_spid_right.kd = g_speed_kd;
            g_spid_left.reset();
            g_spid_right.reset();
            g_spid_left.set_sum_error(0.0f);
            g_spid_right.set_sum_error(0.0f);
            g_enc_left_rev  = ENC_LEFT_REVERSED;
            g_enc_right_rev = ENC_RIGHT_REVERSED;
            g_enc_left.reversed_  = g_enc_left_rev;
            g_enc_right.reversed_ = g_enc_right_rev;
            g_spd_need_warmstart = true;
#endif
        }
        return;
    }

    int key_len = (int)(eq - cmd);
    const char *val_str = eq + 1;
    int val_int = parse_value(val_str);

    if (cmd_is(cmd, key_len, "KP")) {
        g_pid.kp = (float)val_int / 1000.0f;
    } else if (cmd_is(cmd, key_len, "KI")) {
        g_pid.ki = (float)val_int / 1000.0f;
        /* 改 Ki 后清零积分: 旧 Ki 下累积的 sum_error_ 若不清零,
         * 新 Ki 生效时会将其放大/缩小, 导致输出突变 (车猛冲或猛转)。
         * Ki=0 时 sum_error_ 仍在每帧累加, 改成非零时尤为危险。 */
        g_pid.set_sum_error(0.0f);
    } else if (cmd_is(cmd, key_len, "KD")) {
        g_pid.kd = (float)val_int / 1000.0f;
    } else if (cmd_is(cmd, key_len, "BSP")) {
        if (val_int >= 0 && val_int <= (int)PWM_MAX) g_base_speed = (uint16_t)val_int;
    } else if (cmd_is(cmd, key_len, "TOS")) {
        if (val_int >= 0 && val_int <= (int)PWM_MAX) g_turn_outer_speed = (uint16_t)val_int;
    } else if (cmd_is(cmd, key_len, "TIS")) {
        if (val_int >= 0 && val_int <= (int)PWM_MAX) g_turn_inner_speed = (uint16_t)val_int;
    } else if (cmd_is(cmd, key_len, "LTO")) {
        g_line_track_on = (val_int != 0);
    } else if (cmd_is(cmd, key_len, "MTO")) {
        g_motor_on = (val_int != 0);
    }
#if ENABLE_ENCODER
    else if (cmd_is(cmd, key_len, "SKP")) {
        g_speed_kp = (float)val_int / 1000.0f;
        g_spid_left.kp = g_speed_kp;
        g_spid_right.kp = g_speed_kp;
    } else if (cmd_is(cmd, key_len, "SKI")) {
        g_speed_ki = (float)val_int / 1000.0f;
        g_spid_left.ki = g_speed_ki;
        g_spid_right.ki = g_speed_ki;
        /* 改 Ki 后清零积分：防突变 */
        g_spid_left.set_sum_error(0.0f);
        g_spid_right.set_sum_error(0.0f);
    } else if (cmd_is(cmd, key_len, "SKD")) {
        g_speed_kd = (float)val_int / 1000.0f;
        g_spid_left.kd = g_speed_kd;
        g_spid_right.kd = g_speed_kd;
    } else if (cmd_is(cmd, key_len, "SEN")) {
        g_speed_loop_on = (val_int != 0);
    } else if (cmd_is(cmd, key_len, "DEN")) {
        g_dir_pid_on = (val_int != 0);
    } else if (cmd_is(cmd, key_len, "ENCDIR")) {
        /* ENCDIR=L0R1 → 左正常/右反转；同理 L1R0, L0R0, L1R1 */
        const char *p = val_str;
        while (*p) {
            bool left = false;
            if (*p == 'L' || *p == 'l') { left = true; p++; }
            else if (*p == 'R' || *p == 'r') { left = false; p++; }
            else { p++; continue; }
            /* 修复: p++ 后检查是否已到字符串末尾, 防止越界读取 */
            if (*p == '\0') break;
            int rev = (*p == '1') ? 1 : 0;
            (left ? g_enc_left_rev : g_enc_right_rev) = (rev != 0);
            (left ? g_enc_left.reversed_ : g_enc_right.reversed_) = (rev != 0);
            p++;
        }
        char rsp[40];
        int n = snprintf(rsp, sizeof(rsp), "ENCDIR L%d R%d\n",
            (int)g_enc_left_rev, (int)g_enc_right_rev);
        if (n > 0 && n < (int)sizeof(rsp)) uart_debug_send(rsp);
    }
#endif
}

void proto_poll_commands(void)
{
    char line_buf[64];
    while (uart_debug_readline(line_buf, sizeof(line_buf))) {
        int len = (int)strlen(line_buf);
        if (len > 0 && line_buf[len - 1] == '\r') line_buf[--len] = 0;
        if (len > 0) parse_and_apply(line_buf, len);
    }
}
