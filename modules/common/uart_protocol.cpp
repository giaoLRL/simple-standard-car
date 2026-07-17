#include "modules/common/uart_protocol.hpp"
#include "modules/common/uart_debug.hpp"
#include "modules/common/config.hpp"
#include "modules/common/timebase.hpp"
#include "modules/common/iinterfaces.hpp"
#include "modules/control/turn_state_machine.hpp"
#include "modules/pid/pid.hpp"
#include "ti_msp_dl_config.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

extern PID g_pid;
extern volatile int32_t  g_dbg_error;
extern volatile int16_t  g_dbg_left_cmd;
extern volatile int16_t  g_dbg_right_cmd;
extern volatile bool     g_dbg_line_found;
extern volatile uint8_t  g_dbg_digital;
extern volatile uint16_t g_dbg_normal[8];
extern volatile uint16_t g_dbg_analog[8];

static const char *const STATE_NAMES[] = {"直道", "转向延时", "左转", "右转", "丢线"};
static const uint8_t STATE_COUNT = 5;

#define HELLO_CAPS (0x0F)

static uint16_t tlm_seq = 0;
static bool hello_sent = false;

void proto_init(void)
{
    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    hello_sent = false;
    tlm_seq = 0;
    proto_send_hello();
}

void proto_send_hello(void)
{
    uart_debug_printf(
        "$HELLO,0,{"
        "\"dev\":\"LineFollower\","
        "\"mcu\":\"MSPM0G3507\","
        "\"fw\":\"1.0\","
        "\"sensors\":%u,"
        "\"motors\":2,"
        "\"encoders\":0,"
        "\"gyro\":false,"
        "\"caps\":%u,"
        "\"states\":[\"直道\",\"转向延时\",\"左转\",\"右转\",\"丢线\"],"
        "\"tlm\":["
        "\"seq\",\"tick\",\"state\",\"flags\",\"caps\","
        "\"digital\",\"pos\",\"err\","
        "\"leftPwm\",\"rightPwm\","
        "\"n0\",\"n1\",\"n2\",\"n3\",\"n4\",\"n5\",\"n6\",\"n7\","
        "\"a0\",\"a1\",\"a2\",\"a3\",\"a4\",\"a5\",\"a6\",\"a7\""
        "],"
        "\"params\":{"
        "\"kp\":%d,\"ki\":%d,\"kd\":%d,"
        "\"baseSpeed\":%u,\"turnOuter\":%u,\"turnInner\":%u,"
        "\"pwmMax\":%d,\"adcMax\":%u"
        "}"
        "}\n",
        (unsigned)SENSOR_COUNT,
        (unsigned)HELLO_CAPS,
        (int)(g_pid.kp * 1000), (int)(g_pid.ki * 1000), (int)(g_pid.kd * 1000),
        g_base_speed, g_turn_outer_speed, g_turn_inner_speed,
        (int)PWM_MAX, (unsigned)ADC_MAX_VALUE
    );
    hello_sent = true;
}

void proto_send_telemetry(void)
{
    if (!hello_sent) return;

    TurnState state = turn_fsm_get_state();
    uint8_t state_idx = (uint8_t)state;
    if (state_idx >= STATE_COUNT) state_idx = STATE_COUNT - 1;

    uint8_t flags = 0;
    if (!g_line_track_on) flags |= 2;
    if (!g_motor_on) flags |= 4;

    uart_debug_printf(
        "$T,3,%u,%lu,%u,%u,%u,"
        "0x%02X,"
        "%ld,%ld,"
        "%d,%d,"
        "%u,%u,%u,%u,%u,%u,%u,%u,"
        "%u,%u,%u,%u,%u,%u,%u,%u"
        "\n",
        tlm_seq++,
        (unsigned long)timebase_millis(),
        state_idx,
        flags,
        (unsigned)HELLO_CAPS,

        (unsigned)(g_dbg_digital & 0xFF),
        (long)g_dbg_error,
        (long)(g_dbg_error),

        (int)g_dbg_left_cmd,
        (int)g_dbg_right_cmd,

        (unsigned)g_dbg_normal[0], (unsigned)g_dbg_normal[1],
        (unsigned)g_dbg_normal[2], (unsigned)g_dbg_normal[3],
        (unsigned)g_dbg_normal[4], (unsigned)g_dbg_normal[5],
        (unsigned)g_dbg_normal[6], (unsigned)g_dbg_normal[7],

        (unsigned)g_dbg_analog[0], (unsigned)g_dbg_analog[1],
        (unsigned)g_dbg_analog[2], (unsigned)g_dbg_analog[3],
        (unsigned)g_dbg_analog[4], (unsigned)g_dbg_analog[5],
        (unsigned)g_dbg_analog[6], (unsigned)g_dbg_analog[7]
    );

    tlm_seq &= 0xFFFF;
}

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
        } else if (cmd_is(cmd, len, "GO")) {
            g_motor_on = true;
            g_line_track_on = true;
            g_pid.reset();
        } else if (cmd_is(cmd, len, "STP")) {
            g_motor_on = !g_motor_on;
        } else if (cmd_is(cmd, len, "RUN")) {
            g_line_track_on = !g_line_track_on;
        } else if (cmd_is(cmd, len, "CAL")) {
        } else if (cmd_is(cmd, len, "HELLO")) {
            proto_send_hello();
        } else if (cmd_is(cmd, len, "RST")) {
            g_pid.kp = 0.05f;
            g_pid.ki = 0.0000f;
            g_pid.kd = 0.13f;
            g_base_speed = 200;
            g_turn_outer_speed = 170;
            g_turn_inner_speed = 80;
            g_pid.reset();
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
