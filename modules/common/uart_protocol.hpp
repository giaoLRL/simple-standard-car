#pragma once

#include <stdint.h>
#include "modules/common/config.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void proto_init(void);
void proto_send_hello(void);
void proto_send_telemetry(void);
void proto_poll_commands(void);

/* ============================================================
 *  二进制遥测帧载荷（紧凑打包，52 字节整帧）
 *
 *  帧格式: [0xAA][0x55][type:1][len:1][payload:len][checksum:1]
 *    type=0x01 → 遥测数据
 *    len=sizeof(TlmPayload)，由结构体大小自动确定
 *    checksum = XOR(type, len, payload[0..len-1])
 * ============================================================ */
#pragma pack(push, 1)
typedef struct {
    uint16_t seq;          /* 帧序号 */
    uint32_t tick;         /* 毫秒时间戳 */
    uint8_t  state;        /* 状态索引 */
    uint8_t  flags;        /* bit1(0x02)=循迹关, bit2(0x04)=电机关,
                               bit3(0x08)=速度环关, bit4(0x10)=方向环关 */
    int16_t  error;        /* PID 误差 */
    int16_t  left_pwm;     /* 左轮 PWM 指令 */
    int16_t  right_pwm;    /* 右轮 PWM 指令 */
    uint8_t  digital;      /* 8 通道二值化位图 */
    uint16_t normal[8];    /* 归一化值 n0~n7 */
    uint16_t analog[8];    /* 原始模拟量 a0~a7 */
#if ENABLE_ENCODER
    int16_t  left_rpm;     /* 左轮实际转速 RPM（比例 1:1）*/
    int16_t  right_rpm;    /* 右轮实际转速 RPM（比例 1:1）*/
#endif
} TlmPayload;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif
