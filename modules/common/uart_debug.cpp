#include "modules/common/uart_debug.hpp"
#include "modules/common/buzzer.hpp"
#include "modules/common/timebase.hpp"
#include "ti_msp_dl_config.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#define RX_RING_SIZE 128
#define TX_RING_SIZE 256

/* ---- RX ring buffer (ISR writes, main loop reads) ---- */
static volatile char    rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

/* ---- TX ring buffer (main loop writes, ISR reads) ---- */
static volatile char    tx_ring[TX_RING_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;

/* ============================================================
 *  UART 中断服务程序
 *
 *  TX: 从 tx_ring 喂入 TX FIFO，缓冲区空时关闭 TX 中断
 *  RX: 从 RX FIFO 读入 rx_ring（唯一的写入者，无竞态）
 * ============================================================ */
extern "C" void UART_1_INST_IRQHandler(void)
{
    /* ---- TX: 非阻塞发送，从环形缓冲区取数据写入 TX FIFO ---- */
    if (DL_UART_Main_isTXFIFOEmpty(UART_1_INST)) {
        if (tx_head != tx_tail) {
            DL_UART_Main_transmitData(UART_1_INST, (uint8_t)tx_ring[tx_tail]);
            tx_tail = (tx_tail + 1) % TX_RING_SIZE;
        }
        if (tx_head == tx_tail) {
            /* 缓冲区已空，关闭 TX 中断避免空转 */
            DL_UART_Main_disableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
        }
    }

    /* ---- RX: ISR 是 rx_ring 的唯一写入者，无竞态 ---- */
    if (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
            uint8_t next = (rx_head + 1) % RX_RING_SIZE;
            if (next != rx_tail) {
                rx_ring[rx_head] = (char)DL_UART_Main_receiveData(UART_1_INST);
                rx_head = next;
            } else {
                /* Ring buffer 满，丢弃 */
                DL_UART_Main_receiveData(UART_1_INST);
            }
        }
    }
}

void uart_debug_init(uint32_t baud_rate)
{
    /* 波特率由 SYSCFG_DL_UART_1_init() 已正确设置 (9600)，
     * 此处只负责使能 RX 中断。不再调用 setBaudRateDivisor，
     * 避免 uart_debug_init 的除数公式 (ibrd=divisor>>6) 与
     * syscfg 生成的正确除数 (IBRD=208, FBRD=21) 冲突。 */
    (void)baud_rate;  /* 保留参数兼容性，实际不再使用 */

    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    /* TX 中断初始关闭，有数据要发时才打开 */
}

/* ============================================================
 *  uart_debug_send_char — 写入 TX 环形缓冲区并触发 TX 中断
 *
 *  修复 (2026-07-18): 原实现在 TX 缓冲满时无限自旋等待,
 *  若 TX 中断因配置/硬件原因未触发 (如 ESP32 桥未通电),
 *  会永久死锁——proto_init() 里的 proto_send_hello() 发送
 *  约 300 字节 HELLO JSON, 超过 TX ring 256 字节容量,
 *  必然塞满后死等, 导致程序卡在初始化阶段, 蜂鸣器和电机
 *  都不工作。
 *
 *  现在: 自旋等待最多 5 秒, 期间蜂鸣器 1Hz 慢速响报警,
 *  超时后丢弃本字节继续执行, 避免永久死锁。
 * ============================================================ */
#define TX_WAIT_TIMEOUT_MS   (5000U)  /* TX 缓冲满等待超时: 5 秒 */
#define TX_WAIT_BEEP_PERIOD  (500U)   /* 报警蜂鸣器周期: 500ms 响 / 500ms 停 */

void uart_debug_send_char(char c)
{
    uint8_t next;
    uint32_t start_ms = timebase_millis();
    bool waited = false;

    do {
        __disable_irq();
        next = (tx_head + 1) % TX_RING_SIZE;
        __enable_irq();

        if (next == tx_tail) {
            /* TX 缓冲满, 进入等待 */
            waited = true;
            uint32_t elapsed = timebase_millis() - start_ms;

            if (elapsed >= TX_WAIT_TIMEOUT_MS) {
                /* 超时 5 秒: TX 中断可能未触发 (ESP32 未通电等),
                 * 丢弃本字节避免永久死锁, 程序可继续往下执行 */
                buzzer_off();
                return;
            }

            /* 等待期间蜂鸣器 1Hz 慢速响, 提示 TX 缓冲满异常 */
            if ((elapsed / TX_WAIT_BEEP_PERIOD) % 2U == 0U) {
                buzzer_on();
            } else {
                buzzer_off();
            }
        }
    } while (next == tx_tail);

    /* 写入成功, 若曾等待则关闭报警蜂鸣器 */
    if (waited) {
        buzzer_off();
    }

    __disable_irq();
    tx_ring[tx_head] = c;
    tx_head = next;
    __enable_irq();

    /* 触发 TX 中断：若 FIFO 已空则立即进入 ISR 发送 */
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX);
}

/* ---- 批量发送（缓冲区满时可能短暂自旋）---- */
void uart_debug_send_bytes(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        uart_debug_send_char((char)data[i]);
    }
}

void uart_debug_send(const char *str)
{
    while (*str) {
        uart_debug_send_char(*str++);
    }
}

void uart_debug_printf(const char *format, ...)
{
    char buf[640];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len < 0) return;

    for (int i = 0; i < len && i < (int)sizeof(buf) - 1; i++) {
        uart_debug_send_char(buf[i]);
    }
}

/* ============================================================
 *  uart_debug_readline — 从 RX 环形缓冲区读取一行
 *
 *  ISR 是 rx_ring 唯一的写入者（不再 drain RX FIFO）。
 *
 *  修复 (2026-07-18): 原实现在未读到 '\n' 时也会前移 rx_tail，
 *  导致半截命令被丢弃。9600 baud 下一条命令 "KP=0.05\n" 需要
 *  约 9.4ms 才能传完，而主循环 10ms 周期会调用本函数——
 *  命令几乎必然在传输中途被读取并丢弃。
 *  现在：只在找到 '\n' 后才更新 rx_tail，未完整的行保留在
 *  缓冲区里等下次再读。
 *
 *  溢出保护: 若已累积字节超过 buf 容量（异常长行/无 '\n'），
 *  丢弃这些字节防止永久阻塞。
 * ============================================================ */
int uart_debug_readline(char *buf, int max_len)
{
    if (max_len <= 0) return 0;

    /* 快照 rx_head（ISR 是 rx_ring 唯一写入者），避免竞态 */
    uint8_t head;
    __disable_irq();
    head = rx_head;
    __enable_irq();

    /* 扫描 rx_tail..head，查找 '\n'。
     * 不在找到 '\n' 前修改 rx_tail —— 这是修复的核心。 */
    uint8_t scan = rx_tail;
    int newline_found = 0;
    int scanned_bytes = 0;

    while (scan != head) {
        char c = rx_ring[scan];
        if (c == '\n') {
            newline_found = 1;
            break;
        }
        scan = (uint8_t)((scan + 1) % RX_RING_SIZE);
        scanned_bytes++;
    }

    if (!newline_found) {
        /* 行未完整到达。
         *   - 已累积字节未超过 buf 容量 → 保留在缓冲区，等下次再读
         *   - 已超过 buf 容量（异常长行）→ 丢弃，防止永久阻塞 */
        if (scanned_bytes < max_len - 1) {
            return 0;  /* 关键：不更新 rx_tail */
        }
        __disable_irq();
        rx_tail = scan;
        __enable_irq();
        buf[0] = '\0';
        return 0;
    }

    /* 找到 '\n'，scan 指向 '\n' 本身的位置。
     * 复制 rx_tail..scan 之间的数据到 buf（跳过 '\r'）。 */
    int written = 0;
    uint8_t read_pos = rx_tail;
    while (read_pos != scan) {
        char c = rx_ring[read_pos];
        read_pos = (uint8_t)((read_pos + 1) % RX_RING_SIZE);
        if (c == '\r') continue;
        if (written < max_len - 1) {
            buf[written++] = c;
        }
        /* 超出 buf 容量的部分丢弃，但 rx_tail 仍前移到 '\n' 之后 */
    }

    /* 更新 rx_tail：跳过 '\n' 本身 */
    __disable_irq();
    rx_tail = (uint8_t)((scan + 1) % RX_RING_SIZE);
    __enable_irq();

    buf[written] = '\0';
    return 1;
}
