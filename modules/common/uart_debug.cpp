#include "modules/common/uart_debug.hpp"
#include "ti_msp_dl_config.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#define RX_RING_SIZE 128

static volatile char    rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static const uint8_t *g_tx_data      = nullptr;
static int            g_tx_remaining = 0;

extern "C" void UART_1_INST_IRQHandler(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST)) {
        uint8_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = (char)DL_UART_Main_receiveData(UART_1_INST);
            rx_head = next;
        } else {
            DL_UART_Main_receiveData(UART_1_INST);
        }
    }
}
void uart_debug_init(uint32_t baud_rate)
{
    (void)baud_rate;
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    g_tx_data      = nullptr;
    g_tx_remaining = 0;
}

void uart_debug_continue_tx(void)
{
    while (g_tx_data && g_tx_remaining > 0) {
        if (DL_UART_Main_isTXFIFOFull(UART_1_INST)) return;
        DL_UART_Main_transmitData(UART_1_INST, *g_tx_data);
        g_tx_data++;
        g_tx_remaining--;
    }
    g_tx_data      = nullptr;
    g_tx_remaining = 0;
}
static void start_nonblocking_tx(const uint8_t *data, int len)
{
    if (len <= 0) return;
    while (g_tx_data && g_tx_remaining > 0) {
        if (DL_UART_Main_isTXFIFOFull(UART_1_INST)) break;
        DL_UART_Main_transmitData(UART_1_INST, *g_tx_data);
        g_tx_data++;
        g_tx_remaining--;
    }
    if (g_tx_data && g_tx_remaining == 0) g_tx_data = nullptr;
    if (g_tx_data != nullptr) return;
    while (len > 0) {
        if (DL_UART_Main_isTXFIFOFull(UART_1_INST)) {
            g_tx_data = data;
            g_tx_remaining = len;
            return;
        }
        DL_UART_Main_transmitData(UART_1_INST, *data);
        data++;
        len--;
    }
}
void uart_debug_send_char(char c)
{
    uint8_t b = (uint8_t)c;
    start_nonblocking_tx(&b, 1);
}

void uart_debug_send(const char *str)
{
    start_nonblocking_tx((const uint8_t *)str, (int)strlen(str));
}

void uart_debug_send_bytes(const uint8_t *data, int len)
{
    start_nonblocking_tx(data, len);
}

bool uart_debug_send_bytes_nb(const uint8_t *data, int len)
{
    if (len <= 0) return true;
    if (g_tx_data != nullptr) return false;
    start_nonblocking_tx(data, len);
    return true;
}
void uart_debug_printf(const char *format, ...)
{
    uart_debug_continue_tx();
    static char buf[640];
    int         len;
    va_list     args;
    va_start(args, format);
    len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;
        for (int i = 0; i < len; i++) {
            while (DL_UART_Main_isTXFIFOFull(UART_1_INST)) __NOP();
            DL_UART_Main_transmitData(UART_1_INST, (uint8_t)buf[i]);
        }
    }
}
int uart_debug_tx_space(void)
{
    if (g_tx_data != nullptr) return 0;
    return 4;
}

int uart_debug_readline(char *buf, int max_len)
{
    if (max_len <= 0) return 0;
    uint8_t head;
    __disable_irq();
    head = rx_head;
    __enable_irq();
    uint8_t scan = rx_tail;
    int newline_found = 0;
    int scanned_bytes = 0;
    while (scan != head) { char c = rx_ring[scan]; if (c == '\n') { newline_found = 1; break; } scan = (uint8_t)((scan + 1) % RX_RING_SIZE); scanned_bytes++; }
    if (!newline_found) {
        if (scanned_bytes < max_len - 1) return 0;
        __disable_irq();
        rx_tail = scan;
        __enable_irq();
        buf[0] = '\0';
        return 0;
    }
    int written = 0;
    uint8_t read_pos = rx_tail;
    while (read_pos != scan) { char c = rx_ring[read_pos]; read_pos = (uint8_t)((read_pos + 1) % RX_RING_SIZE); if (c == '\r') continue; if (written < max_len - 1) buf[written++] = c; }
    __disable_irq();
    rx_tail = (uint8_t)((scan + 1) % RX_RING_SIZE);
    __enable_irq();
    buf[written] = '\0';
    return 1;
}