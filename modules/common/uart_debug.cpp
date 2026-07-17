#include "modules/common/uart_debug.hpp"
#include "ti_msp_dl_config.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#define RX_RING_SIZE 128

static volatile char rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

extern "C" void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_Main_isTXFIFOEmpty(UART_0_INST)) {
        DL_UART_Main_disableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
    if (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
            uint8_t next = (rx_head + 1) % RX_RING_SIZE;
            if (next != rx_tail) {
                rx_ring[rx_head] = DL_UART_Main_receiveData(UART_0_INST);
                rx_head = next;
            } else {
                DL_UART_Main_receiveData(UART_0_INST);
            }
        }
    }
}

void uart_debug_init(uint32_t baud_rate)
{
    uint32_t divisor = (CPUCLK_FREQ + (baud_rate * 8U)) / (baud_rate * 16U);
    uint32_t ibrd = divisor >> 6;
    uint32_t fbrd = divisor & 0x3F;
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, ibrd, fbrd);

    DL_UART_Main_enableInterrupt(UART_0_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void uart_debug_send_char(char c)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)c);
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

int uart_debug_readline(char *buf, int max_len)
{
    if (max_len <= 0) return 0;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = DL_UART_Main_receiveData(UART_0_INST);
            rx_head = next;
        } else {
            DL_UART_Main_receiveData(UART_0_INST);
        }
    }

    int written = 0;
    uint8_t tail_copy;
    __disable_irq();
    tail_copy = rx_tail;
    __enable_irq();

    while (tail_copy != rx_head && written < max_len - 1) {
        char c = rx_ring[tail_copy];
        tail_copy = (tail_copy + 1) % RX_RING_SIZE;

        if (c == '\r') continue;
        if (c == '\n') {
            __disable_irq();
            rx_tail = tail_copy;
            __enable_irq();
            buf[written] = '\0';
            return 1;
        }

        buf[written++] = c;
    }

    __disable_irq();
    rx_tail = tail_copy;
    __enable_irq();

    buf[written] = '\0';
    return 0;
}
