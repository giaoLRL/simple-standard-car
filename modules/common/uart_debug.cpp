#include "modules/common/uart_debug.hpp"
#include "ti_msp_dl_config.h"

#include <cstdarg>
#include <cstdio>

void uart_debug_init(uint32_t baud_rate)
{
    uint32_t divisor = (CPUCLK_FREQ + (baud_rate * 8U)) / (baud_rate * 16U);
    uint32_t ibrd = divisor >> 6;
    uint32_t fbrd = divisor & 0x3F;
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, ibrd, fbrd);
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
    char buf[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len < 0) return;

    for (int i = 0; i < len && i < (int)sizeof(buf) - 1; i++) {
        uart_debug_send_char(buf[i]);
    }
}
