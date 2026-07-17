#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_debug_init(uint32_t baud_rate);
void uart_debug_send_char(char c);
void uart_debug_send(const char *str);
void uart_debug_printf(const char *format, ...);

int  uart_debug_readline(char *buf, int max_len);

#ifdef __cplusplus
}
#endif
