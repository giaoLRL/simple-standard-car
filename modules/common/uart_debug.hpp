#pragma once

#include <stdint.h>

void uart_debug_init(uint32_t baud_rate);
void uart_debug_send_char(char c);
void uart_debug_send(const char *str);
void uart_debug_printf(const char *format, ...);
