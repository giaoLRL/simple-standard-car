#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_debug_init(uint32_t baud_rate);
void uart_debug_send_char(char c);
void uart_debug_send(const char *str);
void uart_debug_send_bytes(const uint8_t *data, int len);
void uart_debug_printf(const char *format, ...);
void uart_debug_continue_tx(void);

int  uart_debug_readline(char *buf, int max_len);
int  uart_debug_tx_space(void);
bool uart_debug_send_bytes_nb(const uint8_t *data, int len);

#ifdef __cplusplus
}
#endif
