#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void proto_init(void);
void proto_send_hello(void);
void proto_send_telemetry(void);
void proto_poll_commands(void);

#ifdef __cplusplus
}
#endif
