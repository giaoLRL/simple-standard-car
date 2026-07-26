#pragma once

#include <stdint.h>

/* 有源蜂鸣器控制接口，高电平响 */

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint32_t ms);   /* 阻塞式鸣叫 ms 毫秒 */
