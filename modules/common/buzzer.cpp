#include "modules/common/buzzer.hpp"
#include "modules/common/config.hpp"
#include "ti_msp_dl_config.h"

void buzzer_init(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_on(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_beep(uint32_t ms)
{
    buzzer_on();
    delay_cycles(ms * 32000U);
    buzzer_off();
}
