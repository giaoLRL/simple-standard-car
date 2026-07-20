#include "modules/control/turn_state_machine.hpp"
#include "modules/common/config.hpp"

/*
 * 探头位定义（与 line_sensor 中 digital_ 顺序一致）：
 *   bit0~2：左侧 3 路   bit3~4：中心 2 路   bit5~7：右侧 3 路
 */
static constexpr uint8_t LEFT_MASK   = 0x07U;  /* 0000 0111 */
static constexpr uint8_t RIGHT_MASK  = 0xE0U;  /* 1110 0000 */
static constexpr uint8_t CENTER_MASK = 0x18U;  /* 0001 1000 */

/* 备选转弯退出: 中心探头连续在线 N 周期则退出 (防 FSM 卡死) */
static constexpr uint8_t TURN_EXIT_CENTER_COUNT = 15U;  /* 15 × 10ms = 150ms */

/* LOST→STRAIGHT 消抖: 需连续 N 周期检测到线才恢复, 防噪声误触发 */
static constexpr uint8_t LOST_RECOVERY_DEBOUNCE = 3U;  /* 3 × 10ms = 30ms */

/* 转弯前直走延时周期数 (运行时计算) */

static TurnState s_turn_state       = TurnState::STRAIGHT;
static TurnState s_turn_pending     = TurnState::STRAIGHT;  /* 延时结束后执行的方向 */
static bool      s_center_lost      = false;
static uint32_t  s_turn_started_ms  = 0;
static uint8_t   s_center_stable_ct = 0;  /* 中心在线连续计数 */
static uint8_t   s_lost_recovery_ct = 0;  /* LOST 恢复消抖计数 */
static uint8_t   s_turn_advance_ct  = 0;  /* 转弯前延时计数 */

void turn_fsm_init(void)
{
    s_turn_state       = TurnState::STRAIGHT;
    s_turn_pending     = TurnState::STRAIGHT;
    s_center_lost      = false;
    s_turn_started_ms  = 0;
    s_center_stable_ct = 0;
    s_lost_recovery_ct = 0;
    s_turn_advance_ct  = 0;
}

/*
 * turn_fsm_update — 直角弯状态转移逻辑
 *
 * 调用方 (main.cpp) 负责在固定 CONTROL_PERIOD_MS (10ms) 周期内调用此函数。
 * 转移条件:
 *   STRAIGHT → LOST:        line_found=false (优先判断, 防迟滞位残留误触发转弯)
 *   STRAIGHT → TURN_DELAY:  左侧3路或右侧3路全部见黑 (先直走再转)
 *   TURN_DELAY → TURN_LEFT:  延时到期, 按 STRAIGHT 阶段预设的 s_turn_pending 执行
 *   TURN_DELAY → TURN_RIGHT: 同上, TURN_DELAY 阶段不重新判断左右
 *   TURN → STRAIGHT:        主条件: 中心丢线后重新在线 (exit on re-capture)
 *                           备选: 中心持续在线 150ms (仅当曾丢线后在线才生效)
 *   TURN → LOST:            超时 (TURN_TIMEOUT_MS 内未退出)
 *   LOST → STRAIGHT:       连续 3 周期检测到线 (消抖, 防噪声误恢复)
 *
 * 注: millis() 溢出 (49.7 天后) 会触发误超时,
 *     持续运行超过此时间需添加溢出保护。 */
void turn_fsm_update(uint8_t digital, bool line_found, uint32_t now_ms)
{
    switch (s_turn_state) {
        case TurnState::STRAIGHT:
            /* 丢线优先判断: 若 digital_ 的迟滞位残留满足转弯条件但线已丢失,
             *   先触发 LOST 停车而非误入转弯 (digital_ 与 line_found 独立计算)。*/
            if (!line_found) {
                s_turn_state = TurnState::LOST;
            } else if ((digital & LEFT_MASK) == LEFT_MASK) {
                s_turn_state      = TurnState::TURN_DELAY;
                s_turn_pending    = TurnState::TURN_LEFT;
                s_turn_advance_ct = (uint8_t)(g_turn_advance_ms / CONTROL_PERIOD_MS);
            } else if ((digital & RIGHT_MASK) == RIGHT_MASK) {
                s_turn_state      = TurnState::TURN_DELAY;
                s_turn_pending    = TurnState::TURN_RIGHT;
                s_turn_advance_ct = (uint8_t)(g_turn_advance_ms / CONTROL_PERIOD_MS);
            }
            break;

        case TurnState::TURN_DELAY:
            /* 延时期间先检查丢线，再倒计数。
             * 延时到期后进入实际转弯。 */
            if (!line_found) {
                s_turn_state = TurnState::LOST;
                break;
            }
            if (s_turn_advance_ct > 0U) {
                s_turn_advance_ct--;
            }
            if (s_turn_advance_ct == 0U) {
                s_turn_state  = s_turn_pending;
                s_center_lost = false;
                s_center_stable_ct = 0;
                s_turn_started_ms = now_ms;
            }
            break;

        case TurnState::TURN_LEFT:
        case TurnState::TURN_RIGHT:
            /* 记录中心是否曾经丢线 */
            if ((digital & CENTER_MASK) == 0) {
                s_center_lost = true;
                s_center_stable_ct = 0;
            }

            /* 中心在线计数 (用于备选退出) */
            if ((digital & CENTER_MASK) != 0) {
                s_center_stable_ct++;
            } else {
                s_center_stable_ct = 0;
            }

            /* 转弯退出:
             *   主条件: 中心丢线后重新在线 → 立即退出
             *   备选: 中心持续在线超时 (需先丢线, 仅中心从未完全脱线时触发)
             *   注意: 中心从未丢线时备选条件不生效, 依赖 2s 超时进入 LOST */
            if ((s_center_lost && (digital & CENTER_MASK) != 0) ||
                (s_center_lost && s_center_stable_ct >= TURN_EXIT_CENTER_COUNT)) {
                s_turn_state  = TurnState::STRAIGHT;   /* 退出转弯 */
                s_center_lost = false;
                s_center_stable_ct = 0;
                s_turn_started_ms = 0;
            } else if ((uint32_t)(now_ms - s_turn_started_ms) >= g_turn_timeout_ms) {
                s_turn_state = TurnState::LOST;       /* 转弯超时 */
            }
            break;

        case TurnState::LOST:
            /* 消抖: 需连续 LOST_RECOVERY_DEBOUNCE 周期检测到线才恢复,
             *   防止传感器噪声单周期误触发导致反复启停。*/
            if (line_found) {
                s_lost_recovery_ct++;
                if (s_lost_recovery_ct >= LOST_RECOVERY_DEBOUNCE) {
                    s_turn_state = TurnState::STRAIGHT;
                    s_turn_started_ms = 0;
                    s_center_lost = false;
                    s_lost_recovery_ct = 0;
                }
            } else {
                s_lost_recovery_ct = 0;
            }
            break;
    }
}

TurnState turn_fsm_get_state(void)
{
    return s_turn_state;
}

bool turn_fsm_is_straight(void)
{
    return s_turn_state == TurnState::STRAIGHT ||
           s_turn_state == TurnState::TURN_DELAY;
}

bool turn_fsm_is_lost(void)
{
    return s_turn_state == TurnState::LOST;
}

void turn_fsm_get_motor_commands(int16_t *left, int16_t *right)
{
    switch (s_turn_state) {
        case TurnState::TURN_LEFT:
            *left  = (int16_t)g_turn_inner_speed;
            *right = (int16_t)g_turn_outer_speed;
            break;

        case TurnState::TURN_RIGHT:
            *left  = (int16_t)g_turn_outer_speed;
            *right = (int16_t)g_turn_inner_speed;
            break;

        case TurnState::STRAIGHT:
        case TurnState::LOST:
        default:
            /* STRAIGHT: 由 PID 差速转向接管, 此处返回 0 为占位值 */
            /* LOST:     由 main.cpp 调用 g_motor.stop() 接管 */
            *left  = 0;
            *right = 0;
            break;
    }
}
