#include "modules/control/turn_state_machine.hpp"
#include "modules/common/config.hpp"

/*
 * 探头位定义（与 line_sensor 中 digital_ 顺序一致）：
 *   bit0~2：左侧 3 路   bit3~4：中心 2 路   bit5~7：右侧 3 路
 */
static constexpr uint8_t LEFT_MASK   = 0x07U;  /* 0000 0111 */
static constexpr uint8_t RIGHT_MASK  = 0xE0U;  /* 1110 0000 */
static constexpr uint8_t CENTER_MASK = 0x18U;  /* 0001 1000 */

static TurnState s_turn_state  = TurnState::STRAIGHT;
static bool      s_center_lost = false;
static uint32_t  s_turn_tick   = 0;

void turn_fsm_init(void)
{
    s_turn_state  = TurnState::STRAIGHT;
    s_center_lost = false;
    s_turn_tick   = 0;
}

void turn_fsm_update(uint8_t digital, bool line_found)
{
    switch (s_turn_state) {
        case TurnState::STRAIGHT:
            if ((digital & LEFT_MASK) == LEFT_MASK) {
                s_turn_state  = TurnState::TURN_LEFT;
                s_center_lost = false;
                s_turn_tick   = 0;
            } else if ((digital & RIGHT_MASK) == RIGHT_MASK) {
                s_turn_state  = TurnState::TURN_RIGHT;
                s_center_lost = false;
                s_turn_tick   = 0;
            } else if (!line_found) {
                s_turn_state = TurnState::LOST;
            }
            break;

        case TurnState::TURN_LEFT:
        case TurnState::TURN_RIGHT:
            s_turn_tick++;

            if ((digital & CENTER_MASK) == 0) {
                s_center_lost = true;   /* 中心探头曾经丢线 */
            }
            if (s_center_lost && (digital & CENTER_MASK) != 0) {
                s_turn_state = TurnState::STRAIGHT;   /* 重新找到中心线 */
            }
            if (s_turn_tick >= (TURN_TIMEOUT_MS / CONTROL_PERIOD_MS)) {
                s_turn_state = TurnState::LOST;       /* 转弯超时 */
            }
            break;

        case TurnState::LOST:
            if (line_found) {
                s_turn_state = TurnState::STRAIGHT;
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
    return s_turn_state == TurnState::STRAIGHT;
}

bool turn_fsm_is_lost(void)
{
    return s_turn_state == TurnState::LOST;
}

void turn_fsm_get_motor_commands(int16_t *left, int16_t *right)
{
    switch (s_turn_state) {
        case TurnState::TURN_LEFT:
            *left  = TURN_INNER_SPEED;
            *right = TURN_OUTER_SPEED;
            break;

        case TurnState::TURN_RIGHT:
            *left  = TURN_OUTER_SPEED;
            *right = TURN_INNER_SPEED;
            break;

        case TurnState::STRAIGHT:
        case TurnState::LOST:
        default:
            *left  = 0;
            *right = 0;
            break;
    }
}
