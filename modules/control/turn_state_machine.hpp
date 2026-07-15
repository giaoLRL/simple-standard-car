#pragma once

#include <stdint.h>

/* 直角弯状态机 */

enum class TurnState {
    STRAIGHT,   /* 正常循迹 */
    TURN_LEFT,  /* 左直角弯 */
    TURN_RIGHT, /* 右直角弯 */
    LOST,       /* 转弯超时或普通丢线 */
};

/* 初始化状态机 */
void turn_fsm_init(void);

/*
 * 根据当前 digital（bit=1 表示黑线）和 line_found 更新状态。
 * 需要在固定周期内调用（默认 2 ms）。
 */
void turn_fsm_update(uint8_t digital, bool line_found);

/* 获取当前状态（调试用） */
TurnState turn_fsm_get_state(void);

/* 状态查询辅助 */
bool turn_fsm_is_straight(void);
bool turn_fsm_is_lost(void);

/* 根据当前状态输出电机指令 */
void turn_fsm_get_motor_commands(int16_t *left, int16_t *right);
