#include "modules/common/config.hpp"

uint16_t g_base_speed       = 200;
uint16_t g_turn_outer_speed = 170;
uint16_t g_turn_inner_speed = 80;

#if ENABLE_ENCODER
bool  g_speed_loop_on = true;
bool  g_dir_pid_on    = true;
float g_speed_kp      = SPEED_KP_DEFAULT;
float g_speed_ki      = SPEED_KI_DEFAULT;
float g_speed_kd      = SPEED_KD_DEFAULT;
bool  g_enc_left_rev  = ENC_LEFT_REVERSED;
bool  g_enc_right_rev = ENC_RIGHT_REVERSED;
volatile bool g_spd_need_warmstart = true;
#endif

#if ENABLE_GYRO
bool  g_gyro_mode_on     = true;
float g_gyro_target_angle = 0.0f;
float g_gyro_kp = GYRO_KP_DEFAULT;
float g_gyro_ki = GYRO_KI_DEFAULT;
float g_gyro_kd = GYRO_KD_DEFAULT;
#endif
