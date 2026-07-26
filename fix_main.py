import re

with open(r"C:\Users\PC\Documents\新建的循迹小车\main.cpp", "r", encoding="utf-8") as f:
    content = f.read()

# 1. Fix gyro control block: in-place turning
old_gyro = r"""#if ENABLE_GYRO
        if \(g_gyro_mode_on\) \{
            g_gyro\.update\(\);
            g_dbg_gyro_angle = g_gyro\.angle;
            g_dbg_gyro_dps   = g_gyro\.gyro_z_dps;

            float angle_error = g_gyro_target_angle - g_gyro\.angle;
            int32_t correction = round_to_i32\(g_gyro_pid\.calc\(angle_error\)\);
            g_dbg_gyro_corr = correction;

            if \(correction >  \(int32_t\)g_base_speed\) correction =  \(int32_t\)g_base_speed;
            if \(correction < -\(int32_t\)g_base_speed\) correction = -\(int32_t\)g_base_speed;

            int32_t raw_left  = \(int32_t\)g_base_speed \+ correction;
            int32_t raw_right = \(int32_t\)g_base_speed - correction;

            int16_t left_cmd, right_cmd;
            apply_diff_steering\(raw_left, raw_right, &left_cmd, &right_cmd, PWM_MAX\);
#if ENABLE_ENCODER
            speed_loop_trim\(&left_cmd, &right_cmd\);
#endif
            g_motor\.set_speed\(left_cmd, right_cmd\);
            g_dbg_left_cmd  = left_cmd;
            g_dbg_right_cmd = right_cmd;
            g_dbg_correction = correction;
            buzzer_off\(\);
            goto control_done;
        \}
        else
#endif"""

new_gyro = """#if ENABLE_GYRO
        if (g_gyro_mode_on) {
            g_gyro.update();
            g_dbg_gyro_angle = g_gyro.angle;
            g_dbg_gyro_dps   = g_gyro.gyro_z_dps;

            /* ????????? PID??????????????
             *   correction > 0 ??? ?????? ??????/??????
             *   correction < 0 ??? ?????? ??????/?????? */
            float angle_error = g_gyro_target_angle - g_gyro.angle;
            int32_t correction = round_to_i32(g_gyro_pid.calc(angle_error));
            g_dbg_gyro_corr = correction;

            if (correction >  PWM_MAX) correction =  PWM_MAX;
            if (correction < -PWM_MAX) correction = -PWM_MAX;

            /* ??????: ???=+correction, ???=-correction */
            g_motor.set_speed((int16_t)correction, (int16_t)(-correction));
            g_dbg_left_cmd  = (int16_t)correction;
            g_dbg_right_cmd = (int16_t)(-correction);
            g_dbg_correction = correction;
            buzzer_off();
            goto control_done;
        }
        else
#endif"""

content = re.sub(old_gyro, new_gyro, content, flags=re.DOTALL)

# 2. Fix control_done delay
old_done = r"""control_done:
        next_control_ms \+= CONTROL_PERIOD_MS;
        uint32_t after_work_ms = timebase_millis\(\);
        if \(\(int32_t\)\(next_control_ms - after_work_ms\) > 0\) \{
            while \(\(int32_t\)\(next_control_ms - timebase_millis\(\)\) > 0\) \{
                __NOP\(\);
            \}
        \} else \{
            next_control_ms = after_work_ms;
        \}"""

new_done = """control_done:
        for (volatile uint32_t d = 0; d < (CPUCLK_FREQ / 100U); d++) {
            __NOP();
        }"""

content = re.sub(old_done, new_done, content, flags=re.DOTALL)

# 3. Remove next_control_ms declaration
content = re.sub(r'    uint32_t next_control_ms = timebase_millis\(\);\n', '', content)

with open(r"C:\Users\PC\Documents\新建的循迹小车\main.cpp", "w", encoding="utf-8") as f:
    f.write(content)

print("Done")
