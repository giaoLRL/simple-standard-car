/**
 * @brief       MPU6050 驱动实现 (硬件 I2C0)
 * @details     I2C 读写完全参照云台项目验证方案:
 *              每字节独立 START+STOP 事务。
 *              仅使用陀螺 Z 轴进行偏航角度积分。
 * @author      Haoqi Liu
 * @date        2026-07-26
 */

#include "modules/gyro/mpu6050.hpp"
#include "modules/timebase/timebase.hpp"
#include "ti_msp_dl_config.h"
#include <cmath>

Mpu6050 g_gyro;

/* ---- 调试变量 ---- */
volatile uint8_t  g_dbg_gyro_step = 0;
volatile uint8_t  g_dbg_gyro_ack  = 0;
volatile uint8_t  g_dbg_gyro_who  = 0;
volatile uint32_t g_dbg_i2c_msr   = 0;
volatile I2CDebug g_i2c_dbg;

/* ---- 运行时调试 ---- */
volatile uint32_t g_dbg_update_cnt = 0;
volatile int16_t  g_dbg_raw_z = 0;

/* ============================================================
 *  底层 I2C 读写 (参照云台 mpu6050.c, 已验证可靠)
 * ============================================================ */

/*
 * 写寄存器 — 2 字节: reg + data, 一次 START+STOP 事务
 * 完全复制云台 mpu6050_write_reg 的成功模式.
 */
bool Mpu6050::write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU6050_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    uint32_t timeout = I2C_TIMEOUT;
    uint32_t status;
    do {
        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (--timeout == 0) return false;
    } while ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0);

    if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                  DL_I2C_CONTROLLER_STATUS_ADDR_ACK  |
                  DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
        return false;
    }
    return true;
}

/*
 * 读寄存器 — 逐字节, 每字节独立 START+STOP
 * 完全复制云台 mpu6050_read_regs 的成功模式.
 */
bool Mpu6050::read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    for (uint8_t n = 0; n < len; n++) {
        uint32_t status;
        uint32_t timeout;

        g_i2c_dbg.last_reg = reg + n;

        /*
         * Step 1: 写当前寄存器地址 (TX, Advanced API, START+STOP)
         * 完全复制 mpu6050_write_reg 的成功模式, 只发 1 字节.
         */
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
        DL_I2C_startControllerTransferAdvanced(I2C_0_INST, MPU6050_I2C_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX, 1,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_ENABLE);

        timeout = I2C_TIMEOUT;
        do {
            status = DL_I2C_getControllerStatus(I2C_0_INST);
            if (--timeout == 0) {
                g_i2c_dbg.fail_step = 1;
                g_i2c_dbg.fail_reason = 1;
                g_i2c_dbg.total_fails++;
                return false;
            }
        } while (!(status & DL_I2C_CONTROLLER_STATUS_IDLE));

        if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                      DL_I2C_CONTROLLER_STATUS_ADDR_ACK |
                      DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
            g_i2c_dbg.fail_step = 1;
            g_i2c_dbg.fail_reason = 2;
            g_i2c_dbg.status_value = status;
            g_i2c_dbg.total_fails++;
            return false;
        }

        /* 寄存器地址自增 (下一个字节) */
        reg++;

        /*
         * Step 2: 读 1 字节 (RX, Advanced API, START+STOP, ACK_DISABLE)
         * ACK_DISABLE → 1 字节读完后立即 NACK → 从机释放总线 → STOP.
         */
        DL_I2C_startControllerTransferAdvanced(I2C_0_INST, MPU6050_I2C_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_RX, 1,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);

        timeout = I2C_TIMEOUT;
        do {
            status = DL_I2C_getControllerStatus(I2C_0_INST);
            if (--timeout == 0) {
                g_i2c_dbg.fail_step = 2;
                g_i2c_dbg.fail_reason = 1;
                g_i2c_dbg.total_fails++;
                return false;
            }
        } while (!(status & DL_I2C_CONTROLLER_STATUS_IDLE));

        if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                      DL_I2C_CONTROLLER_STATUS_ADDR_ACK |
                      DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
            g_i2c_dbg.fail_step = 2;
            g_i2c_dbg.fail_reason = 2;
            g_i2c_dbg.status_value = status;
            g_i2c_dbg.total_fails++;
            return false;
        }

        buf[n] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    return true;
}

/* ---- 便利封装 ---- */

uint8_t Mpu6050::read_reg(uint8_t reg)
{
    uint8_t val = 0xFF;
    read_regs(reg, &val, 1);
    return val;
}

int16_t Mpu6050::read_word(uint8_t reg_h)
{
    uint8_t hi, lo;
    if (!read_regs(reg_h,     &hi, 1)) return 0;
    if (!read_regs(reg_h + 1, &lo, 1)) return 0;
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

/* ============================================================
 *  init — 初始化 MPU6050 (参照云台 mpu6050_init)
 * ============================================================ */
bool Mpu6050::init()
{
    g_dbg_gyro_step = 0;
    g_dbg_gyro_ack  = 0;
    g_dbg_gyro_who  = 0;
    for (volatile uint32_t* p = (volatile uint32_t*)&g_i2c_dbg; p < (volatile uint32_t*)(&g_i2c_dbg + 1); p++) { *p = 0; }

    /*
     * 轻量 I2C 复位: 不碰 Target (SysConfig 已配 Controller-Only),
     * 只保证每次上电/复位后 I2C 状态机干净.
     * 完全复制云台 mpu6050_init 的复位序列.
     */
    {
        DL_I2C_ClockConfig clk_cfg;
        DL_I2C_getClockConfig(I2C_0_INST, &clk_cfg);
        DL_I2C_reset(I2C_0_INST);
        DL_I2C_enablePower(I2C_0_INST);
        for (volatile int i = 0; i < 16; i++) { __asm("nop"); }
        DL_I2C_setClockConfig(I2C_0_INST, &clk_cfg);
        DL_I2C_setTimerPeriod(I2C_0_INST, 15);  /* ~200kHz */
        DL_I2C_disableControllerClockStretching(I2C_0_INST);
        DL_I2C_enableController(I2C_0_INST);
    }

    /* MPU6050 上电后内部 PLL 需要稳定时间, 等 50ms */
    g_dbg_gyro_step = 1;
    for (volatile int i = 0; i < 400000; i++) { __asm("nop"); }

    /* 检测 WHO_AM_I — 最多重试 10 次 (参照云台) */
    uint8_t whoami = 0;
    for (int retry = 0; retry < 10; retry++) {
        g_dbg_gyro_step = 2;
        if (read_regs(MPU6050_WHO_AM_I, &whoami, 1)) {
            g_dbg_gyro_who = whoami;
            if (whoami == 0x68 || whoami == 0x70) break;
        }
        g_dbg_i2c_msr = DL_I2C_getControllerStatus(I2C_0_INST);
        for (volatile int i = 0; i < 80000; i++) { __asm("nop"); }  /* ~10ms */
    }
    if (whoami != 0x68 && whoami != 0x70) {
        g_dbg_gyro_step = 7;
        return false;
    }

    /* 2. 唤醒 + 选择 PLL 时钟源 (CLKSEL=1) */
    g_dbg_gyro_step = 3;
    if (!write_reg(MPU6050_PWR_MGMT_1, 0x01)) {
        g_dbg_gyro_step = 7;
        return false;
    }

    /* 等待稳定 ~40ms */
    for (volatile int i = 0; i < 320000; i++) { __asm("nop"); }

    /* 3. 采样率分频: 0 → 1kHz (陀螺) */
    if (!write_reg(MPU6050_SMPLRT_DIV, 0x00)) {
        g_dbg_gyro_step = 7;
        return false;
    }

    /* 4. DLPF: BW=44Hz */
    if (!write_reg(MPU6050_CONFIG, 0x03)) {
        g_dbg_gyro_step = 7;
        return false;
    }

    /* 5. 陀螺量程: +/-250dps */
    if (!write_reg(MPU6050_GYRO_CONFIG, 0x00)) {
        g_dbg_gyro_step = 7;
        return false;
    }

    /* 6. 加速度计量程: +/-2g */
    if (!write_reg(MPU6050_ACCEL_CONFIG, 0x00)) {
        g_dbg_gyro_step = 7;
        return false;
    }

    last_update_us = timebase_micros();
    g_dbg_gyro_step = 7;
    return true;
}

/* ============================================================
 *  calibrate_bias — 零偏校准
 * ============================================================ */
void Mpu6050::calibrate_bias()
{
    float sum = 0.0f;
    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
        int16_t raw = read_word(MPU6050_GYRO_ZOUT_H);
        sum += (float)raw;
    }
    bias = (sum / (float)GYRO_BIAS_SAMPLES) / GYRO_DPS_PER_LSB;
    angle = 0.0f;
}

/* ============================================================
 *  update — 角度更新
 * ============================================================ */
void Mpu6050::update()
{
    int16_t raw_z = read_word(MPU6050_GYRO_ZOUT_H); g_dbg_raw_z = raw_z; g_dbg_update_cnt++; g_dbg_raw_z = raw_z; g_dbg_update_cnt++;
    gyro_z_dps = (float)raw_z / GYRO_DPS_PER_LSB;

    uint32_t now_us = timebase_micros();
    float dt;
    if (last_update_us == 0) {
        dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    } else {
        uint32_t delta_us = now_us - last_update_us;
        if (delta_us == 0) delta_us = 10000UL;
        dt = (float)delta_us / 1000000.0f;
    }
    last_update_us = now_us;

    angle += (gyro_z_dps - bias) * dt;

    if (!std::isfinite(angle)) {
        angle = 0.0f;
    }
}




