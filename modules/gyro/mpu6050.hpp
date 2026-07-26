/**
 * @brief       MPU6050 六轴传感器驱动 (硬件 I2C0)
 * @details     I2C 读写完全参照云台项目验证方案:
 *              每字节独立 START+STOP 事务，规避 MSPM0 I2C 硅级 bug。
 *              仅使用陀螺 Z 轴进行偏航角度积分，不依赖 DMP。
 * @author      Haoqi Liu
 * @date        2026-07-26
 *
 * 硬件连接:
 *   SCL → PA11 (IOMUX_PINCM22)
 *   SDA → PA10 (IOMUX_PINCM21)
 *   INT → PA27 (IOMUX_PINCM60, 预留 GPIO 输入)
 */

#pragma once

#include <cstdint>
#include "modules/common/config.hpp"

/* ---- MPU6050 寄存器地址 ---- */
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75

/* I2C 7-bit 地址 */
#define MPU6050_I2C_ADDR        0x68

/* I2C 超时 (循环次数, 参照云台) */
#define I2C_TIMEOUT             100000

class Mpu6050 {
public:
    bool init();
    void calibrate_bias();
    void update();

    float angle      = 0.0f;
    float gyro_z_dps = 0.0f;
    float bias       = 0.0f;

private:
    bool    write_reg(uint8_t reg, uint8_t data);
    bool    read_regs(uint8_t reg, uint8_t *buf, uint8_t len);
    uint8_t read_reg(uint8_t reg);
    int16_t read_word(uint8_t reg_h);

    uint32_t last_update_us = 0;
};

extern Mpu6050 g_gyro;

/* ---- 调试变量 (CCS Expressions) ---- */
extern volatile uint8_t  g_dbg_gyro_step;
extern volatile uint8_t  g_dbg_gyro_ack;
extern volatile uint8_t  g_dbg_gyro_who;
extern volatile uint32_t g_dbg_i2c_msr;
extern volatile uint32_t g_dbg_update_cnt;
extern volatile int16_t  g_dbg_raw_z;

typedef struct {
    volatile uint32_t fail_step;
    volatile uint32_t fail_reason;
    volatile uint32_t status_value;
    volatile uint32_t total_fails;
    volatile uint32_t last_reg;
} I2CDebug;
extern volatile I2CDebug g_i2c_dbg;

