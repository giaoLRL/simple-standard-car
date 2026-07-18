#pragma once
/*
 * iinterfaces.hpp — 硬件抽象接口定义
 *
 * 业务逻辑层通过此文件定义的纯虚接口访问硬件，不依赖具体实现。
 * main.cpp 负责创建具体实例并注入依赖。
 *
 * 参考: 感为无MCU灰度传感器例程的数据获取模式
 *       Get_Digtal_For_User / Get_Normalize_For_User / Get_Anolog_Value
 *
 * 依赖: 无
 */

#include <stdint.h>

/* ============================================================
 *  IMotorDriver — 电机驱动接口
 * ============================================================ */

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    /*
     * 设置左右轮速度
     *   left:  左轮 PWM 占空比，正=前进，负=后退，范围 [-PWM_MAX, PWM_MAX]
     *   right: 右轮 PWM 占空比，正=前进，负=后退，范围 [-PWM_MAX, PWM_MAX]
     */
    virtual void set_speed(int16_t left, int16_t right) = 0;

    /* 立即停止两轮 */
    virtual void stop() = 0;

    static constexpr int16_t MAX = 1000;
};

/* ============================================================
 *  ISensorArray — 传感器阵列接口
 *
 *  参照感为例程提供三种数据访问方式：
 *    1. 模拟量 — 原始 ADC 值（用于标定/调试）
 *    2. 数字量 — 二值化结果（8 位，每位对应一个通道）
 *    3. 归一化量 — 0~ADC_MAX_VALUE 线性归一化值
 * ============================================================ */

class ISensorArray {
public:
    virtual ~ISensorArray() = default;

    /*
     * 读取全部传感器并计算位置误差。
     * 同时更新模拟量、数字量、归一化量。
     *   返回: true = 检测到线，false = 丢线
     */
    virtual bool update() = 0;

    /* 获取线位置偏差（加权平均，与 g_sensor_position 同量纲）*/
    virtual int32_t get_error() const = 0;

    /* 是否检测到线 */
    virtual bool line_found() const = 0;

    /*
     * 获取数字量输出（8 位二值化结果）
     *   bit 0 = 通道 0，1 = 黑（线），0 = 白（地板）
     *   仅在校准完成后有效
     */
    virtual uint8_t get_digital() const = 0;

    /*
     * 获取归一化值（0 ~ ADC_MAX_VALUE）
     *   result: 8 元素数组，存放各通道归一化结果
     *   返回: true = 校准已完成，数据有效；false = 未校准
     */
    virtual bool get_normalized(uint16_t *result) const = 0;

    /*
     * 获取原始模拟量（ADC 原始值，不依赖校准状态）
     *   result: 8 元素数组
     *   返回: true = 数据有效
     */
    virtual bool get_analog(uint16_t *result) const = 0;
};
