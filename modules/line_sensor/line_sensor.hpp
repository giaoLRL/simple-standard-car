#pragma once
/*
 * line_sensor.hpp / line_sensor.cpp — 8 路循迹传感器模块
 *
 * 硬件: 8 路红外对管 → CD4051 模拟开关（AD0=PB20, AD1=PB25, AD2=PB24）
 *       → ADC0 通道 3（PA24）
 *
 * 参考: 感为无MCU灰度传感器例程
 *   - No_Mcu_Ganv_Grayscale_Sensor.c（均值滤波 + 二值化 + 归一化）
 *   - No_Mcu_Ganv_Grayscale_Sensor_Config.h（宏抽象 + 数据结构）
 *
 * 依赖: ISensorArray（实现接口）、config.hpp（参数 + GPIO 宏）
 *
 * 设计要点:
 *   1. 8 采样均值滤波，由 DMA burst 一次搬运 8 个样本后求平均
 *   2. 预计算归一化系数（init 时），运行时只做乘法
 *   3. 二值化输出（1:2 / 2:1 灰度阈值），用于快速丢线判断
 *   4. 加权平均线位置：Σ(dark_i × pos_i) / Σ(dark_i)
 *   5. ADC 配置为 repeat-single-channel，由 DMA 触发搬运
 */

#include "ti_msp_dl_config.h"
#include "modules/common/config.hpp"
#include "modules/common/iinterfaces.hpp"

/* ============================================================
 *  LineSensor — 8 路循迹传感器
 * ============================================================ */

class LineSensor : public ISensorArray {
public:
    LineSensor();

    /* ISensorArray 实现 */
    bool     update() override;
    int32_t  get_error()                 const override { return error_; }
    bool     line_found()                const override { return line_found_; }
    uint8_t  get_digital()               const override { return digital_; }
    bool     get_normalized(uint16_t *result) const override;
    bool     get_analog(uint16_t *result)     const override;

    /* 硬件初始化 */
    void init();

    /*
     * 带校准参数的初始化（参照感为例程 No_MCU_Ganv_Sensor_Init 模式）
     *   white: 白基准数组（8 元素）
     *   black: 黑基准数组（8 元素）
     * 调用后 ok_ 置 true，归一化系数和灰度阈值可用
     */
    void init_with_calibration(const uint16_t *white, const uint16_t *black);

    /*
     * 按键触发黑白双标定（启动时调用一次）：
     *   第 1 次按键：采集白底；
     *   第 2 次按键：采集黑线。
     * 已废弃原 calibrate_white()，保留同名包装以兼容旧调用。
     */
    void calibrate();
    void calibrate_white() { calibrate(); }

    /* 传感器就绪标志（校准完成后为 true）*/
    bool is_ready() const { return ok_; }

    /* 调试访问器 */
    uint16_t get_raw(uint8_t index)       const { return analog_[index]; }
    uint16_t get_white_cal(uint8_t index) const { return cal_white_[index]; }
    uint16_t get_black_cal(uint8_t index) const { return cal_black_[index]; }
    uint16_t get_normalized_ch(uint8_t i) const { return normal_[i]; }

private:
    /* 切换一路 MUX，由 DMA 连续搬运 ADC_SAMPLES_PER_CHANNEL 个样本到 buffer，
     * 阻塞等待 burst 完成，返回该路均值 */
    bool read_adc_burst_dma_(uint8_t ch, uint16_t *result);

    /*
     * 成员变量
     *
     * 标定数据
     */
    uint16_t cal_white_[SENSOR_COUNT];  /* 白基准（校准值）*/
    uint16_t cal_black_[SENSOR_COUNT];  /* 黑基准（校准值）*/
    uint16_t gray_white_[SENSOR_COUNT]; /* 白色判定阈值 (white×2+black)/3 */
    uint16_t gray_black_[SENSOR_COUNT]; /* 黑色判定阈值 (white+black×2)/3 */
    float    norm_factor_[SENSOR_COUNT]; /* 归一化系数: bits / (white-black) */

    /* 运行时数据 */
    uint16_t analog_[SENSOR_COUNT];     /* 原始模拟量（均值滤波后）*/
    uint16_t normal_[SENSOR_COUNT];     /* 归一化值 0 ~ ADC_MAX_VALUE */
    uint8_t  digital_;                  /* 二值化输出（bit 0~7）*/
    int32_t  error_;                    /* 线位置偏差 */
    bool     line_found_;               /* 是否检测到线 */
    bool     ok_;                       /* 传感器就绪标志 */
};

extern LineSensor g_line_sensor;   /* 全局单例 */
