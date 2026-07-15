/*
 * line_sensor.cpp — 8 路循迹传感器模块实现
 *
 * 硬件: CD4051 模拟开关 + ADC0_CH3
 *
 * 参考: 感为无MCU灰度传感器例程
 *   - Get_Analog_value:  8 采样均值滤波 + 通道切换
 *   - convertAnalogToDigital: 二值化（1:2 / 2:1 灰度阈值）
 *   - normalizeAnalogValues:  预计算系数 × (raw - black)
 *   - No_MCU_Ganv_Sensor_Init: 校准参数计算 + 系数预计算
 *
 * 依赖: config.hpp（GPIO 宏）、iinterfaces.hpp（接口）
 *
 * 已知限制:
 *   1. ADC 阻塞读取，8 通道 × 8 采样 ≈ 0.8 ms（对 500 Hz 循环可接受）
 *   2. 不含 Direction 反转（硬件接线固定，不需要运行时反转）
 */

#include "modules/line_sensor/line_sensor.hpp"

/* 全局单例 */
LineSensor g_line_sensor;

/* ADC 转换完成标志（ISR 置位 → 主循环轮询，仅本文件内可见）*/
static volatile bool g_adc_done;

/* ============================================================
 *  构造与初始化
 * ============================================================ */

LineSensor::LineSensor()
    : digital_{0}
    , error_{0}
    , line_found_{false}
    , ok_{false}
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        cal_white_[i]   = 0;
        cal_black_[i]   = 0;
        gray_white_[i]  = 0;
        gray_black_[i]  = 0;
        norm_factor_[i] = 0.0f;
        analog_[i]      = 0;
        normal_[i]      = 0;
    }
}

void LineSensor::init()
{
    /*
     * ADC 和 GPIO 已在 SYSCFG_DL_init() 中初始化。
     * 此处复位运行时状态。标定数据保留。
     */
    g_adc_done   = false;
    digital_     = 0;
    error_       = 0;
    line_found_  = false;
}

/* ============================================================
 *  带校准的初始化（参照 No_MCU_Ganv_Sensor_Init）
 * ============================================================ */

void LineSensor::init_with_calibration(const uint16_t *white,
                                       const uint16_t *black)
{
    uint16_t w, b, temp;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        w = white[i];
        b = black[i];

        /* 确保白值 > 黑值（必要时交换）*/
        if (b >= w) {
            temp = w;
            w    = b;
            b    = temp;
        }

        /* 计算灰度阈值（1:2 和 2:1 分界点）
         *   > gray_white → 判定为白（bit=1）
         *   < gray_black → 判定为黑（bit=0）
         *   中间区域 → 保持上一状态（迟滞）*/
        gray_white_[i] = (uint16_t)(((uint32_t)w * 2 + b) / 3U);
        gray_black_[i] = (uint16_t)(((uint32_t)w + b * 2) / 3U);

        cal_white_[i] = w;
        cal_black_[i] = b;

        /* 预计算归一化系数: bits / span
         * 运行时只需: normal = (raw - black) × factor */
        int32_t span = (int32_t)w - (int32_t)b;
        if (span < 1) {
            norm_factor_[i] = 0.0f;  /* 无效通道 */
        } else {
            norm_factor_[i] = (float)ADC_MAX_VALUE / (float)span;
        }
    }

    ok_ = true;
}

/* ============================================================
 *  快速白底采样（启动时调用）
 * ============================================================ */

void LineSensor::calibrate_white()
{
    uint32_t sum[SENSOR_COUNT] = {0};
    uint16_t white[SENSOR_COUNT];
    uint16_t black[SENSOR_COUNT];

    for (uint16_t scan = 0; scan < WHITE_STARTUP_SCANS; scan++) {
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            SWITCH_MUX_CHANNEL(i);
            delay_cycles(640U);  /* 模拟开关稳定等待 */
            sum[i] += read_adc_avg_();
        }
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        white[i] = (uint16_t)(sum[i] / WHITE_STARTUP_SCANS);
        black[i] = (white[i] > INITIAL_BLACK_SPAN)
                       ? (white[i] - INITIAL_BLACK_SPAN)
                       : 0U;
    }

    init_with_calibration(white, black);
}

/* ============================================================
 *  ISensorArray 接口 — 主更新
 *
 *  流程（参照 No_Mcu_Ganv_Sensor_Task_Without_tick）:
 *    1. 采集 8 通道模拟量（均值滤波）
 *    2. 二值化处理
 *    3. 归一化处理
 *    4. 加权平均计算线偏差
 * ============================================================ */

bool LineSensor::update()
{
    /* ---- 1. 采集 8 通道模拟量（均值滤波）---- */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        SWITCH_MUX_CHANNEL(i);
        delay_cycles(640U);          /* 模拟开关稳定 */
        analog_[i] = read_adc_avg_(); /* 8 采样取均值 */

        /* 动态追踪黑/白基准（运行中适应光照变化）*/
        if (analog_[i] < cal_black_[i]) {
            cal_black_[i] = analog_[i];
            /* 重新计算该通道系数 */
            int32_t span = (int32_t)cal_white_[i] - (int32_t)cal_black_[i];
            norm_factor_[i] = (span > 0)
                ? (float)ADC_MAX_VALUE / (float)span
                : 0.0f;
        }
        if (analog_[i] > cal_white_[i]) {
            cal_white_[i] = analog_[i];
            int32_t span = (int32_t)cal_white_[i] - (int32_t)cal_black_[i];
            norm_factor_[i] = (span > 0)
                ? (float)ADC_MAX_VALUE / (float)span
                : 0.0f;
        }
    }

    /* ---- 2. 二值化处理（参照 convertAnalogToDigital）---- */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (analog_[i] > gray_white_[i]) {
            digital_ |= (1U << i);   /* 白（地板）→ bit 置 1 */
        } else if (analog_[i] < gray_black_[i]) {
            digital_ &= ~(1U << i);  /* 黑（线）→ bit 清 0 */
        }
        /* 中间灰度 → 保持上一状态（迟滞特性）*/
    }

    /* ---- 3. 归一化处理（参照 normalizeAnalogValues）---- */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (analog_[i] < cal_black_[i]) {
            normal_[i] = 0;
        } else if (norm_factor_[i] == 0.0f) {
            normal_[i] = 0;
        } else {
            uint16_t n = (uint16_t)((float)(analog_[i] - cal_black_[i])
                                    * norm_factor_[i]);
            normal_[i] = (n > ADC_MAX_VALUE) ? ADC_MAX_VALUE : n;
        }
    }

    /* ---- 4. 加权平均计算线偏差 ---- */
    int32_t  weighted_sum = 0;
    uint32_t normal_sum   = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        /* 用归一化值的"暗度补数"做加权（暗 = 线上）*/
        uint16_t dark = ADC_MAX_VALUE - normal_[i];
        weighted_sum += (int32_t)dark * g_sensor_position[i];
        normal_sum   += dark;
    }

    if (normal_sum < LINE_PRESENT_MIN) {
        error_      = 0;
        line_found_ = false;
        return false;
    }

    error_      = weighted_sum / (int32_t)normal_sum;
    line_found_ = true;
    return true;
}

/* ============================================================
 *  用户数据获取接口
 * ============================================================ */

bool LineSensor::get_normalized(uint16_t *result) const
{
    if (!ok_) {
        return false;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        result[i] = normal_[i];
    }
    return true;
}

bool LineSensor::get_analog(uint16_t *result) const
{
    if (!ok_) {
        return false;
    }
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        result[i] = analog_[i];
    }
    return true;
}

/* ============================================================
 *  私有 — ADC 读取（8 采样均值滤波，参照 Get_Analog_value）
 * ============================================================ */

uint16_t LineSensor::read_adc_avg_()
{
    uint32_t sum = 0;

    for (uint8_t j = 0; j < ADC_SAMPLES_PER_CHANNEL; j++) {
        g_adc_done = false;
        DL_ADC12_startConversion(LINE_ADC_INST);

        while (!g_adc_done) {
            __WFE();
        }

        sum += DL_ADC12_getMemResult(LINE_ADC_INST, DL_ADC12_MEM_IDX_0);
        DL_ADC12_enableConversions(LINE_ADC_INST);
    }

    return (uint16_t)(sum / ADC_SAMPLES_PER_CHANNEL);
}

/*
 * ADC ISR
 *
 * 只置标志位，不做浮点/阻塞。
 * 必须用 extern "C" 确保 C 链接——name mangling 会导致向量表找不到此函数，
 * 使用弱定义的空 handler，ADC 中断永远不会触发。
 */
extern "C" void LINE_ADC_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(LINE_ADC_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            g_adc_done = true;
            break;
        default:
            break;
    }
}
