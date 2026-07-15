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
 *   1. CD4051 8 路复用，MUX 切换仍由 CPU 控制；DMA 负责同一通道 burst 搬运。
 *   2. 当前为 burst DMA：每路启动 1 次 DMA，连续搬 8 个样本，仅 1 次中断。
 *   3. 依赖 LINE_ADC 配置为 repeat-single-channel 模式；若配置不对会只采 1 次。
 *   4. 已移除运行中动态学习；黑/白基准由 PB21 按键触发上电双标定确定。
 *   5. PA0 接有源蜂鸣器，高电平响，用于标定步骤提示 / 丢线报警。
 *   6. digital_ bit=1 表示黑线，供 main.cpp 直角弯状态机使用。
 *   7. 若 DMA/ADC/GPIO API 宏名/函数名与 SDK 版本不一致，请按 ti_msp_dl_config.h 调整。
 */

#include "modules/line_sensor/line_sensor.hpp"
#include "modules/common/buzzer.hpp"

/* 全局单例 */
LineSensor g_line_sensor;

/* DMA burst 传输缓冲与完成标志（ISR 置位 → 主循环轮询，仅本文件内可见）*/
static uint16_t      g_adc_dma_buf[ADC_SAMPLES_PER_CHANNEL];
static volatile bool g_dma_done;

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
     * ADC、GPIO 和 DMA 已在 SYSCFG_DL_init() 中初始化。
     * 此处复位运行时状态。标定数据保留。
     */
    g_dma_done   = false;
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
         *   < gray_black → 判定为黑（线），digital_ bit 置 1
         *   > gray_white → 判定为白（地板），digital_ bit 清 0
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
 *  本地辅助：等待用户按键按下一次并释放（带消抖）
 * ============================================================ */

static void wait_key_press_()
{
    /*
     * 按键接地，上拉输入：未按下为高电平，按下为低电平。
     * 注意：此处不能用 __WFE()，因为 PB21 没有配置中断/事件；
     *       一旦进入 WFE 且无线程事件，CPU 会永久睡眠。
     */
    while ((DL_GPIO_readPins(KEY_USER_PORT, KEY_USER_PIN) & KEY_USER_PIN) != 0) {
        __NOP();
    }

    /* 消抖：等待约 20 ms */
    delay_cycles(20U * 32000U);

    /* 等待释放 */
    while ((DL_GPIO_readPins(KEY_USER_PORT, KEY_USER_PIN) & KEY_USER_PIN) == 0) {
        __NOP();
    }

    delay_cycles(20U * 32000U);
}

/* ============================================================
 *  按键触发黑白双标定（启动时调用）
 *
 *  流程：
 *    1. 上电后电机保持停止，等待用户把 8 路探头全部放到白底；
 *    2. 按一次 KEY_USER，采集 white[8]；
 *    3. 把 8 路探头全部放到黑线（或整板覆盖黑线），再按一次 KEY_USER；
 *    4. 采集 black[8]，计算阈值，标定完成。
 * ============================================================ */

void LineSensor::calibrate()
{
    uint32_t sum[SENSOR_COUNT] = {0};
    uint16_t white[SENSOR_COUNT];
    uint16_t black[SENSOR_COUNT];

    /* ---- 白底标定 ---- */
    wait_key_press_();
    buzzer_beep(100U);  /* 开始采白底 */

    for (uint16_t scan = 0; scan < WHITE_STARTUP_SCANS; scan++) {
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            sum[i] += read_adc_burst_dma_(i);
        }
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        white[i] = (uint16_t)(sum[i] / WHITE_STARTUP_SCANS);
        sum[i] = 0;  /* 清零用于黑线累加 */
    }
    buzzer_beep(50U);   /* 白底采样完成 */

    /* ---- 黑线标定 ---- */
    wait_key_press_();
    buzzer_beep(100U);  /* 开始采黑线 */

    for (uint16_t scan = 0; scan < WHITE_STARTUP_SCANS; scan++) {
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            sum[i] += read_adc_burst_dma_(i);
        }
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        black[i] = (uint16_t)(sum[i] / WHITE_STARTUP_SCANS);
    }
    buzzer_beep(50U);
    delay_cycles(100U * 32000U);
    buzzer_beep(50U);   /* 黑线采样完成：短响两声 */

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
        analog_[i] = read_adc_burst_dma_(i);

        /*
         * 已移除运行中动态学习：黑/白基准只在标定时确定。
         * 如需适应环境变化，应重新上电标定或改用固定常量。
         */
    }

    /* ---- 2. 二值化处理（参照 convertAnalogToDigital）---- */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (analog_[i] < gray_black_[i]) {
            digital_ |= (1U << i);   /* 黑（线）→ bit 置 1 */
        } else if (analog_[i] > gray_white_[i]) {
            digital_ &= ~(1U << i);  /* 白（地板）→ bit 清 0 */
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
 *  私有 — DMA burst 读取（一路 8 次采样合并为 1 次 DMA 中断）
 *
 *  依赖：LINE_ADC 在 SysConfig 中配置为 repeat-single-channel 模式，
 *        LINE_DMA 配置为 src 固定、dst 递增、传输长度 8。
 *  注意：以下 API 基于 MSPM0 DriverLib 常见命名。若编译报错，请根据
 *  你 SDK 中的 ti_msp_dl_config.h / dl_dma.h / dl_adc12.h 调整。
 * ============================================================ */

#ifndef LINE_DMA_CHANNEL
#define LINE_DMA_CHANNEL 0   /* SysConfig 里为 LINE_DMA 选的通道号 */
#endif

uint16_t LineSensor::read_adc_burst_dma_(uint8_t ch)
{
    g_dma_done = false;

    /* 切换 CD4051 并等待模拟开关稳定 */
    SWITCH_MUX_CHANNEL(ch);
    delay_cycles(640U);

    /*
     * 配置 DMA burst 传输：
     *   源地址：ADC12 MEM0 结果寄存器（固定）
     *   目标地址：g_adc_dma_buf（递增）
     *   长度：ADC_SAMPLES_PER_CHANNEL 个半字
     */
    DL_DMA_setSrcAddr(LINE_DMA_INST, LINE_DMA_CHANNEL,
        (uint32_t)DL_ADC12_getMemResultAddress(LINE_ADC_INST, DL_ADC12_MEM_IDX_0));
    DL_DMA_setDestAddr(LINE_DMA_INST, LINE_DMA_CHANNEL,
        (uint32_t)g_adc_dma_buf);
    DL_DMA_setTransferSize(LINE_DMA_INST, LINE_DMA_CHANNEL, ADC_SAMPLES_PER_CHANNEL);

    /*
     * 目标地址递增应在 SysConfig 中配置；若 DriverLib 提供运行时 API，
     * 可取消下面一行的注释（API 名可能为 DL_DMA_setDestIncrement 等）：
     *   DL_DMA_setDestIncrement(LINE_DMA_INST, LINE_DMA_CHANNEL,
     *       DL_DMA_ADDR_INCREMENT_ENABLE);
     */

    DL_DMA_enableChannel(LINE_DMA_INST, LINE_DMA_CHANNEL);

    /*
     * 启动 ADC。repeat-single 模式下 ADC 会持续转换同一通道，
     * 每次转换完成自动触发 DMA，直到 DMA 搬够 8 个样本。
     */
    DL_ADC12_startConversion(LINE_ADC_INST);

    /*
     * 等待 DMA 完成。原本用 __WFE()，但若 DMA 中断未正确配置会永久睡眠；
     * 改为轮询更保险，DMA ISR 会在完成后把 g_dma_done 置 true。
     * 功耗略有增加，但可避免硬死机。
     */
    while (!g_dma_done) {
        __NOP();
    }

    /* 求均值 */
    uint32_t sum = 0;
    for (uint8_t j = 0; j < ADC_SAMPLES_PER_CHANNEL; j++) {
        sum += g_adc_dma_buf[j];
    }
    return (uint16_t)(sum / ADC_SAMPLES_PER_CHANNEL);
}

/*
 * DMA ISR
 *
 * 只在 8 次采样全部搬运完成后触发一次。
 * 必须用 extern "C" 确保 C 链接。
 */
extern "C" void LINE_DMA_INST_IRQHandler(void)
{
    /*
     * 不同 SDK 版本获取 pending interrupt 的方式不同，常见两种：
     *   DL_DMA_getPendingInterrupt(LINE_DMA_INST)
     *   DL_DMA_getPendingInterrupt(LINE_DMA_INST, LINE_DMA_CHANNEL)
     * 请按实际头文件选择。
     */
    switch (DL_DMA_getPendingInterrupt(LINE_DMA_INST, LINE_DMA_CHANNEL)) {
        case DL_DMA_IIDX_DMA_CHANNEL0_DONE:
            g_dma_done = true;
            DL_DMA_disableChannel(LINE_DMA_INST, LINE_DMA_CHANNEL);
            /*
             * burst 完成后停止 ADC 重复转换，防止产生多余的 DMA 请求。
             * 若 DriverLib 没有 stopConversion，可改用 disableConversions。
             */
            DL_ADC12_stopConversion(LINE_ADC_INST);
            break;
        default:
            break;
    }
}
