/*
 * line_sensor.cpp — 8 路循迹传感器模块实现
 *
 * 硬件: CD4051 模拟开关 + ADC0_CH3
 *
 * 参考: 感为无MCU灰度传感器例程 (No_Mcu_Ganv_Grayscale_Sensor.c/.h)
 *   - Get_Analog_value:  8 采样均值滤波 + 通道切换
 *   - convertAnalogToDigital: 二值化（1:2 / 2:1 灰度阈值）
 *   - normalizeAnalogValues:  预计算系数 × (raw - black)
 *   - No_Mcu_Ganv_Sensor_Init: 校准参数计算 + 系数预计算
 *
 * 依赖: config.hpp（GPIO 宏）、iinterfaces.hpp（接口）
 *
 * 已知限制:
 *   1. CD4051 8 路复用，MUX 切换仍由 CPU 控制；DMA 负责同一通道 burst 搬运。
 *   2. 当前为 burst DMA：每路启动 1 次 DMA，连续搬 8 个样本，仅 1 次中断。
 *   3. 依赖 LINE_ADC 配置为 repeat-single-channel 模式；若配置不对会只采 1 次。
 *   4. 已移除运行中动态学习；黑/白基准由 PB21 按键触发上电双标定确定。
 *   5. PA2 接有源蜂鸣器，高电平响，用于标定步骤提示 / 丢线报警。
 *   6. digital_ bit=1 表示黑线，供 main.cpp 直角弯状态机使用。
 *   7. DMA 等待带有限超时，异常时返回丢线并在标定阶段要求重新采集。
 */

#include "modules/line_sensor/line_sensor.hpp"
#include "modules/common/buzzer.hpp"

/* 全局单例 */
LineSensor g_line_sensor;

/* DMA burst 传输缓冲与完成标志 (ISR 置位 → 主循环轮询)
 *   buf 由 DMA 引擎异步写入, 须声明为 volatile 防止编译器缓存 */
static volatile uint16_t g_adc_dma_buf[ADC_SAMPLES_PER_CHANNEL];
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
    ok_ = false;

    /* 手册规定正常情况下白大黑小；反极性表示过曝或标定顺序错误。 */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (white[i] <= black[i] ||
            (uint16_t)(white[i] - black[i]) < MIN_CALIBRATION_SPAN) {
            return;
        }
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        uint16_t w = white[i];
        uint16_t b = black[i];

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

    /*
     * 标定对称性自检: 左右 4 路 span (white-black) 的均值差异超过 15%
     * 表示传感器安装高度不一致或标定环境不均匀, 拒收本次标定。
     * 左右不对称会导致"线居中时 error≠0", 是"线永远偏一侧"的主要物理根因之一。
     */
    float left_span_avg  = 0.0f;
    float right_span_avg = 0.0f;
    for (uint8_t i = 0; i < SENSOR_COUNT / 2U; i++) {
        left_span_avg  += (float)((int32_t)cal_white_[i]        - (int32_t)cal_black_[i]);
        right_span_avg += (float)((int32_t)cal_white_[i + 4U]   - (int32_t)cal_black_[i + 4U]);
    }
    left_span_avg  /= (float)(SENSOR_COUNT / 2U);
    right_span_avg /= (float)(SENSOR_COUNT / 2U);

    float span_asymmetry;
    if (left_span_avg + right_span_avg > 0.0f) {
        span_asymmetry = (left_span_avg - right_span_avg)
                       / ((left_span_avg + right_span_avg) * 0.5f);
    } else {
        span_asymmetry = 0.0f;
    }

    if (span_asymmetry >  0.15f ||
        span_asymmetry < -0.15f) {
        /* 左右不对称 >15%, 标定无效 */
        return;
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
    delay_cycles(20U * (CPUCLK_FREQ / 1000U));

    /* 等待释放 */
    while ((DL_GPIO_readPins(KEY_USER_PORT, KEY_USER_PIN) & KEY_USER_PIN) == 0) {
        __NOP();
    }

    delay_cycles(20U * (CPUCLK_FREQ / 1000U));
}

/* 标定错误报警: 3 声短促蜂鸣 (150ms 响 / 100ms 间隔) */
static void signal_calibration_error_()
{
    for (uint8_t i = 0; i < 3U; i++) {
        buzzer_beep(150U);
        delay_cycles(100U * (CPUCLK_FREQ / 1000U));
    }
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
    /* 最多重试 MAX_CALIBRATION_RETRIES 次, 防止传感器硬件故障永久阻塞启动。
     * 超过重试次数后退出 (ok_ 保持 false), 主循环检测不到线将停车报警。 */
    uint8_t retry = 0;
    while (!ok_ && retry < MAX_CALIBRATION_RETRIES) {
        uint32_t sum[SENSOR_COUNT] = {0};
        uint16_t white[SENSOR_COUNT] = {0};
        uint16_t black[SENSOR_COUNT] = {0};
        bool capture_ok = true;

        /* ---- 白底标定 ---- */
        wait_key_press_();
        buzzer_beep(100U);

        for (uint16_t scan = 0;
             scan < WHITE_STARTUP_SCANS && capture_ok; scan++) {
            for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
                uint16_t sample;
                if (!read_adc_burst_dma_(i, &sample)) {
                    capture_ok = false;
                    break;
                }
                sum[i] += sample;
            }
        }
        if (!capture_ok) {
            signal_calibration_error_();
            retry++;
            continue;
        }

        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            white[i] = (uint16_t)(sum[i] / WHITE_STARTUP_SCANS);
            sum[i] = 0;
        }
        buzzer_beep(50U);

        /* ---- 黑线标定 ---- */
        wait_key_press_();
        buzzer_beep(100U);

        for (uint16_t scan = 0;
             scan < WHITE_STARTUP_SCANS && capture_ok; scan++) {
            for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
                uint16_t sample;
                if (!read_adc_burst_dma_(i, &sample)) {
                    capture_ok = false;
                    break;
                }
                sum[i] += sample;
            }
        }
        if (!capture_ok) {
            signal_calibration_error_();
            retry++;
            continue;
        }

        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            black[i] = (uint16_t)(sum[i] / WHITE_STARTUP_SCANS);
        }

        init_with_calibration(white, black);
        if (!ok_) {
            signal_calibration_error_();
            retry++;
        }
    }

    if (ok_) {
        /* 标定成功: 两声短鸣 */
        buzzer_beep(50U);
        delay_cycles(100U * (CPUCLK_FREQ / 1000U));
        buzzer_beep(50U);
    } else {
        /* 标定失败: 长鸣报警, 进入降级模式 */
        buzzer_beep(500U);
    }
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
    /* 标定未完成时拒绝更新: 归一化和位置计算依赖 cal_black_/cal_white_,
     * 未标定时 error_=0 → 车直冲，故直接拒绝。*/
    if (!ok_) {
        digital_    = 0;
        error_      = 0;
        line_found_ = false;
        return false;
    }

    /* ---- 1. 采集 8 通道模拟量（均值滤波）---- */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!read_adc_burst_dma_(i, &analog_[i])) {
            digital_    = 0;
            error_      = 0;
            line_found_ = false;
            return false;
        }

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

    /* ---- 3. 归一化处理 (移除 cal 边界硬限幅, 保留 [0,4095] 限幅) ----
     *   软限幅仅防 uint16_t 溢出。
     *   normal_[i] 可在 [0, 4095] 内连续变化,
     *   线与白的过渡区产生中间灰度 → 子传感器插值。 */
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        float scaled = (float)((int32_t)analog_[i] - (int32_t)cal_black_[i])
                       * norm_factor_[i];
        if (scaled < 0.0f)        scaled = 0.0f;
        if (scaled > 4095.0f)     scaled = 4095.0f;
        normal_[i] = (uint16_t)scaled;
    }

    /* ---- 4. 加权平均计算线偏差 ---- */
    int32_t weighted_sum = 0;
    int32_t normal_sum   = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
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
 * ============================================================ */

bool LineSensor::read_adc_burst_dma_(uint8_t ch, uint16_t *result)
{
    if (result == nullptr) {
        return false;
    }

    DL_ADC12_stopConversion(LINE_ADC_INST);
    DL_DMA_disableChannel(DMA, LINE_DMA_CHAN_ID);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    g_dma_done = false;

    /* 切换 CD4051 并等待模拟开关稳定 (~20 µs @32 MHz) */
    SWITCH_MUX_CHANNEL(ch);
    delay_cycles(640U);

    /*
     * 配置 DMA burst 传输：
     *   源地址：ADC12 MEM0 结果寄存器（固定）
     *   目标地址：g_adc_dma_buf（递增）
     *   长度：ADC_SAMPLES_PER_CHANNEL 个半字
     */
    DL_DMA_setSrcAddr(DMA, LINE_DMA_CHAN_ID,
        (uint32_t)DL_ADC12_getMemResultAddress(LINE_ADC_INST, DL_ADC12_MEM_IDX_0));
    DL_DMA_setDestAddr(DMA, LINE_DMA_CHAN_ID,
        (uint32_t)g_adc_dma_buf);
    DL_DMA_setTransferSize(DMA, LINE_DMA_CHAN_ID, ADC_SAMPLES_PER_CHANNEL);

    DL_DMA_enableChannel(DMA, LINE_DMA_CHAN_ID);

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
    uint32_t timeout = ADC_DMA_TIMEOUT_LOOPS;
    while (!g_dma_done && timeout > 0U) {
        timeout--;
        __NOP();
    }

    if (!g_dma_done) {
        DL_ADC12_stopConversion(LINE_ADC_INST);
        DL_DMA_disableChannel(DMA, LINE_DMA_CHAN_ID);
        DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
        *result = 0;
        return false;
    }

    /* 求均值 */
    uint32_t sum = 0;
    for (uint8_t j = 0; j < ADC_SAMPLES_PER_CHANNEL; j++) {
        sum += g_adc_dma_buf[j];
    }
    *result = (uint16_t)(sum / ADC_SAMPLES_PER_CHANNEL);
    return true;
}

/*
 * DMA ISR
 *
 * 只在 8 次采样全部搬运完成后触发一次。
 * 必须用 extern "C" 确保 C 链接。
 */
extern "C" void DMA_IRQHandler(void)
{
    switch (DL_DMA_getPendingInterrupt(DMA)) {
        case DL_DMA_EVENT_IIDX_DMACH0:
            DL_DMA_disableChannel(DMA, LINE_DMA_CHAN_ID);
            DL_ADC12_stopConversion(LINE_ADC_INST);
            /* 内存屏障: 确保 DMA 写入在 g_dma_done 置位前对 CPU 可见
             * (Cortex-M0+ 无数据缓存, DSB 保证总线写入顺序) */
            __DSB();
            g_dma_done = true;
            break;
        default:
            break;
    }
}
