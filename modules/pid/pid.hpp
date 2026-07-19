/**
 * @brief       PID基础控制库(C++)
 * @details     增量式/位置式 PID 控制器。
 *              增量式内置 D 项低通滤波、输出衰减、增量步长限幅，
 *              适配无编码器的循迹小车差速转向场景。
 * @author      Haoqi Liu
 * @date        2025-1-20
 * @version     V1.3.0
 * @note
 * @warning     本 MCU (MSPM0G3507, Cortex-M0+) 无硬件 FPU，
 *              浮点运算由软浮点库模拟。高频率控制场景建议迁移至
 *              定点数 (Q16) 实现以降低 CPU 负载。
 * @par         历史版本
 *              V1.0.0 创建于 2024-9-21, 更改自 C 语言版本 PID 库
 *              V1.1.0 创建于 2025-1-20, 添加类初始化默认限幅参数
 *              V1.2.0 创建于 2025-7-16, 修复审查发现的全部问题:
 *                - 变量重命名: error→error_prev_, pre_error→error_prev2_
 *                - 成员名 type_ 避免与枚举 Type 冲突
 *                - 添加 D 项一阶低通滤波 (deriv_lpf_alpha_)
 *                - 添加输出衰减 (output_decay_) 防弯道后残留偏航
 *                - 添加增量步长限幅 (max_delta_)
 *                - 改进文档注释
 *              V1.3.0 创建于 2025-7-16, D 项改为标准二阶差分:
 *                - D 项从 LPF(e[k]-e[k-1]) 改为 LPF((e[k]-e[k-1])-(e[k-1]-e[k-2]))
 *                - 新增 error_prev2_ 成员用于存储 e[k-2]
 * */

#ifndef PID_H
#define PID_H
#include <cmath>

class PID {
public:
    enum Type {
        position_type, // 位置式 PID
        delta_type,    // 增量式 PID
    };

    /**
     * @brief 不允许默认初始化, 初始化必须要有 PID 种类, kp, ki, kd 参数
     * */
    PID() = delete;

    /**
     * @brief PID 初始化
     * @param type             PID 类型 (position_type / delta_type)
     * @param kp, ki, kd       PID 三个参数
     * @param sum_error_limit_p 积分限幅上限 (仅位置式有效, NAN=不限)
     * @param sum_error_limit_n 积分限幅下限 (仅位置式有效, NAN=不限)
     * @param output_limit_p    输出限幅上限 (NAN=不限)
     * @param output_limit_n    输出限幅下限 (NAN=不限)
     * @param deriv_lpf_alpha   导数低通滤波系数 (仅增量式有效, 0~1, 0=无滤波, 默认 0.5)
     * @param output_decay      输出衰减系数 (仅增量式有效, 0~1, 1=不衰减, 默认 0.999)
     *                         每周期 output*=decay, 消除弯道后残留偏航
     * @param max_delta         增量步长限幅 (仅增量式有效, NAN=不限幅)
     * */
    PID(const Type type, const float kp, const float ki, const float kd,
        const float sum_error_limit_p = NAN, const float sum_error_limit_n = NAN,
        const float output_limit_p = NAN, const float output_limit_n = NAN,
        const float deriv_lpf_alpha = 0.5f,
        const float output_decay    = 0.999f,
        const float max_delta       = NAN)
        : type_(type), kp(kp), ki(ki), kd(kd),
          sum_error_limit_p(sum_error_limit_p), sum_error_limit_n(sum_error_limit_n),
          output_limit_p(output_limit_p), output_limit_n(output_limit_n),
          deriv_lpf_alpha_(deriv_lpf_alpha), output_decay_(output_decay),
          max_delta_(max_delta) {}

    /**
     * @brief 设置 PID 目标值
     * @param Target 新的 PID 目标值
     * */
    void SetTarget(const float Target) { target = Target; }

    /**
     * @brief 设置积分值, 用于外部条件积分/抗饱和 (仅位置式有效)
     * @param sum_error PID 积分值
     */
    void set_sum_error(const float sum_error) { sum_error_ = sum_error; }

    /**
     * @brief 读取积分值 (仅位置式有效)
     * @return 当前积分累加值 (delta 型始终返回 0)
     */
    [[nodiscard]] float get_sum_error() const { return sum_error_; }

    /**
     * @brief 读取上一次误差 (e[k-1]), 用于外部死区判断或 D 项跟踪
     * @return 上一帧的误差值
     */
    [[nodiscard]] float get_error() const { return error_prev_; }

    /**
     * @brief 重置增量式 PID 的累积输出, 用于状态切换时清零
     */
    void reset_output() { output_ = 0.0f; }

    /**
     * @brief 热启动：将累积输出预设为 feedforward 值，
     *        避免增量式 PID 从 0 爬坡到目标值的漫长过渡期。
     */
    void warm_start(float initial_output) { output_ = initial_output; }

    /**
     * @brief 重置全部动态状态 (模式切换或重新启动时调用)
     */
    void reset() {
        error_prev_     = 0.0f;
        error_prev2_    = 0.0f;
        sum_error_      = 0.0f;
        output_         = 0.0f;
        filtered_deriv_ = 0.0f;
    }

    /**
     * @brief PID 计算
     * @param input PID 观测值 (传感器读数)
     * @return PID 计算结果
     * @note  内部使用标准 error = target - input 约定
     * */
    [[nodiscard]] float calc(float input);

    /* ---- 参数 ---- */
    float target{0};     // 目标值
    Type  type_;         // PID 种类: position_type 或 delta_type
    float kp, ki, kd;    // 比例、积分、微分系数

    /* ---- 限幅参数 (运行时可变) ---- */
    float sum_error_limit_p{NAN}; // 积分限幅上限 (仅位置式有效)
    float sum_error_limit_n{NAN}; // 积分限幅下限 (仅位置式有效)
    float output_limit_p{NAN};    // 输出限幅上限
    float output_limit_n{NAN};    // 输出限幅下限

private:
    /* ---- 运行时状态 ---- */
    float error_prev_{0};      // 上一帧误差 e[k-1]
    float error_prev2_{0};     // 上两帧误差 e[k-2] (用于二阶差分 D 项)
    float sum_error_{0};       // 累计偏差值 (仅位置式 PID 使用)
    float output_{0};          // PID 输出值 (增量式为累积输出)
    float filtered_deriv_{0};  // D 项一阶低通滤波状态

    /* ---- 配置 ---- */
    float deriv_lpf_alpha_;    // 导数低通滤波系数 (0~1, 0=无滤波)
    float output_decay_;       // 输出衰减系数 (0~1, 1=不衰减)
    float max_delta_;          // 增量步长限幅 (NAN=不限)
};

/* ============================================================
 *  calc() 内联实现
 * ============================================================ */
inline float PID::calc(const float input) {
    /* 标准 PID 误差: e = target - measurement */
    const float error_curr = target - input;

    if (type_ == position_type) {
        /*** 位置式 PID: u = Kp*e(t) + Ki*∫e(t)dt + Kd*[e(t)-e(t-1)] ***/
        sum_error_ += error_curr;

        /* 积分限幅 */
        if (!std::isnan(sum_error_limit_p) && sum_error_ >= sum_error_limit_p)
            sum_error_ = sum_error_limit_p;
        if (!std::isnan(sum_error_limit_n) && sum_error_ <= sum_error_limit_n)
            sum_error_ = sum_error_limit_n;

        output_ = kp * error_curr +
                  ki * sum_error_ +
                  kd * (error_curr - error_prev_);
        error_prev_ = error_curr;

    } else if (type_ == delta_type) {
        /*** 增量式 PID: Δu = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*LPF(Δ²e)
         *   D 项使用误差的二阶差分 (标准增量式 PID):
         *     Δ²e = (e[k]-e[k-1]) - (e[k-1]-e[k-2])
         *   再通过一阶低通滤波抑制二阶差分的量化噪声放大。 ***/

        const float raw_deriv = error_curr - error_prev_;

        /* D 项二阶差分 + LPF: 真正的"误差加速度"阻尼, 而非 P 项副本 */
        const float raw_deriv2 = raw_deriv - (error_prev_ - error_prev2_);
        filtered_deriv_ += deriv_lpf_alpha_ * (raw_deriv2 - filtered_deriv_);

        float delta = kp * raw_deriv +
                      ki * error_curr +
                      kd * filtered_deriv_;

        /* 增量步长限幅: 防止大误差时单周期饱和 */
        if (!std::isnan(max_delta_)) {
            if (delta >  max_delta_) delta =  max_delta_;
            if (delta < -max_delta_) delta = -max_delta_;
        }

        output_ += delta;

        /* 输出衰减 (泄漏积分器): 防止弯道后残留偏航
         *   衰减率恒定，时间常数 ≈ T/(1-decay).
         *   设为 1.0 时完全不衰减, 增量式 PID 的隐式积分自然维持偏置补偿. */
        output_ *= output_decay_;

        /* 状态更新: e[k-2] ← e[k-1], e[k-1] ← e[k] */
        error_prev2_ = error_prev_;
        error_prev_  = error_curr;
    }

    /* 输出限幅 (两种 PID 共用) */
    if (!std::isnan(output_limit_p) && output_ >= output_limit_p)
        output_ = output_limit_p;
    if (!std::isnan(output_limit_n) && output_ <= output_limit_n)
        output_ = output_limit_n;

    /* NaN/Inf 防护: 软浮点库或传感器故障可产生非有限值,
     * 传播到下游 round_to_i32() 将触发未定义行为。 */
    if (!std::isfinite(output_)) {
        output_ = 0.0f;
    }

    return output_;
}

#endif // PID_H
