/**
* 2006pid.c
 *
 * 简洁、自包含的PID实现，支持按功能使能标志和实例参数独立配置。
 * 本文件实现了2006pid.h中声明的接口：
 *
 *   void PID_Init(PID_t* pid, float kp, float ki, float kd, float max_output);
 *   void PID_SetFeedforward(PID_t* pid, float kff, float kaff);
 *   void PID_ConfigFeatures(PID_t* pid, bool enable_kp, bool enable_ki, bool enable_kd,
 *                           bool enable_kff, bool enable_kaff);
 *   void PID_SetParams(PID_t* pid, float deadband, float integral_limit);
 *   float PID_Calc(PID_t* pid, float ref, float feedback, float dt);
 *
 * 行为说明：
 *  - 默认配置保持原有行为：所有功能启用，
 *    默认死区4.0f，integral_limit为0时使用output_max * 2.0f作为积分限幅
 *  - 每个PID_t实例都有自己的标志位和参数，因此位置环和速度环可以独立配置
 *  - PID_Calc函数会遵循功能标志并使用实例参数
 *
 * 注意事项：
 *  - 本实现特意保留了原有代码中"通过检查饱和方向实现抗积分饱和"的行为
 *  - 不改变任何外部API签名
 */

#include "2006pid.h"

#ifndef PID_ERROR_DEADBAND
#define PID_ERROR_DEADBAND 4.0f // 默认误差死区值
#endif

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_output 输出最大值（限幅）
 */

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    if (pid == NULL) return;

    /* 增益参数 */
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    /* 前馈参数默认值 */
    pid->Kff = 0.0f;
    pid->Kaff = 0.0f;

    /* 状态变量初始化 */
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_ref = 0.0f;
    pid->output = 0.0f;
    pid->output_max = max_output;

    /* 功能标志：默认全部启用以保持向后兼容性 */
    pid->enable_kp   = true;
    pid->enable_ki   = true;
    pid->enable_kd   = true;
    pid->enable_kff  = true;
    pid->enable_kaff = true;

    /* 实例特定参数 */
    pid->deadband = PID_ERROR_DEADBAND;
    pid->integral_limit = 0.0f; /* 0表示使用默认值(output_max * 2.0f) */
}

void PID_SetFeedforward(PID_t *pid, float kff, float kaff)
{
    if (pid == NULL) return;
    pid->Kff  = kff;
    pid->Kaff = kaff;
}

void PID_ConfigFeatures(PID_t* pid,
                        bool enable_kp,
                        bool enable_ki,
                        bool enable_kd,
                        bool enable_kff,
                        bool enable_kaff)
{
    if (pid == NULL) return;
    pid->enable_kp   = enable_kp;
    pid->enable_ki   = enable_ki;
    pid->enable_kd   = enable_kd;
    pid->enable_kff  = enable_kff;
    pid->enable_kaff = enable_kaff;
}

void PID_SetParams(PID_t* pid, float deadband, float integral_limit)
{
    if (pid == NULL) return;

    if (deadband > 0.0f) {
        pid->deadband = deadband;
    } else {
        pid->deadband = PID_ERROR_DEADBAND;
    }

    /* 积分限幅值：如果<=0，则视为"使用默认值" */
    pid->integral_limit = integral_limit;
}

float PID_Calc(PID_t *pid, float ref, float feedback, float dt)
{
    if (pid == NULL) return 0.0f; // 空指针检查，返回0

    /* 计算误差 */
    float error = ref - feedback;

    /* 确定有效的死区值（实例参数或默认值） */
    float deadband = (pid->deadband > 0.0f) ? pid->deadband : PID_ERROR_DEADBAND;

    /* 死区处理：小误差视为零以抑制噪声引起的输出抖动 */
    if (fabsf(error) < deadband) {
        error = 0.0f;
        /* 重置上一次误差，避免离开死区时微分项突变 */
        pid->last_error = 0.0f;
    }

    /* 比例项计算（遵循功能标志） */
    float p_term = 0.0f;
    if (pid->enable_kp) {
        p_term = pid->Kp * error;
    }

    /* 微分项计算（遵循功能标志）。使用简单离散微分：
       微分 = 当前误差 - 上次误差。注意：微分计算后更新last_error */
    float derivative = error - pid->last_error;
    float d_term = 0.0f;
    if (pid->enable_kd) {
        d_term = pid->Kd * derivative;
    }

    /* 前馈项计算（遵循功能标志） */
    float ff_term = 0.0f;
    if (pid->enable_kff) {
        ff_term = pid->Kff * ref;
    }

    float accel_term = 0.0f;
    if (pid->enable_kaff) {
        /* 如果dt有效，计算加速度相关项 */
        if (dt > 0.000001f) {
            float acceleration = (ref - pid->last_ref) / dt;
            accel_term = pid->Kaff * acceleration;
        } else {
            accel_term = 0.0f;
        }
    }

    /* 更新last_ref供下一次计算使用 */
    pid->last_ref = ref;

    /* 积分项贡献（仅用于计算输出；实际的积分项可能在抗积分饱和检查后更新） */
    float integral_term = pid->enable_ki ? (pid->Ki * pid->integral) : 0.0f;

    /* 使用当前积分值的初步输出（用于抗积分饱和判断） */
    float current_output = p_term + integral_term + d_term + ff_term + accel_term;

    /* 抗积分饱和：判断增加误差到积分项是否会导致或延长饱和
       保持原有行为：未饱和时允许积分，或当积分有助于退出饱和时允许积分 */
    int should_update_integral = 0;

    if (fabsf(current_output) <= pid->output_max) {
        /* 当前未饱和：检查积分项增加误差后的潜在输出 */
        float potential_integral = pid->integral + error;
        float potential_integral_term = pid->enable_ki ? (pid->Ki * potential_integral) : 0.0f;
        float potential_output = p_term + potential_integral_term + d_term + ff_term + accel_term;

        if (fabsf(potential_output) <= pid->output_max) {
            /* 安全，可以积分 */
            should_update_integral = 1;
        } else {
            /* 如果潜在输出超过饱和值，但误差方向与饱和方向相反，
               允许积分以帮助退出饱和 */
            if ((potential_output > pid->output_max && error < 0.0f) ||
                (potential_output < -pid->output_max && error > 0.0f)) {
                should_update_integral = 1;
            } else {
                should_update_integral = 0;
            }
        }
    } else {
        /* 当前已饱和：只有当误差有助于将输出拉回正常范围时才积分 */
        if ((current_output > pid->output_max && error < 0.0f) ||
            (current_output < -pid->output_max && error > 0.0f)) {
            should_update_integral = 1;
        } else {
            should_update_integral = 0;
        }
    }

    /* 如果积分项使能且允许更新，则更新积分项 */
    if (pid->enable_ki && should_update_integral) {
        pid->integral += error;
    }

    /* 积分限幅：使用实例特定限幅或基于output_max的默认限幅 */
    float integral_max = (pid->integral_limit > 0.000001f) ? pid->integral_limit : (pid->output_max * 2.0f);    //如为0则改为2.0比例
    if (pid->integral > integral_max) {
        pid->integral = integral_max;   // 上限限幅
    } else if (pid->integral < -integral_max) {
        pid->integral = -integral_max;  // 下限限幅
    }

    /* 更新和限幅后重新计算积分项贡献 */
    integral_term = pid->enable_ki ? (pid->Ki * pid->integral) : 0.0f;

    /* 最终输出组合 */
    pid->output = p_term + integral_term + d_term + ff_term + accel_term;

    /* 输出限幅 */
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }

    /* 更新last_error供下一次微分计算（在计算完成后更新以匹配之前的行为） */
    pid->last_error = error;

    return pid->output;
}