/**
 * 2006pid.c
 *
 * 带实例特定功能标志和参数的PID实现。
 * 此版本实现了死区标记：当瞬时误差在死区内时，
 * 我们将误差视为0用于控制，但不将`last_error`置零。
 * 只有在死区外才计算微分项并更新`last_error`。
 *
 * 这避免了因在进入死区时将last_error置零而导致的微分冲击，
 * 同时仍能抑制死区内的噪声驱动输出。
 */

#include "2006pid.h"
#include <stddef.h>

#ifndef PID_ERROR_DEADBAND
#define PID_ERROR_DEADBAND 4.0f // 默认误差死区值
#endif

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    if (pid == NULL) return;    // 空指针检查，返回0

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

    pid->integral_limit = integral_limit;
}

float PID_Calc(PID_t *pid, float ref, float feedback, float dt)
{
    if (pid == NULL) return 0.0f;

    /* 计算误差 */
    float error = ref - feedback;

    /* 确定有效的死区值（实例参数或默认值） */
    float deadband = (pid->deadband > 0.0f) ? pid->deadband : PID_ERROR_DEADBAND;

    /* 死区标记：判断是否在死区内。
       如果在死区内，将误差视为0用于控制计算，但
       不重置 pid->last_error。同时，在死区内不计算/更新微分项，
       以避免离开死区时的微分冲击。 */
    bool in_deadband = (fabsf(error) < deadband);
    if (in_deadband) {
        /* 将误差视为0用于控制项，但保持last_error不变 */
        error = 0.0f;
    }

    /* 比例项计算（遵循功能标志） */
    float p_term = 0.0f;
    if (pid->enable_kp) {
        p_term = pid->Kp * error;
    }

    /* 微分项：只在死区外计算。
       这避免了如果之前强行将last_error设为零时，
       离开死区时产生大的微分冲击。 */
    float derivative = 0.0f;
    float d_term = 0.0f;
    if (!in_deadband) {
        derivative = error - pid->last_error;
        if (pid->enable_kd) {
            d_term = pid->Kd * derivative;
        }
    } else {
        /* 在死区内 => 微分项贡献为零 */
        derivative = 0.0f;
        d_term = 0.0f;
    }

    /* 前馈项 */
    float ff_term = 0.0f;
    if (pid->enable_kff) {
        ff_term = pid->Kff * ref;
    }

    float accel_term = 0.0f;
    if (pid->enable_kaff) {
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
        float potential_integral = pid->integral + error;
        float potential_integral_term = pid->enable_ki ? (pid->Ki * potential_integral) : 0.0f;
        float potential_output = p_term + potential_integral_term + d_term + ff_term + accel_term;

        if (fabsf(potential_output) <= pid->output_max) {
            should_update_integral = 1;
        } else {
            if ((potential_output > pid->output_max && error < 0.0f) ||
                (potential_output < -pid->output_max && error > 0.0f)) {
                should_update_integral = 1;
            } else {
                should_update_integral = 0;
            }
        }
    } else {
        if ((current_output > pid->output_max && error < 0.0f) ||
            (current_output < -pid->output_max && error > 0.0f)) {
            should_update_integral = 1;
        } else {
            should_update_integral = 0;
        }
    }

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

    /* 只有在死区外才更新last_error（这样下一次的微分是相对于
       最后一次有意义的误差）。这避免了离开死区时的微分冲击。 */
    if (!in_deadband) {
        pid->last_error = error;
    }

    return pid->output;
}