//
// Created by 17087 on 25-11-2.
//
#include "2006pid.h"
#include <math.h>
#include <string.h>

// ==================== 私有函数声明 ====================
static int is_feature_enabled(const PID_t *pid, PIDFeature_e feature);
static float calculate_feedforward(PID_t *pid, float ref, float dt);
static float calculate_derivative(PID_t *pid, float error, float feedback);
static int should_integrate(PID_t *pid, float error, float current_output);

// ==================== 初始化函数 ====================

void PID_InitWithConfig(PID_t *pid, const PIDConfig_t *config)
{
    // 复制配置
    memcpy(&pid->config, config, sizeof(PIDConfig_t));
    
    // 初始化运行时状态
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedback = 0.0f;
    pid->last_ref = 0.0f;
    pid->output = 0.0f;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    PIDConfig_t config = {
        .Kp = kp,
        .Ki = ki,
        .Kd = kd,
        .output_max = max_output,
        .features = PID_FEATURE_NONE,  // 默认无额外功能
        .Kff = 0.0f,
        .Kaff = 0.0f,
        .integral_separation_threshold = 0.0f,
        .error_deadband = 0.0f,
        .integral_limit_factor = 2.0f,  // 默认积分限幅系数
    };
    
    PID_InitWithConfig(pid, &config);
}

void PID_SetFeedforward(PID_t *pid, float kff, float kaff)
{
    pid->config.Kff = kff;
    pid->config.Kaff = kaff;
    
    // 自动启用前馈功能
    if (kff != 0.0f)
    {
        pid->config.features |= PID_FEATURE_FEEDFORWARD;
    }
    if (kaff != 0.0f)
    {
        pid->config.features |= PID_FEATURE_ACCEL_FEEDFORWARD;
    }
}

void PID_SetFeature(PID_t *pid, PIDFeature_e feature, int enable)
{
    if (enable)
    {
        pid->config.features |= feature;
    }
    else
    {
        pid->config.features &= ~feature;
    }
}

void PID_SetIntegralSeparation(PID_t *pid, float threshold)
{
    pid->config.integral_separation_threshold = threshold;
    if (threshold > 0.0f)
    {
        pid->config.features |= PID_FEATURE_INTEGRAL_SEPARATION;
    }
}

void PID_SetErrorDeadband(PID_t *pid, float deadband)
{
    pid->config.error_deadband = deadband;
    if (deadband > 0.0f)
    {
        pid->config.features |= PID_FEATURE_ERROR_DEADBAND;
    }
}

void PID_SetIntegralLimitFactor(PID_t *pid, float factor)
{
    pid->config.integral_limit_factor = factor;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_feedback = 0.0f;
    pid->last_ref = 0.0f;
    pid->output = 0.0f;
}

// ==================== 主计算函数 ====================

float PID_Calc(PID_t *pid, float ref, float feedback, float dt)
{
    float error = ref - feedback;
    
    // ===== 1. 误差死区处理 =====
    if (is_feature_enabled(pid, PID_FEATURE_ERROR_DEADBAND))
    {
        if (fabsf(error) < pid->config.error_deadband)
        {
            error = 0.0f;
            pid->last_error = 0.0f;  // 重置，避免离开死区时微分突变
        }
    }
    
    // ===== 2. 计算比例项 =====
    float p_term = pid->config.Kp * error;
    
    // ===== 3. 计算微分项 =====
    float d_term = calculate_derivative(pid, error, feedback);
    
    // ===== 4. 计算前馈项 =====
    float feedforward_term = calculate_feedforward(pid, ref, dt);
    
    // ===== 5. 计算当前输出（用于判断是否应该积分） =====
    float current_output = p_term + pid->config.Ki * pid->integral + d_term + feedforward_term;
    
    // ===== 6. 积分分离判断 =====
    int can_integrate = 1;
    if (is_feature_enabled(pid, PID_FEATURE_INTEGRAL_SEPARATION))
    {
        if (fabsf(error) > pid->config.integral_separation_threshold)
        {
            can_integrate = 0;  // 误差过大，不积分
        }
    }
    
    // ===== 7. 抗积分饱和判断 =====
    int should_update = 0;
    if (can_integrate)
    {
        should_update = should_integrate(pid, error, current_output);
    }
    
    // ===== 8. 更新积分项 =====
    if (should_update)
    {
        pid->integral += error;
    }
    
    // ===== 9. 积分限幅 =====
    float integral_max = pid->config.output_max * pid->config.integral_limit_factor;
    if (pid->integral > integral_max)
    {
        pid->integral = integral_max;
    }
    else if (pid->integral < -integral_max)
    {
        pid->integral = -integral_max;
    }
    
    // ===== 10. 计算最终输出 =====
    pid->output = p_term + pid->config.Ki * pid->integral + d_term + feedforward_term;
    
    // ===== 11. 输出限幅 =====
    if (pid->output > pid->config.output_max)
    {
        pid->output = pid->config.output_max;
    }
    else if (pid->output < -pid->config.output_max)
    {
        pid->output = -pid->config.output_max;
    }
    
    // ===== 12. 更新历史状态 =====
    pid->last_error = error;
    pid->last_feedback = feedback;
    
    return pid->output;
}

// ==================== 私有辅助函数实现 ====================

/**
 * @brief 检查功能是否启用
 */
static int is_feature_enabled(const PID_t *pid, PIDFeature_e feature)
{
    return (pid->config.features & feature) != 0;
}

/**
 * @brief 计算前馈项
 */
static float calculate_feedforward(PID_t *pid, float ref, float dt)
{
    float ff_term = 0.0f;
    
    // 速度前馈
    if (is_feature_enabled(pid, PID_FEATURE_FEEDFORWARD))
    {
        ff_term += pid->config.Kff * ref;
    }
    
    // 加速度前馈
    if (is_feature_enabled(pid, PID_FEATURE_ACCEL_FEEDFORWARD))
    {
        float acceleration = 0.0f;
        if (dt > 0.0001f)  // 避免除零
        {
            acceleration = (ref - pid->last_ref) / dt;
        }
        ff_term += pid->config.Kaff * acceleration;
        pid->last_ref = ref;  // 更新上一次目标值
    }
    
    return ff_term;
}

/**
 * @brief 计算微分项
 */
static float calculate_derivative(PID_t *pid, float error, float feedback)
{
    float d_term = 0.0f;
    
    if (pid->config.Kd != 0.0f)
    {
        if (is_feature_enabled(pid, PID_FEATURE_DERIVATIVE_ON_MEAS))
        {
            // 微分先行：微分作用于测量值而非误差
            // 优点：避免设定值突变时的微分冲击
            float derivative = -(feedback - pid->last_feedback);
            d_term = pid->config.Kd * derivative;
        }
        else
        {
            // 标准微分：微分作用于误差
            float derivative = error - pid->last_error;
            d_term = pid->config.Kd * derivative;
        }
    }
    
    return d_term;
}

/**
 * @brief 判断是否应该更新积分项（抗积分饱和）
 */
static int should_integrate(PID_t *pid, float error, float current_output)
{
    int should_update = 0;
    
    // 计算前馈项（不重复计算，这里简化处理）
    float feedforward_term = 0.0f;
    if (is_feature_enabled(pid, PID_FEATURE_FEEDFORWARD))
    {
        feedforward_term += pid->config.Kff * pid->last_ref;
    }
    
    // 判断当前输出是否已饱和
    if (fabsf(current_output) <= pid->config.output_max)
    {
        // 当前输出未饱和
        // 计算加上error后的积分项和输出
        float p_term = pid->config.Kp * error;
        float potential_integral = pid->integral + error;
        float potential_output = p_term + pid->config.Ki * potential_integral + feedforward_term;
        
        // 如果加上error后不会饱和，正常积分
        if (fabsf(potential_output) <= pid->config.output_max)
        {
            should_update = 1;
        }
        // 如果加上error后会饱和，判断error方向
        else
        {
            // 如果error方向与饱和方向相反，允许积分（帮助退出饱和）
            if ((potential_output > pid->config.output_max && error < 0) ||
                (potential_output < -pid->config.output_max && error > 0))
            {
                should_update = 1;
            }
        }
    }
    else
    {
        // 当前输出已饱和
        // 如果error方向与饱和方向相反，允许积分（帮助退出饱和）
        if ((current_output > pid->config.output_max && error < 0) ||
            (current_output < -pid->config.output_max && error > 0))
        {
            should_update = 1;
        }
    }
    
    return should_update;
}