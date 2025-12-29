/**
 ******************************************************************************
 * @file    2006pid.c
 * @brief   可配置模块化PID控制器实现
 * @note    保持原有计算逻辑，特别是抗积分饱和机制
 ******************************************************************************
 */

#include "2006pid.h"
#include <math.h>
#include <string.h>

// ==================== 辅助函数 ====================

/**
 * @brief 限幅函数
 */
static inline float clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// ==================== 初始化函数 ====================

/**
 * @brief 初始化PID控制器（基本参数）
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    // 设置PID参数
    pid->params.Kp = kp;
    pid->params.Ki = ki;
    pid->params.Kd = kd;
    
    // 初始化配置为默认值
    PID_ConfigDefault(pid);
    
    // 清零状态
    PID_Reset(pid);
}

/**
 * @brief 初始化PID配置为默认值（所有功能禁用）
 */
void PID_ConfigDefault(PID_t *pid)
{
    memset(&pid->config, 0, sizeof(PID_Config_t));
    
    // 默认值设置
    pid->config.enable_feedforward = false;
    pid->config.Kff = 0.0f;
    
    pid->config.enable_accel_feedforward = false;
    pid->config.Kaff = 0.0f;
    
    pid->config.enable_integral_limit = false;
    pid->config.integral_max = 0.0f;
    pid->config.integral_min = 0.0f;
    
    pid->config.enable_derivative_filter = false;
    pid->config.derivative_filter_coef = 0.0f;
    
    pid->config.enable_output_limit = false;
    pid->config.output_max = 0.0f;
    pid->config.output_min = 0.0f;
    
    pid->config.enable_anti_windup = false;
    
    pid->config.enable_error_deadband = false;
    pid->config.error_deadband = 0.0f;
}

// ==================== 功能配置函数 ====================

/**
 * @brief 配置速度前馈
 */
void PID_ConfigFeedforward(PID_t *pid, bool enable, float kff)
{
    pid->config.enable_feedforward = enable;
    pid->config.Kff = kff;
}

/**
 * @brief 配置加速度前馈
 */
void PID_ConfigAccelFeedforward(PID_t *pid, bool enable, float kaff)
{
    pid->config.enable_accel_feedforward = enable;
    pid->config.Kaff = kaff;
}

/**
 * @brief 配置积分限幅
 */
void PID_ConfigIntegralLimit(PID_t *pid, bool enable, float max, float min)
{
    pid->config.enable_integral_limit = enable;
    pid->config.integral_max = max;
    pid->config.integral_min = min;
}

/**
 * @brief 配置微分滤波
 */
void PID_ConfigDerivativeFilter(PID_t *pid, bool enable, float coef)
{
    pid->config.enable_derivative_filter = enable;
    pid->config.derivative_filter_coef = clamp(coef, 0.0f, 1.0f);
}

/**
 * @brief 配置输出限幅
 */
void PID_ConfigOutputLimit(PID_t *pid, bool enable, float max, float min)
{
    pid->config.enable_output_limit = enable;
    pid->config.output_max = max;
    pid->config.output_min = min;
}

/**
 * @brief 配置抗积分饱和
 */
void PID_ConfigAntiWindup(PID_t *pid, bool enable)
{
    pid->config.enable_anti_windup = enable;
}

/**
 * @brief 配置误差死区
 */
void PID_ConfigErrorDeadband(PID_t *pid, bool enable, float deadband)
{
    pid->config.enable_error_deadband = enable;
    pid->config.error_deadband = fabsf(deadband);  // 确保为正值
}

// ==================== 计算和控制函数 ====================

/**
 * @brief PID计算
 * @note  保持原有计算逻辑，特别是抗积分饱和机制
 */
float PID_Calc(PID_t *pid, float ref, float feedback, float dt)
{
    float error = ref - feedback;
    
    // ===== 1. 误差死区处理 =====
    if (pid->config.enable_error_deadband)
    {
        if (fabsf(error) < pid->config.error_deadband)
        {
            error = 0.0f;
            pid->state.last_error = 0.0f;  // 重置，避免离开死区时微分突变
        }
    }
    
    // ===== 2. 计算比例项 =====
    float p_term = pid->params.Kp * error;
    
    // ===== 3. 计算微分项 =====
    float derivative = error - pid->state.last_error;
    
    // 微分滤波（可选）
    if (pid->config.enable_derivative_filter)
    {
        // 一阶低通滤波: d_filtered = α * d_new + (1-α) * d_old
        derivative = (1.0f - pid->config.derivative_filter_coef) * derivative + 
                     pid->config.derivative_filter_coef * pid->state.last_derivative;
        pid->state.last_derivative = derivative;
    }
    
    float d_term = pid->params.Kd * derivative;
    pid->state.last_error = error;
    
    // ===== 4. 前馈项计算 =====
    float feedforward_term = 0.0f;
    float accel_feedforward_term = 0.0f;
    
    // 速度前馈（可选）
    if (pid->config.enable_feedforward)
    {
        feedforward_term = pid->config.Kff * ref;
    }
    
    // 加速度前馈（可选）
    if (pid->config.enable_accel_feedforward)
    {
        float acceleration = 0.0f;
        if (dt > 0.0001f)  // 避免除零
        {
            acceleration = (ref - pid->state.last_ref) / dt;
        }
        accel_feedforward_term = pid->config.Kaff * acceleration;
    }
    pid->state.last_ref = ref;  // 更新上一次目标值
    
    // ===== 5. 抗积分饱和逻辑（保持原有逻辑） =====
    int should_update_integral = 0;
    
    if (pid->config.enable_anti_windup && pid->config.enable_output_limit)
    {
        // 使用原有的抗积分饱和算法
        // 1. 先计算当前输出（使用当前积分项 + 前馈项）
        float current_output = p_term + pid->params.Ki * pid->state.integral + d_term + 
                              feedforward_term + accel_feedforward_term;
        
        // 2. 判断是否应该更新积分项
        float output_max = pid->config.output_max;
        float output_min = pid->config.output_min;
        
        // 判断当前输出是否已饱和
        if (current_output <= output_max && current_output >= output_min)
        {
            // 当前输出未饱和
            // 计算加上error后的积分项和输出（包含前馈）
            float potential_integral = pid->state.integral + error;
            float potential_output = p_term + pid->params.Ki * potential_integral + d_term + 
                                    feedforward_term + accel_feedforward_term;
            
            // 如果加上error后不会饱和，正常积分
            if (potential_output <= output_max && potential_output >= output_min)
            {
                should_update_integral = 1;
            }
            // 如果加上error后会饱和，判断error方向
            else
            {
                // 如果error方向与饱和方向相反，允许积分（帮助退出饱和）
                if ((potential_output > output_max && error < 0) ||
                    (potential_output < output_min && error > 0))
                {
                    should_update_integral = 1;
                }
                // 如果error方向与饱和方向相同，不积分（防止继续饱和）
            }
        }
        else
        {
            // 当前输出已饱和
            // 如果error方向与饱和方向相反，允许积分（帮助退出饱和）
            if ((current_output > output_max && error < 0) ||
                (current_output < output_min && error > 0))
            {
                should_update_integral = 1;
            }
            // 如果error方向与饱和方向相同，不积分（防止继续饱和）
        }
    }
    else
    {
        // 未启用抗积分饱和，正常积分
        should_update_integral = 1;
    }
    
    // ===== 6. 更新积分项 =====
    if (should_update_integral)
    {
        pid->state.integral += error;
    }
    
    // ===== 7. 积分限幅（可选，独立于抗积分饱和） =====
    if (pid->config.enable_integral_limit)
    {
        pid->state.integral = clamp(pid->state.integral, 
                                   pid->config.integral_min, 
                                   pid->config.integral_max);
    }
    
    // ===== 8. 计算最终输出 =====
    pid->state.output = p_term + pid->params.Ki * pid->state.integral + d_term + 
                       feedforward_term + accel_feedforward_term;
    
    // ===== 9. 输出限幅（可选） =====
    if (pid->config.enable_output_limit)
    {
        pid->state.output = clamp(pid->state.output, 
                                 pid->config.output_min, 
                                 pid->config.output_max);
    }
    
    return pid->state.output;
}

/**
 * @brief 复位PID状态
 */
void PID_Reset(PID_t *pid)
{
    pid->state.integral = 0.0f;
    pid->state.last_error = 0.0f;
    pid->state.last_ref = 0.0f;
    pid->state.last_derivative = 0.0f;
    pid->state.output = 0.0f;
}