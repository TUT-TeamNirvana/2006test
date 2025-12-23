//
// Created by 17087 on 25-11-2.
//
#include "2006pid.h"
#include <math.h>

// 误差死区，抑制小幅噪声引起的抖动（单位与反馈一致，当前为 rpm）
#define PID_ERROR_DEADBAND 4.0f

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Kff = 0.0f;   // 默认不启用前馈
    pid->Kaff = 0.0f;  // 默认不启用加速度前馈
    pid->integral = 0;
    pid->last_error = 0;
    pid->last_ref = 0.0f;
    pid->output = 0;
    pid->output_max = max_output;
}

void PID_SetFeedforward(PID_t *pid, float kff, float kaff)
{
    pid->Kff = kff;
    pid->Kaff = kaff;
}

float PID_Calc(PID_t *pid, float ref, float feedback, float dt)
{
    float error = ref - feedback;

    // 死区：小于阈值的误差视为 0，避免微小噪声驱动输出
    if (fabsf(error) < PID_ERROR_DEADBAND)
    {
        error = 0.0f;
        pid->last_error = 0.0f; // 重置，避免离开死区时微分突变
    }
    
    // 计算比例项
    float p_term = pid->Kp * error;
    
    // 计算微分项
    float derivative = error - pid->last_error;
    float d_term = pid->Kd * derivative;
    pid->last_error = error;
    
    // ===== 前馈项计算 =====
    // 1. 速度前馈：根据目标速度直接给出基础输出
    float feedforward_term = pid->Kff * ref;
    
    // 2. 加速度前馈：根据目标速度变化率补偿惯性
    float acceleration = 0.0f;
    if (dt > 0.0001f)  // 避免除零
    {
        acceleration = (ref - pid->last_ref) / dt;
    }
    float accel_feedforward_term = pid->Kaff * acceleration;
    pid->last_ref = ref;  // 更新上一次目标值
    
    // 1. 先计算当前输出（使用当前积分项 + 前馈项）
    float current_output = p_term + pid->Ki * pid->integral + d_term + feedforward_term + accel_feedforward_term;
    
    // 2. 判断是否应该更新积分项
    int should_update = 0;
    
    // 判断当前输出是否已饱和
    if (fabs(current_output) <= pid->output_max)
    {
        // 当前输出未饱和
        // 计算加上error后的积分项和输出（包含前馈）
        float potential_integral = pid->integral + error;
        float potential_output = p_term + pid->Ki * potential_integral + d_term + feedforward_term + accel_feedforward_term;
        
        // 如果加上error后不会饱和，正常积分
        if (fabs(potential_output) <= pid->output_max)
        {
            should_update = 1;
        }
        // 如果加上error后会饱和，判断error方向
        else
        {
            // 如果error方向与饱和方向相反，允许积分（帮助退出饱和）
            if ((potential_output > pid->output_max && error < 0) ||
                (potential_output < -pid->output_max && error > 0))
            {
                should_update = 1;
            }
            // 如果error方向与饱和方向相同，不积分（防止继续饱和）
        }
    }
    else
    {
        // 当前输出已饱和
        // 如果error方向与饱和方向相反，允许积分（帮助退出饱和）
        if ((current_output > pid->output_max && error < 0) ||
            (current_output < -pid->output_max && error > 0))
        {
            should_update = 1;
        }
        // 如果error方向与饱和方向相同，不积分（防止继续饱和）
    }
    
    // 3. 更新积分项
    if (should_update)
    {
        pid->integral += error;
    }
    
    // 4. 积分限幅（防止积分项过大）
    float integral_max = pid->output_max * 2.0f;
    if (pid->integral > integral_max)
    {
        pid->integral = integral_max;
    }
    else if (pid->integral < -integral_max)
    {
        pid->integral = -integral_max;
    }
    
    // 5. 重新计算输出（使用更新并限幅后的积分项 + 前馈项）
    pid->output = p_term + pid->Ki * pid->integral + d_term + feedforward_term + accel_feedforward_term;
    
    // 6. 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}