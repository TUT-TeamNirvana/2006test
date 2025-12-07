//
// Created by 17087 on 25-11-2.
//
#include "2006pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->output = 0;
    pid->output_max = max_output;
}

float PID_Calc(PID_t *pid, float ref, float feedback)
{
    float error = ref - feedback;
    
    // 计算比例项
    float p_term = pid->Kp * error;
    
    // 计算微分项
    float derivative = error - pid->last_error;
    float d_term = pid->Kd * derivative;
    pid->last_error = error;
    
    // 计算未限幅的输出（使用当前积分项）
    float output_unlimited = p_term + pid->Ki * pid->integral + d_term;
    
    // 抗积分饱和：在更新积分项之前判断输出是否会饱和
    // 只有在输出未饱和时才累加积分项
    if (output_unlimited <= pid->output_max && output_unlimited >= -pid->output_max)
    {
        // 输出未饱和，正常累加积分项
        pid->integral += error;
    }
    // 如果输出已饱和，则不累加积分项（抗积分饱和）
    
    // 重新计算输出（使用更新后的积分项）
    pid->output = p_term + pid->Ki * pid->integral + d_term;
    
    // 限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}