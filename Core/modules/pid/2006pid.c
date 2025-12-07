//
// Created by 17087 on 25-11-2.
//
#include "2006pid.h"
#include <math.h>

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
    
    // 1. 计算如果加上当前误差，积分项会变成多少
    float potential_integral = pid->integral + error;

    // 2. 计算对应的输出
    float potential_output = p_term + pid->Ki * potential_integral + d_term;

    // 3. 判断是否应该更新积分项
    int should_update = 0;

    // 情况1：输出不会饱和，可以正常积分
    if (fabs(potential_output) <= pid->output_max) {
        should_update = 1;
    }
    // 情况2：正向饱和但误差为负（帮助退出饱和）
    else if (potential_output > pid->output_max && error < 0) {
        should_update = 1;
    }
    // 情况3：负向饱和但误差为正（帮助退出饱和）
    else if (potential_output < -pid->output_max && error > 0) {
        should_update = 1;
    }
    // 情况4：饱和且误差方向与饱和方向相同，不积分（防止继续饱和）

    // 4. 更新积分项
    if (should_update) {
        pid->integral = potential_integral;  // 直接使用计算好的积分项
    }
    // 否则不更新积分项（抗饱和）

    // 5. 积分限幅（重要！防止积分项过大）
    // 设置一个合理的积分限幅，比如输出限幅的2-3倍
    float integral_max = pid->output_max * 2.0f;  // 20000
    if (pid->integral > integral_max) {
        pid->integral = integral_max;
    } else if (pid->integral < -integral_max) {
        pid->integral = -integral_max;
    }
    // 重新计算输出（使用更新后的积分项）
    pid->output = p_term + pid->Ki * pid->integral + d_term;
    
    // 限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < -pid->output_max)
        pid->output = -pid->output_max;

    return pid->output;
}