/**
 ******************************************************************************
 * @file    cascade_pid.c
 * @brief   串级PID控制器实现
 ******************************************************************************
 */

#include "cascade_pid.h"

/**
 * @brief 初始化串级PID
 */
void CascadePID_Init(CascadePID_t *cpid,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd,
                     float speed_limit, float current_limit)
{
    // 初始化外环（位置环）
    PID_Init(&cpid->outer_loop, outer_kp, outer_ki, outer_kd, speed_limit);
    
    // 初始化内环（速度环）
    PID_Init(&cpid->inner_loop, inner_kp, inner_ki, inner_kd, current_limit);
    
    // 默认为单速度环模式（向后兼容）
    cpid->mode = CASCADE_MODE_SPEED_ONLY;
    cpid->speed_limit = speed_limit;
}

/**
 * @brief 设置控制模式
 */
void CascadePID_SetMode(CascadePID_t *cpid, CascadeMode_e mode)
{
    cpid->mode = mode;
    
    // 切换模式时复位PID状态
    CascadePID_Reset(cpid);
}

/**
 * @brief 串级PID计算
 */
float CascadePID_Calculate(CascadePID_t *cpid,
                          float target,
                          float actual_pos,
                          float actual_speed,
                          float dt)
{
    float output = 0.0f;
    
    switch (cpid->mode)
    {
        case CASCADE_MODE_SPEED_ONLY:
        {
            // 单速度环模式：target直接作为速度目标
            output = PID_Calc(&cpid->inner_loop, target, actual_speed, dt);
            break;
        }
        
        case CASCADE_MODE_CASCADE:
        {
            // 串级模式：位置环输出作为速度环目标
            float target_speed = PID_Calc(&cpid->outer_loop, target, actual_pos, dt);
            output = PID_Calc(&cpid->inner_loop, target_speed, actual_speed, dt);
            break;
        }
        
        default:
            output = 0.0f;
            break;
    }
    
    return output;
}

/**
 * @brief 复位串级PID状态
 */
void CascadePID_Reset(CascadePID_t *cpid)
{
    // 复位外环
    cpid->outer_loop.integral = 0.0f;
    cpid->outer_loop.last_error = 0.0f;
    cpid->outer_loop.last_ref = 0.0f;
    cpid->outer_loop.output = 0.0f;
    
    // 复位内环
    cpid->inner_loop.integral = 0.0f;
    cpid->inner_loop.last_error = 0.0f;
    cpid->inner_loop.last_ref = 0.0f;
    cpid->inner_loop.output = 0.0f;
}