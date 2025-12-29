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

/*******************************************************************************
 * 新增：为外环/内环提供配置包装函数（调用 PID 层的配置接口）
 *
 * 这些函数允许在不改变原有控制调用点和速度环计算逻辑的前提下，
 * 为位置环（outer_loop）和速度环（inner_loop）分别开启/关闭功能并设置参数。
 *******************************************************************************/

/**
 * @brief 为外环（位置环）配置功能开关
 */
void CascadePID_ConfigOuterFeatures(CascadePID_t *cpid,
                                    bool enable_kp,
                                    bool enable_ki,
                                    bool enable_kd,
                                    bool enable_kff,
                                    bool enable_kaff)
{
    if (cpid == NULL) return;
    PID_ConfigFeatures(&cpid->outer_loop,
                       enable_kp, enable_ki, enable_kd,
                       enable_kff, enable_kaff);
}

/**
 * @brief 为内环（速度环）配置功能开关
 */
void CascadePID_ConfigInnerFeatures(CascadePID_t *cpid,
                                    bool enable_kp,
                                    bool enable_ki,
                                    bool enable_kd,
                                    bool enable_kff,
                                    bool enable_kaff)
{
    if (cpid == NULL) return;
    PID_ConfigFeatures(&cpid->inner_loop,
                       enable_kp, enable_ki, enable_kd,
                       enable_kff, enable_kaff);
}

/**
 * @brief 为外环设置可调参数（例如死区与积分限幅）
 */
void CascadePID_SetOuterParams(CascadePID_t *cpid, float deadband, float integral_limit)
{
    if (cpid == NULL) return;
    PID_SetParams(&cpid->outer_loop, deadband, integral_limit);
}

/**
 * @brief 为内环设置可调参数（例如死区与积分限幅）
 */
void CascadePID_SetInnerParams(CascadePID_t *cpid, float deadband, float integral_limit)
{
    if (cpid == NULL) return;
    PID_SetParams(&cpid->inner_loop, deadband, integral_limit);
}