/**
 ******************************************************************************
 * @file    cascade_pid.c
 * @brief   串级PID控制器实现 - 支持位置环和速度环独立配置
 * @note    位置环和速度环的功能参数可独立设置，互不影响
 ******************************************************************************
 */

#include "cascade_pid.h"

// ==================== 初始化函数 ====================

/**
 * @brief 初始化串级PID（只设置基本PID参数）
 */
void CascadePID_Init(CascadePID_t *cpid,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd)
{
    // 初始化外环（位置环）
    PID_Init(&cpid->outer_loop, outer_kp, outer_ki, outer_kd);
    
    // 初始化内环（速度环）
    PID_Init(&cpid->inner_loop, inner_kp, inner_ki, inner_kd);
    
    // 默认为单速度环模式（向后兼容）
    cpid->mode = CASCADE_MODE_SPEED_ONLY;
}


// ==================== 模式控制函数 ====================

/**
 * @brief 设置控制模式
 */
void CascadePID_SetMode(CascadePID_t *cpid, CascadeMode_e mode)
{
    cpid->mode = mode;
    
    // 切换模式时复位PID状态
    CascadePID_Reset(cpid);
}


// ==================== 位置环（外环）功能配置函数 ====================

/**
 * @brief 配置位置环速度前馈
 */
void CascadePID_ConfigOuterFeedforward(CascadePID_t *cpid, bool enable, float kff)
{
    PID_ConfigFeedforward(&cpid->outer_loop, enable, kff);
}

/**
 * @brief 配置位置环加速度前馈
 */
void CascadePID_ConfigOuterAccelFeedforward(CascadePID_t *cpid, bool enable, float kaff)
{
    PID_ConfigAccelFeedforward(&cpid->outer_loop, enable, kaff);
}

/**
 * @brief 配置位置环积分限幅
 */
void CascadePID_ConfigOuterIntegralLimit(CascadePID_t *cpid, bool enable, float max, float min)
{
    PID_ConfigIntegralLimit(&cpid->outer_loop, enable, max, min);
}

/**
 * @brief 配置位置环微分滤波
 */
void CascadePID_ConfigOuterDerivativeFilter(CascadePID_t *cpid, bool enable, float coef)
{
    PID_ConfigDerivativeFilter(&cpid->outer_loop, enable, coef);
}

/**
 * @brief 配置位置环输出限幅
 */
void CascadePID_ConfigOuterOutputLimit(CascadePID_t *cpid, bool enable, float max, float min)
{
    PID_ConfigOutputLimit(&cpid->outer_loop, enable, max, min);
}

/**
 * @brief 配置位置环抗积分饱和
 */
void CascadePID_ConfigOuterAntiWindup(CascadePID_t *cpid, bool enable)
{
    PID_ConfigAntiWindup(&cpid->outer_loop, enable);
}

/**
 * @brief 配置位置环误差死区
 */
void CascadePID_ConfigOuterErrorDeadband(CascadePID_t *cpid, bool enable, float deadband)
{
    PID_ConfigErrorDeadband(&cpid->outer_loop, enable, deadband);
}


// ==================== 速度环（内环）功能配置函数 ====================

/**
 * @brief 配置速度环速度前馈
 */
void CascadePID_ConfigInnerFeedforward(CascadePID_t *cpid, bool enable, float kff)
{
    PID_ConfigFeedforward(&cpid->inner_loop, enable, kff);
}

/**
 * @brief 配置速度环加速度前馈
 */
void CascadePID_ConfigInnerAccelFeedforward(CascadePID_t *cpid, bool enable, float kaff)
{
    PID_ConfigAccelFeedforward(&cpid->inner_loop, enable, kaff);
}

/**
 * @brief 配置速度环积分限幅
 */
void CascadePID_ConfigInnerIntegralLimit(CascadePID_t *cpid, bool enable, float max, float min)
{
    PID_ConfigIntegralLimit(&cpid->inner_loop, enable, max, min);
}

/**
 * @brief 配置速度环微分滤波
 */
void CascadePID_ConfigInnerDerivativeFilter(CascadePID_t *cpid, bool enable, float coef)
{
    PID_ConfigDerivativeFilter(&cpid->inner_loop, enable, coef);
}

/**
 * @brief 配置速度环输出限幅
 */
void CascadePID_ConfigInnerOutputLimit(CascadePID_t *cpid, bool enable, float max, float min)
{
    PID_ConfigOutputLimit(&cpid->inner_loop, enable, max, min);
}

/**
 * @brief 配置速度环抗积分饱和
 */
void CascadePID_ConfigInnerAntiWindup(CascadePID_t *cpid, bool enable)
{
    PID_ConfigAntiWindup(&cpid->inner_loop, enable);
}

/**
 * @brief 配置速度环误差死区
 */
void CascadePID_ConfigInnerErrorDeadband(CascadePID_t *cpid, bool enable, float deadband)
{
    PID_ConfigErrorDeadband(&cpid->inner_loop, enable, deadband);
}


// ==================== 计算和控制函数 ====================

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
    // 复位外环（位置环）
    PID_Reset(&cpid->outer_loop);
    
    // 复位内环（速度环）
    PID_Reset(&cpid->inner_loop);
}