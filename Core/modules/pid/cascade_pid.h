/**
 ******************************************************************************
 * @file    cascade_pid.h
 * @brief   串级PID控制器（位置环+速度环）
 * @note    支持外环和内环独立功能配置
 ******************************************************************************
 */

#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include "2006pid.h"

/* 控制模式枚举（方案A：只有2种模式） */
typedef enum
{
    CASCADE_MODE_SPEED_ONLY = 0,    // 单速度环模式（默认）
    CASCADE_MODE_CASCADE = 1        // 串级模式（位置+速度）
} CascadeMode_e;

/* 串级PID结构体 */
typedef struct
{
    PID_t outer_loop;  // 外环（位置环，通常用PD）
    PID_t inner_loop;  // 内环（速度环，通常用PI）
    
    CascadeMode_e mode;            // 当前控制模式
    float speed_limit;             // 外环输出限制（速度限制，RPM）
    
} CascadePID_t;


/**
 * @brief 初始化串级PID
 * @param cpid 串级PID指针
 * @param outer_kp 外环比例增益（位置环）
 * @param outer_ki 外环积分增益（位置环，通常为0）
 * @param outer_kd 外环微分增益（位置环）
 * @param inner_kp 内环比例增益（速度环）
 * @param inner_ki 内环积分增益（速度环）
 * @param inner_kd 内环微分增益（速度环，通常为0）
 * @param speed_limit 速度限制（RPM）
 * @param current_limit 电流限制（mA）
 */
void CascadePID_Init(CascadePID_t *cpid,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd,
                     float speed_limit, float current_limit);

/**
 * @brief 使用配置结构体初始化串级PID（高级）
 * @param cpid 串级PID指针
 * @param outer_config 外环（位置环）配置
 * @param inner_config 内环（速度环）配置
 * @param speed_limit 速度限制（RPM）
 */
void CascadePID_InitWithConfig(CascadePID_t *cpid,
                               const PIDConfig_t *outer_config,
                               const PIDConfig_t *inner_config,
                               float speed_limit);

/**
 * @brief 获取外环（位置环）PID指针，用于配置功能
 * @param cpid 串级PID指针
 * @return 外环PID指针
 * @note 可用于调用 PID_SetFeature、PID_SetIntegralSeparation 等功能配置函数
 */
PID_t* CascadePID_GetOuterLoop(CascadePID_t *cpid);

/**
 * @brief 获取内环（速度环）PID指针，用于配置功能
 * @param cpid 串级PID指针
 * @return 内环PID指针
 * @note 可用于调用 PID_SetFeature、PID_SetIntegralSeparation 等功能配置函数
 */
PID_t* CascadePID_GetInnerLoop(CascadePID_t *cpid);

/**
 * @brief 设置控制模式
 * @param cpid 串级PID指针
 * @param mode 控制模式
 */
void CascadePID_SetMode(CascadePID_t *cpid, CascadeMode_e mode);

/**
 * @brief 串级PID计算
 * @param cpid 串级PID指针
 * @param target 目标值（速度模式：RPM，串级模式：角度）
 * @param actual_pos 实际位置（度）- 仅串级模式使用
 * @param actual_speed 实际速度（RPM）
 * @param dt 控制周期（秒）
 * @return 电流输出（mA）
 * 
 * @note 根据mode的不同：
 *       - SPEED_ONLY: target直接作为速度目标，actual_pos被忽略
 *       - CASCADE: target作为位置目标，外环输出作为内环目标
 */
float CascadePID_Calculate(CascadePID_t *cpid,
                          float target,
                          float actual_pos,
                          float actual_speed,
                          float dt);

/**
 * @brief 复位串级PID状态
 * @param cpid 串级PID指针
 */
void CascadePID_Reset(CascadePID_t *cpid);

#endif // CASCADE_PID_H