/**
 ******************************************************************************
 * @file    cascade_pid.h
 * @brief   串级PID控制器（位置环+速度环），支持独立配置
 * @note    位置环和速度环的功能参数可独立设置，互不影响
 ******************************************************************************
 */

#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include "2006pid.h"

/* 控制模式枚举 */
typedef enum
{
    CASCADE_MODE_SPEED_ONLY = 0,    // 单速度环模式（默认）
    CASCADE_MODE_CASCADE = 1        // 串级模式（位置+速度）
} CascadeMode_e;

/* 串级PID结构体 */
typedef struct
{
    PID_t outer_loop;              // 外环（位置环）- 独立配置
    PID_t inner_loop;              // 内环（速度环）- 独立配置
    
    CascadeMode_e mode;            // 当前控制模式
    
} CascadePID_t;


// ==================== 初始化函数 ====================

/**
 * @brief 初始化串级PID（只设置基本PID参数）
 * @param cpid 串级PID指针
 * @param outer_kp 外环比例增益（位置环）
 * @param outer_ki 外环积分增益（位置环，通常为0）
 * @param outer_kd 外环微分增益（位置环）
 * @param inner_kp 内环比例增益（速度环）
 * @param inner_ki 内环积分增益（速度环）
 * @param inner_kd 内环微分增益（速度环，通常为0）
 * @note 功能配置需要使用各自的配置函数单独设置
 */
void CascadePID_Init(CascadePID_t *cpid,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd);


// ==================== 模式控制函数 ====================

/**
 * @brief 设置控制模式
 * @param cpid 串级PID指针
 * @param mode 控制模式
 */
void CascadePID_SetMode(CascadePID_t *cpid, CascadeMode_e mode);


// ==================== 位置环（外环）功能配置函数 ====================

/**
 * @brief 配置位置环速度前馈
 */
void CascadePID_ConfigOuterFeedforward(CascadePID_t *cpid, bool enable, float kff);

/**
 * @brief 配置位置环加速度前馈
 */
void CascadePID_ConfigOuterAccelFeedforward(CascadePID_t *cpid, bool enable, float kaff);

/**
 * @brief 配置位置环积分限幅
 */
void CascadePID_ConfigOuterIntegralLimit(CascadePID_t *cpid, bool enable, float max, float min);

/**
 * @brief 配置位置环微分滤波
 */
void CascadePID_ConfigOuterDerivativeFilter(CascadePID_t *cpid, bool enable, float coef);

/**
 * @brief 配置位置环输出限幅
 */
void CascadePID_ConfigOuterOutputLimit(CascadePID_t *cpid, bool enable, float max, float min);

/**
 * @brief 配置位置环抗积分饱和
 */
void CascadePID_ConfigOuterAntiWindup(CascadePID_t *cpid, bool enable);

/**
 * @brief 配置位置环误差死区
 */
void CascadePID_ConfigOuterErrorDeadband(CascadePID_t *cpid, bool enable, float deadband);


// ==================== 速度环（内环）功能配置函数 ====================

/**
 * @brief 配置速度环速度前馈
 */
void CascadePID_ConfigInnerFeedforward(CascadePID_t *cpid, bool enable, float kff);

/**
 * @brief 配置速度环加速度前馈
 */
void CascadePID_ConfigInnerAccelFeedforward(CascadePID_t *cpid, bool enable, float kaff);

/**
 * @brief 配置速度环积分限幅
 */
void CascadePID_ConfigInnerIntegralLimit(CascadePID_t *cpid, bool enable, float max, float min);

/**
 * @brief 配置速度环微分滤波
 */
void CascadePID_ConfigInnerDerivativeFilter(CascadePID_t *cpid, bool enable, float coef);

/**
 * @brief 配置速度环输出限幅
 */
void CascadePID_ConfigInnerOutputLimit(CascadePID_t *cpid, bool enable, float max, float min);

/**
 * @brief 配置速度环抗积分饱和
 */
void CascadePID_ConfigInnerAntiWindup(CascadePID_t *cpid, bool enable);

/**
 * @brief 配置速度环误差死区
 */
void CascadePID_ConfigInnerErrorDeadband(CascadePID_t *cpid, bool enable, float deadband);


// ==================== 计算和控制函数 ====================

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


// ==================== 便捷访问宏（可选） ====================

/**
 * @brief 直接访问位置环PID控制器
 * @note  可用于直接设置参数或读取状态
 */
#define CASCADE_GET_OUTER_PID(cpid)  (&((cpid)->outer_loop))

/**
 * @brief 直接访问速度环PID控制器
 * @note  可用于直接设置参数或读取状态
 */
#define CASCADE_GET_INNER_PID(cpid)  (&((cpid)->inner_loop))


#endif // CASCADE_PID_H