/**
 ******************************************************************************
 * @file    cascade_pid.h
 * @brief   串级PID控制器（位置环+速度环）
 * @note    方案A：只支持速度环和串级两种模式
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


/*******************************************************************************
 *  下面是新增的配置接口（帮助在不改变速度环 PID 计算调用方式的前提下，
 *  为位置环（outer_loop）和速度环（inner_loop）分别配置功能开关和参数）
 *
 *  说明：
 *  - 每个 CascadePID_t 包含独立的 outer_loop 和 inner_loop（类型为 PID_t），
 *    因此可以通过这些接口为两环分别设置是否启用比例/积分/微分/速度前馈/加速度前馈
 *    以及每环的 deadband 和 integral_limit。
 *  - 这些接口只声明在头文件中；实现应在 cascade_pid.c 或 2006pid.c 中完成，
 *    且不改变现有的 PID_Calc 使用方式（调用方无需更改调用点）。
 *******************************************************************************/

/**
 * @brief 为外环（位置环）配置功能开关
 * @param cpid 串级PID指针
 * @param enable_kp 是否启用比例项
 * @param enable_ki 是否启用积分项
 * @param enable_kd 是否启用微分项
 * @param enable_kff 是否启用速度前馈
 * @param enable_kaff 是否启用加速度前馈
 */
/**
 * @brief 为外环（位置环）配置功能开关
 * @param cpid 串级PID指针
 * @param enable_kp 是否启用比例项
 * @param enable_ki 是否启用积分项
 * @param enable_kd 是否启用微分项
 * @param enable_d_on_meas 是否启用微分先行（基于测量），true=使用测量导数代替误差导数
 * @param enable_kff 是否启用速度前馈
 * @param enable_kaff 是否启用加速度前馈
 */
void CascadePID_ConfigOuterFeatures(CascadePID_t *cpid,
                                    bool enable_kp,
                                    bool enable_ki,
                                    bool enable_kd,
                                    bool enable_d_on_meas,
                                    bool enable_kff,
                                    bool enable_kaff);

/**
 * @brief 为内环（速度环）配置功能开关
 * @param cpid 串级PID指针
 * @param enable_kp 是否启用比例项
 * @param enable_ki 是否启用积分项
 * @param enable_kd 是否启用微分项
 * @param enable_kff 是否启用速度前馈
 * @param enable_kaff 是否启用加速度前馈
 */
/**
 * @brief 为内环（速度环）配置功能开关
 * @param cpid 串级PID指针
 * @param enable_kp 是否启用比例项
 * @param enable_ki 是否启用积分项
 * @param enable_kd 是否启用微分项
 * @param enable_d_on_meas 是否启用微分先行（基于测量），true=使用测量导数代替误差导数
 * @param enable_kff 是否启用速度前馈
 * @param enable_kaff 是否启用加速度前馈
 */
void CascadePID_ConfigInnerFeatures(CascadePID_t *cpid,
                                    bool enable_kp,
                                    bool enable_ki,
                                    bool enable_kd,
                                    bool enable_d_on_meas,
                                    bool enable_kff,
                                    bool enable_kaff);

/**
 * @brief 为外环设置可调参数（例如死区与积分限幅）
 * @param cpid 串级PID指针
 * @param deadband 误差死区（>0 表示使用该值；<=0 表示使用库默认）
 * @param integral_limit 积分限幅（<=0 表示使用默认基于 output_max 的限幅）
 */
void CascadePID_SetOuterParams(CascadePID_t *cpid, float deadband, float integral_limit);

/**
 * @brief 为内环设置可调参数（例如死区与积分限幅）
 * @param cpid 串级PID指针
 * @param deadband 误差死区（>0 表示使用该值；<=0 表示使用库默认）
 * @param integral_limit 积分限幅（<=0 表示使用默认基于 output_max 的限幅）
 */
void CascadePID_SetInnerParams(CascadePID_t *cpid, float deadband, float integral_limit);

#endif // CASCADE_PID_H