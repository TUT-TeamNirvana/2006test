//
// Created by 17087 on 25-11-2.
//

#ifndef M2006PID_H
#define M2006PID_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID功能配置结构体
 * @note  每个功能都有独立的使能开关和参数
 */
typedef struct
{
    // ===== 前馈控制 =====
    bool enable_feedforward;        // 速度前馈使能
    float Kff;                      // 速度前馈增益
    
    bool enable_accel_feedforward;  // 加速度前馈使能
    float Kaff;                     // 加速度前馈增益
    
    // ===== 积分限幅 =====
    bool enable_integral_limit;     // 积分限幅使能
    float integral_max;             // 积分最大值
    float integral_min;             // 积分最小值
    
    // ===== 微分滤波 =====
    bool enable_derivative_filter;  // 微分滤波使能
    float derivative_filter_coef;   // 微分滤波系数 (0~1, 越大滤波越强)
    
    // ===== 输出限幅 =====
    bool enable_output_limit;       // 输出限幅使能
    float output_max;               // 输出最大值
    float output_min;               // 输出最小值
    
    // ===== 抗积分饱和 =====
    bool enable_anti_windup;        // 抗积分饱和使能
    
    // ===== 误差死区 =====
    bool enable_error_deadband;     // 误差死区使能
    float error_deadband;           // 死区阈值
    
} PID_Config_t;

/**
 * @brief PID基本参数结构体
 */
typedef struct
{
    float Kp;  // 比例增益
    float Ki;  // 积分增益
    float Kd;  // 微分增益
} PID_Params_t;

/**
 * @brief PID状态变量结构体
 */
typedef struct
{
    float integral;              // 积分累积
    float last_error;            // 上次误差
    float last_ref;              // 上次目标值（用于加速度前馈）
    float last_derivative;       // 上次微分值（用于微分滤波）
    float output;                // 输出值
} PID_State_t;

/**
 * @brief PID控制器完整结构体
 */
typedef struct
{
    PID_Params_t params;  // PID参数
    PID_Config_t config;  // 功能配置
    PID_State_t state;    // 状态变量
} PID_t;


// ==================== 初始化函数 ====================

/**
 * @brief 初始化PID控制器（基本参数）
 * @param pid PID控制器指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief 初始化PID配置为默认值（所有功能禁用）
 * @param pid PID控制器指针
 */
void PID_ConfigDefault(PID_t *pid);


// ==================== 功能配置函数 ====================

/**
 * @brief 配置速度前馈
 * @param pid PID控制器指针
 * @param enable 是否使能
 * @param kff 速度前馈增益
 */
void PID_ConfigFeedforward(PID_t *pid, bool enable, float kff);

/**
 * @brief 配置加速度前馈
 * @param pid PID控制器指针
 * @param enable 是否使能
 * @param kaff 加速度前馈增益
 */
void PID_ConfigAccelFeedforward(PID_t *pid, bool enable, float kaff);

/**
 * @brief 配置积分限幅
 * @param pid PID控制器指针
 * @param enable 是否使能
 * @param max 积分最大值
 * @param min 积分最小值
 */
void PID_ConfigIntegralLimit(PID_t *pid, bool enable, float max, float min);

/**
 * @brief 配置微分滤波
 * @param pid PID控制器指针
 * @param enable 是否使能
 * @param coef 滤波系数 (0~1, 0=不滤波, 1=完全滤波)
 */
void PID_ConfigDerivativeFilter(PID_t *pid, bool enable, float coef);

/**
 * @brief 配置输出限幅
 * @param pid PID控制器指针
 * @param enable 是否使能
 * @param max 输出最大值
 * @param min 输出最小值
 */
void PID_ConfigOutputLimit(PID_t *pid, bool enable, float max, float min);

/**
 * @brief 配置抗积分饱和
 * @param pid PID控制器指针
 * @param enable 是否使能
 */
void PID_ConfigAntiWindup(PID_t *pid, bool enable);

/**
 * @brief 配置误差死区
 * @param pid PID控制器指针
 * @param enable 是否使能
 * @param deadband 死区阈值
 */
void PID_ConfigErrorDeadband(PID_t *pid, bool enable, float deadband);


// ==================== 计算和控制函数 ====================

/**
 * @brief PID计算
 * @param pid PID控制器指针
 * @param ref 目标值
 * @param feedback 反馈值
 * @param dt 时间间隔（秒）
 * @return 控制输出
 */
float PID_Calc(PID_t *pid, float ref, float feedback, float dt);

/**
 * @brief 复位PID状态
 * @param pid PID控制器指针
 */
void PID_Reset(PID_t *pid);


#endif //M2006PID_H