//
// Created by 17087 on 25-11-2.
//

#ifndef M2006PID_H
#define M2006PID_H

#include <stdint.h>

// ==================== PID功能配置标志位 ====================
// 使用位掩码，可以灵活组合各种功能
typedef enum
{
    PID_FEATURE_NONE                = 0x00,    // 无额外功能（基础PID）
    PID_FEATURE_FEEDFORWARD         = 0x01,    // 速度前馈（Kff）
    PID_FEATURE_ACCEL_FEEDFORWARD   = 0x02,    // 加速度前馈（Kaff）
    PID_FEATURE_DERIVATIVE_ON_MEAS  = 0x04,    // 微分先行（微分作用于测量值而非误差）
    PID_FEATURE_INTEGRAL_SEPARATION = 0x08,    // 积分分离（大误差时不积分）
    PID_FEATURE_ANTI_WINDUP         = 0x10,    // 抗积分饱和（默认总是启用，此标志保留）
    PID_FEATURE_ERROR_DEADBAND      = 0x20,    // 误差死区
} PIDFeature_e;

// ==================== PID参数配置结构体 ====================
typedef struct
{
    // 基础PID参数
    float Kp;
    float Ki;
    float Kd;
    float output_max;              // 输出限幅
    
    // 功能启用标志（组合使用 PIDFeature_e）
    uint32_t features;
    
    // ===== 前馈参数 =====
    float Kff;                     // 速度前馈增益（需启用 PID_FEATURE_FEEDFORWARD）
    float Kaff;                    // 加速度前馈增益（需启用 PID_FEATURE_ACCEL_FEEDFORWARD）
    
    // ===== 积分分离参数 =====
    float integral_separation_threshold;  // 积分分离阈值（需启用 PID_FEATURE_INTEGRAL_SEPARATION）
    
    // ===== 误差死区参数 =====
    float error_deadband;          // 误差死区范围（需启用 PID_FEATURE_ERROR_DEADBAND）
    
    // ===== 积分限幅参数 =====
    float integral_limit_factor;   // 积分限幅系数（积分限幅 = output_max * factor）
                                   // 默认为 2.0，可以为速度环和位置环设置不同值
    
} PIDConfig_t;

// ==================== PID运行时状态 ====================
typedef struct
{
    // 配置（只读，初始化后不应修改）
    PIDConfig_t config;
    
    // 运行时状态
    float integral;
    float last_error;
    float last_feedback;           // 上一次的反馈值（用于微分先行）
    float last_ref;                // 上一次的目标值（用于加速度前馈）
    float output;
    
} PID_t;

// ==================== API函数 ====================

/**
 * @brief 使用配置结构体初始化PID
 * @param pid PID控制器指针
 * @param config 配置参数指针
 */
void PID_InitWithConfig(PID_t *pid, const PIDConfig_t *config);

/**
 * @brief 快速初始化PID（向后兼容，基础功能）
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_output 输出限幅
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_output);

/**
 * @brief 设置前馈参数（向后兼容）
 * @param pid PID控制器指针
 * @param kff 速度前馈增益
 * @param kaff 加速度前馈增益
 * @note 调用此函数会自动启用前馈功能
 */
void PID_SetFeedforward(PID_t *pid, float kff, float kaff);

/**
 * @brief 启用或禁用某个功能
 * @param pid PID控制器指针
 * @param feature 功能标志（可使用 | 组合多个功能）
 * @param enable 1=启用, 0=禁用
 */
void PID_SetFeature(PID_t *pid, PIDFeature_e feature, int enable);

/**
 * @brief 设置积分分离阈值
 * @param pid PID控制器指针
 * @param threshold 误差阈值（超过此值时不积分）
 */
void PID_SetIntegralSeparation(PID_t *pid, float threshold);

/**
 * @brief 设置误差死区
 * @param pid PID控制器指针
 * @param deadband 死区范围
 */
void PID_SetErrorDeadband(PID_t *pid, float deadband);

/**
 * @brief 设置积分限幅系数
 * @param pid PID控制器指针
 * @param factor 积分限幅系数（积分限幅 = output_max * factor）
 */
void PID_SetIntegralLimitFactor(PID_t *pid, float factor);

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