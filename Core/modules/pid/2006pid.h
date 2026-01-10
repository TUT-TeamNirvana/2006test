//
// Created by 17087 on 25-11-2.
//

#ifndef M2006PID_H
#define M2006PID_H

#include <stdbool.h>
#include <math.h>

/**
 * PID_t - 基本PID结构，向后兼容原有字段
 *
 * 为了支持“每个功能独立开关和参数，可为位置环和速度环分别配置”的需求，
 * 我们在原有PID结构上增加了：
 *  - 每项功能的开关（布尔型）：比例、积分、微分、速度前馈、加速度前馈
 *  - 若干常用的可配置参数：死区（deadband）、积分限幅（integral_limit）
 *
 * 重要兼容性说明：
 *  - 不改变现有的 `PID_Calc` 签名或其行为（调用方可继续按原方式使用）
 *  - 通过新的配置接口（在此头文件声明）来打开/关闭功能并设置参数
 *  - 因为每个环（位置环 / 速度环）都有各自的 `PID_t` 实例，
 *    所以对不同环使用不同配置是自然可行的（在电机初始化时或运行时调用 setter）
 */
typedef struct
{
    /* --- 原有控制量 --- */
    float Kp;
    float Ki;
    float Kd;
    float Kff;           // 速度前馈增益
    float Kaff;          // 加速度前馈增益

    /* PID 状态量（内部使用）*/
    float integral;
    float last_error;
    float last_ref;      // 上一次的目标值，用于计算加速度前馈
    float last_feedback; // 上一次的反馈值（用于微分先行实现）
    float output;
    float output_max;


    /* --- 可配置功能开关（每项为独立布尔开关） --- */
    bool enable_kp;         // 是否启用比例项（默认 true）
    bool enable_ki;         // 是否启用积分项（默认 true）
    bool enable_kd;         // 是否启用微分项（默认 true）
    bool enable_d_on_meas;  // 是否启用微分先行（基于测量，默认 false）
    bool enable_kff;        // 是否启用速度前馈（默认 true）
    bool enable_kaff;       // 是否启用加速度前馈（默认 true）


    /* --- 可配置参数 --- */
    float deadband;      // 误差死区（与反馈单位一致，默认与旧实现兼容）
    float integral_limit; // 积分限幅（用于防止积分风up，默认基于 output_max * 2.0f 保持兼容）

} PID_t;

/* 基本初始化（保持与原接口兼容） */
void PID_Init(PID_t* pid, float kp, float ki, float kd, float max_output);

/* 前馈设置（保持兼容） */
void PID_SetFeedforward(PID_t* pid, float kff, float kaff);

/*
 * 新增：按功能开关配置
 *  - 可以在初始化后、或运行时为单个 PID 实例配置哪些项是启用的
 *  - 例如：为位置环关闭前馈，或为速度环仅启用 PI（关闭 Kd）
 */
void PID_ConfigFeatures(PID_t* pid,
                        bool enable_kp,
                        bool enable_ki,
                        bool enable_kd,
                        bool enable_d_on_meas,
                        bool enable_kff,
                        bool enable_kaff);

/*
 * 新增：设置额外的可配置参数
 *  - deadband：误差死区（与旧实现中的 PID_ERROR_DEADBAND 对应）
 *  - integral_limit：手动设置积分限幅（若为 0 则采纳内部默认）
 */
void PID_SetParams(PID_t* pid, float deadband, float integral_limit);

/*
 * 计算函数（保持与原签名兼容）
 *  - 该函数内部应尊重 `PID_t` 中的开关和参数（在实现文件中完成）
 *  - 外部调用不需更改调用方式；要为位置/速度环使用不同配置，
 *    只需在各自的 PID_t 实例上调用上述配置接口。
 */
float PID_Calc(PID_t* pid, float ref, float feedback, float dt);

#endif //M2006PID_H
