//
// Created by 17087 on 25-11-3.
//

#ifndef M2006_MOTOR_H
#define M2006_MOTOR_H

#include "main.h"
#include "bsp_can.h"
#include "cascade_pid.h"  // 使用串级PID（包含了2006pid.h）

// RC一阶低通滤波时间常数（单位：秒）
// 时间常数越小，滤波越轻，响应越快；时间常数越大，滤波越重，响应越慢
#define M2006_SPEED_LPF_TAU_S     0.010f  // 10ms，对应截止频率约16Hz
#define M2006_CURRENT_LPF_TAU_S   0.010f  // 10ms，对应截止频率约16Hz

#define M2006_MAX_NUM 3

// 控制模式枚举
typedef enum
{
    M2006_MODE_SPEED = CASCADE_MODE_SPEED_ONLY,    // 速度环模式（默认）
    M2006_MODE_CASCADE = CASCADE_MODE_CASCADE      // 串级模式（位置+速度）
} M2006_ControlMode_e;

// 反馈数据结构体
typedef struct
{
    int16_t angle_raw;         // 原始角度 (0-8191)
    float angle;               // 当前角度 (0-360度)
    float angle_continuous;    // 连续角度（支持多圈，单位：度）
    int16_t speed_rpm;         // 原始转速
    int16_t given_current;     // 给定电流
    uint8_t temp;              // 温度
    float speed_filtered;      // 一阶低通后的转速
    float current_filtered;    // 一阶低通后的电流
} M2006_Feedback_t;

// M2006电机结构体
typedef struct
{
    CANInstance *can;
    M2006_Feedback_t feedback;
    CascadePID_t controller;       // 串级PID控制器
    M2006_ControlMode_e mode;      // 当前控制模式
    float target;                  // 目标值（速度或位置，根据mode）
    uint8_t id;
    int32_t total_angle_raw;       // 累积原始角度（用于多圈计数）
    int16_t last_angle_raw;        // 上次原始角度（用于检测过零点）
} M2006_t;

/**
 * @brief 初始化单个电机（可为每个电机单独设置PID参数）
 * @param motor 电机指针
 * @param hcan CAN句柄
 * @param motor_id 电机ID (0-3对应0x201-0x204)
 * @param outer_kp 外环（位置环）比例系数
 * @param outer_ki 外环（位置环）积分系数
 * @param outer_kd 外环（位置环）微分系数
 * @param inner_kp 内环（速度环）比例系数
 * @param inner_ki 内环（速度环）积分系数
 * @param inner_kd 内环（速度环）微分系数
 * @param speed_limit 速度限制 (RPM)
 * @param current_limit 电流限制 (mA)
 */
void M2006_InitSingle(M2006_t *motor, CAN_HandleTypeDef *hcan, uint8_t motor_id,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd,
                     float speed_limit, float current_limit);

/**
 * @brief 初始化所有电机（使用默认参数）
 * @param motors 电机数组
 * @param hcan CAN句柄
 * @note 使用默认参数，建议后续为每个电机单独调参
 */
void M2006_InitAll(M2006_t *motors, CAN_HandleTypeDef *hcan);

/**
 * @brief 获取内环（速度环）PID指针，用于配置功能
 * @param motor 电机指针
 * @return 速度环PID指针
 * @note 可用于调用 PID_SetFeature、PID_SetIntegralSeparation 等功能配置函数
 */
PID_t* M2006_GetInnerPID(M2006_t *motor);

/**
 * @brief 获取外环（位置环）PID指针，用于配置功能
 * @param motor 电机指针
 * @return 位置环PID指针
 * @note 可用于调用 PID_SetFeature、PID_SetIntegralSeparation 等功能配置函数
 */
PID_t* M2006_GetOuterPID(M2006_t *motor);

// 设置控制模式
void M2006_SetControlMode(M2006_t *motor, M2006_ControlMode_e mode);

// 设置目标速度（RPM）- 速度环模式下使用
void M2006_SetSpeedTarget(M2006_t *motor, float target_rpm);

// 设置目标位置（度）- 串级模式下使用
void M2006_SetPosTarget(M2006_t *motor, float target_angle);

// 兼容旧接口（默认设置速度目标）
void M2006_SetTarget(M2006_t *motor, float target);

// 更新所有电机
void M2006_UpdateAll(M2006_t *motors, uint8_t motor_count);

// CAN回调
void M2006_Callback(CANInstance *instance);

// 获取反馈频率
uint32_t M2006_GetFeedbackFrequency(uint8_t motor_id);

#endif //M2006_MOTOR_H
