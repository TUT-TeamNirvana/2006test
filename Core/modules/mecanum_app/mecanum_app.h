//
// Created by AI Assistant
// 麦轮应用模块 - 封装麦轮项目的完整控制逻辑
// 注意：此模块专用于麦轮项目，不适用于轮腿项目
//

#ifndef MECANUM_APP_H
#define MECANUM_APP_H

#include "m2006_motor.h"
#include "sbus.h"
#include "usart.h"
#include <stdint.h>

/**
 * @brief 麦轮应用配置结构体
 */
typedef struct {
    M2006_t *motors;              // 电机数组指针（至少4个）
    int8_t *motor_directions;     // 电机方向数组（4个元素）
    float deadzone;               // 遥控器死区阈值
} MecanumAppConfig_t;

/**
 * @brief 初始化麦轮应用
 * @param config 麦轮应用配置结构体指针
 * @note 必须在SBUS、CAN、UART初始化后调用
 */
void MecanumApp_Init(MecanumAppConfig_t *config);

/**
 * @brief 麦轮应用主循环更新（1ms周期调用）
 * @param sbus_data SBUS数据结构指针
 * @note 处理遥控器输入、底盘控制、夹爪控制、电机PID更新
 */
void MecanumApp_Update(SBUS_Data_t *sbus_data);

/**
 * @brief 停止麦轮应用（所有电机停止）
 */
void MecanumApp_Stop(void);

#endif // MECANUM_APP_H