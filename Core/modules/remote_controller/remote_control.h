//
// Created by AI Assistant
// 遥控器输入处理模块
// 注意：此模块专用于麦轮项目，不适用于轮腿项目
//

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "sbus.h"
#include "usart.h"

/**
 * @brief 映射函数 - 将输入值从一个范围映射到另一个范围
 * @param x 输入值
 * @param in_min 输入最小值
 * @param in_max 输入最大值
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 * @return 映射后的输出值（带限幅）
 */
float RC_MapValue(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief 获取处理后的底盘控制值（包含死区处理）
 * @param sbus_data SBUS数据结构指针
 * @param vx 输出X轴速度（前后）
 * @param vy 输出Y轴速度（左右）
 * @param wz 输出旋转角速度
 * @param deadzone 死区阈值（默认200）
 */
void RC_GetChassisControl(SBUS_Data_t *sbus_data, float *vx, float *vy, float *wz, float deadzone);

/**
 * @brief 初始化夹爪控制模块（麦轮项目专用）
 */
void RC_GripperInit(void);

/**
 * @brief 夹爪控制处理（非阻塞）
 * @param sbus_data SBUS数据结构指针
 * @note 通过CH5通道控制夹爪，使用环形缓冲区非阻塞发送
 * @warning 此函数专用于麦轮项目的非FSUS舵机，不适用于轮腿项目
 */
void RC_ProcessGripperControl(SBUS_Data_t *sbus_data);

#endif // REMOTE_CONTROL_H