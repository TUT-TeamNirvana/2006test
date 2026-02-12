//
// Created by AI Assistant
// 麦克纳姆轮底盘运动控制模块
//

#ifndef MECANUM_CHASSIS_H
#define MECANUM_CHASSIS_H

#include "m2006_motor.h"
#include <stdint.h>

/**
 * @brief 初始化麦克纳姆轮底盘
 * @param motors 指向4个电机的数组指针
 * @param directions 电机方向数组（可选，传NULL则使用默认方向 {+1, +1, -1, -1}）
 */
void Mecanum_Chassis_Init(M2006_t *motors, int8_t *directions);

/**
 * @brief 麦克纳姆轮底盘运动控制
 * @param vx X方向速度（前后）
 * @param vy Y方向速度（左右）
 * @param wz 旋转角速度
 * @note 使用X形麦克纳姆轮布局的运动学分配
 */
void Mecanum_Chassis_Control(float vx, float vy, float wz);

/**
 * @brief 设置底盘电机方向
 * @param directions 电机方向数组（长度为4）
 */
void Mecanum_Chassis_SetDirection(int8_t *directions);

/**
 * @brief 获取底盘电机方向
 * @param directions 输出电机方向数组（长度为4）
 */
void Mecanum_Chassis_GetDirection(int8_t *directions);

#endif // MECANUM_CHASSIS_H