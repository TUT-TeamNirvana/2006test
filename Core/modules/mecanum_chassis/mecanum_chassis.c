//
// Created by AI Assistant
// 麦克纳姆轮底盘控制模块
//

#include "mecanum_chassis.h"
#include <math.h>

// 私有变量
static M2006_t *chassis_motors = NULL;
static int8_t motor_dir[4] = {+1, +1, -1, -1}; // 按照电机安装方向

/**
 * @brief 初始化麦克纳姆轮底盘
 * @param motors 指向4个电机的数组指针
 * @param directions 电机方向数组（可选，传NULL则使用默认方向）
 */
void Mecanum_Chassis_Init(M2006_t *motors, int8_t *directions)
{
    chassis_motors = motors;
    
    // 如果提供了自定义方向，则使用自定义方向
    if (directions != NULL)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_dir[i] = directions[i];
        }
    }
}

/**
 * @brief 麦克纳姆轮底盘运动控制
 * @param vx X方向速度（前后）
 * @param vy Y方向速度（左右）
 * @param wz 旋转角速度
 */
void Mecanum_Chassis_Control(float vx, float vy, float wz)
{
    if (chassis_motors == NULL)
    {
        return; // 未初始化，直接返回
    }
    
    // 麦克纳姆轮运动学分配 (X形布局)
    float v1 = +vx - vy - wz;  // 左前
    float v2 = +vx + vy - wz;  // 左后
    float v3 = +vx - vy + wz;  // 右后
    float v4 = +vx + vy + wz;  // 右前

    // 应用安装方向
    M2006_SetSpeedTarget(&chassis_motors[0], motor_dir[0] * v1);
    M2006_SetSpeedTarget(&chassis_motors[1], motor_dir[1] * v2);
    M2006_SetSpeedTarget(&chassis_motors[2], motor_dir[2] * v3);
    M2006_SetSpeedTarget(&chassis_motors[3], motor_dir[3] * v4);
}

/**
 * @brief 设置底盘电机方向
 * @param directions 电机方向数组（长度为4）
 */
void Mecanum_Chassis_SetDirection(int8_t *directions)
{
    if (directions != NULL)
    {
        for (int i = 0; i < 4; i++)
        {
            motor_dir[i] = directions[i];
        }
    }
}

/**
 * @brief 获取底盘电机方向
 * @param directions 输出电机方向数组（长度为4）
 */
void Mecanum_Chassis_GetDirection(int8_t *directions)
{
    if (directions != NULL)
    {
        for (int i = 0; i < 4; i++)
        {
            directions[i] = motor_dir[i];
        }
    }
}