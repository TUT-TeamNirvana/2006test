//
// Created by AI Assistant
// 麦轮应用模块实现 - 封装麦轮项目的完整控制逻辑
// 注意：此模块专用于麦轮项目，不适用于轮腿项目
//

#include "mecanum_app.h"
#include "mecanum_chassis.h"
#include "remote_control.h"

// 私有变量
static MecanumAppConfig_t app_config;
static uint8_t is_initialized = 0;

/**
 * @brief 初始化麦轮应用
 * @param config 麦轮应用配置结构体指针
 * @note 必须在SBUS、CAN、UART初始化后调用
 */
void MecanumApp_Init(MecanumAppConfig_t *config)
{
    if (config == NULL || config->motors == NULL)
    {
        return;
    }
    
    // 保存配置
    app_config = *config;
    
    // 设置默认死区（如果未指定）
    if (app_config.deadzone <= 0.0f)
    {
        app_config.deadzone = 200.0f;
    }
    
    // 初始化麦克纳姆轮底盘
    Mecanum_Chassis_Init(app_config.motors, app_config.motor_directions);
    
    // 初始化夹爪控制（如果提供了串口句柄）
    if (app_config.gripper_uart != NULL)
    {
        RC_GripperInit(app_config.gripper_uart);
    }
    
    is_initialized = 1;
}

/**
 * @brief 麦轮应用主循环更新（1ms周期调用）
 * @param sbus_data SBUS数据结构指针
 * @note 处理遥控器输入、底盘控制、夹爪控制、电机PID更新
 */
void MecanumApp_Update(SBUS_Data_t *sbus_data)
{
    if (!is_initialized || sbus_data == NULL)
    {
        return;
    }
    
    float vx, vy, wz;
    
    // 获取遥控器底盘控制值（包含死区处理）
    RC_GetChassisControl(sbus_data, &vx, &vy, &wz, app_config.deadzone);
    
    // 处理夹爪控制（如果初始化了夹爪）
    if (app_config.gripper_uart != NULL)
    {
        RC_ProcessGripperControl(sbus_data);
    }
    
    // 麦克纳姆轮底盘运动控制
    Mecanum_Chassis_Control(vx, vy, wz);
    
    // 更新电机PID并发送CAN帧（假设是4个电机）
    M2006_UpdateAll(app_config.motors, 4);
}

/**
 * @brief 停止麦轮应用（所有电机停止）
 */
void MecanumApp_Stop(void)
{
    if (!is_initialized)
    {
        return;
    }
    
    // 停止底盘运动
    Mecanum_Chassis_Control(0, 0, 0);
    
    // 更新电机以应用停止指令
    M2006_UpdateAll(app_config.motors, 4);
}