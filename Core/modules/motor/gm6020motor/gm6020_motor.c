//
// Created by 17087 on 25-11-3.
//
#include "gm6020_motor.h"
#include <stdio.h>

// 控制周期定义（单位：秒）
#define GM6020_CONTROL_PERIOD_S 0.001f  // 1ms = 0.001s

// GM6020编码器参数
#define GM6020_ENCODER_RESOLUTION 8192  // 编码器分辨率：0-8191

static GM6020_t *motor_list[GM6020_MAX_NUM] = {0};

// 反馈频率检测相关变量
static uint32_t feedback_count[GM6020_MAX_NUM] = {0};

/* -------------------- 初始化单个电机 -------------------- */
void GM6020_InitSingle(GM6020_t *motor, CAN_HandleTypeDef *hcan, uint8_t motor_id,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd,
                     float speed_limit, float current_limit)
{
    uint32_t rx_id = 0x205 + motor_id;

    CAN_Init_Config_s config = {
        .can_handle = hcan,
        .tx_id = (motor_id < 4) ? 0x1FF : 0x2FF,
        .rx_id = rx_id,
        .can_module_callback = GM6020_Callback,
        .id = motor
    };

    motor->can = CANRegister(&config);
    motor->id = motor_id + 1; // 电机编号（1~4）
    
    // 初始化串级PID控制器（每个电机独立参数）
    CascadePID_Init(&motor->controller,
                   outer_kp, outer_ki, outer_kd,  // 外环PD参数（位置环）
                   inner_kp, inner_ki, inner_kd,  // 内环PI参数（速度环）
                   speed_limit, current_limit);   // 速度限制和电流限制

    //内环（速度环）PID初始化
    CascadePID_ConfigInnerFeatures(&motor->controller, true, true, false, false, false, false);
    CascadePID_SetInnerParams(&motor->controller, 2.0f, 0.0f);

    //外环（位置环）PID初始化
    CascadePID_ConfigOuterFeatures(&motor->controller, true, false, true, true, false, false);
    CascadePID_SetOuterParams(&motor->controller, 0.1f, 0.0f);

    // 默认为速度环模式（向后兼容）
    motor->mode = GM6020_MODE_SPEED;
    motor->controller.mode = CASCADE_MODE_SPEED_ONLY;
    motor->target = 0.0f;
    
    // 初始化反馈数据
    motor->feedback.angle_raw = 0;
    motor->feedback.angle = 0.0f;
    motor->feedback.angle_continuous = 0.0f;
    motor->feedback.speed_rpm = 0;
    motor->feedback.given_current = 0;
    motor->feedback.temp = 0;
    motor->feedback.speed_filtered = 0.0f;
    motor->feedback.current_filtered = 0.0f;
    
    // 初始化多圈计数
    motor->total_angle_raw = 0;
    motor->last_angle_raw = 0;
    
    CANSetDLC(motor->can, 8);
    motor_list[motor_id] = motor;
}

/* -------------------- 初始化所有电机（使用默认参数） -------------------- */
void GM6020_InitAll(GM6020_t *motors, CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < GM6020_MAX_NUM; i++)
    {
        // 使用默认参数初始化（建议后续为每个电机单独调参）
        GM6020_InitSingle(&motors[i], hcan, i,
                        5.0f, 0.0f, 0.1f,      // 外环PD参数（位置环）
                        2.5f, 0.011f, 0.0f,    // 内环PI参数（速度环）
                        300.0f, 30000.0f);     // 速度限制和电流限制
    }
}

/* -------------------- 设置控制模式 -------------------- */
void GM6020_SetControlMode(GM6020_t *motor, GM6020_ControlMode_e mode)
{
    motor->mode = mode;
    CascadePID_SetMode(&motor->controller, (CascadeMode_e)mode);
}

/* -------------------- 设置目标速度（仅速度环模式有效） -------------------- */
void GM6020_SetSpeedTarget(GM6020_t *motor, float target_rpm)
{
    // 只在速度环模式下有效
    if (motor->mode == GM6020_MODE_SPEED) {
        motor->target = target_rpm;
    }
}

/* -------------------- 设置目标位置（仅串级模式有效） -------------------- */
void GM6020_SetPosTarget(GM6020_t *motor, float target_angle)
{
    // 只在串级模式下有效
    if (motor->mode == GM6020_MODE_CASCADE) {
        motor->target = target_angle;
    }
}

/* -------------------- 设置目标（通用接口，任何模式都有效） -------------------- */
void GM6020_SetTarget(GM6020_t *motor, float target)
{
    motor->target = target;
}

/* -------------------- 更新所有电机 -------------------- */
void GM6020_UpdateAll(GM6020_t *motors, uint8_t motor_count)
{
    // 安全检查：确保至少有一个电机且can实例有效
    if (motors == NULL || motor_count == 0 || motors[0].can == NULL)
    {
        return;  // 电机未初始化，直接返回
    }
    
    int16_t currents[8] = {0};

    // 计算每个电机的控制输出
    for (int i = 0; i < motor_count && i < 8; i++)
    {
        // 禁用中断，原子读取反馈数据
        __disable_irq();
        float current_pos = motors[i].feedback.angle_continuous;
        float current_speed = motors[i].feedback.speed_filtered;
        __enable_irq();
        
        // 根据当前模式计算输出
        float out = CascadePID_Calculate(&motors[i].controller,
                                        motors[i].target,
                                        current_pos,
                                        current_speed,
                                        GM6020_CONTROL_PERIOD_S);
        
        // 输出限幅
        if (out > 30000.0f) out = 30000.0f;
        if (out < -30000.0f) out = -30000.0f;
        currents[i] = (int16_t)out;
    }

    // 如果有1-4号电机，打包发送 0x1FF 帧
    if (motor_count > 0 && motors[0].can != NULL) {
        motors[0].can->tx_buff[0] = (currents[0] >> 8) & 0xFF;
        motors[0].can->tx_buff[1] = (currents[0]) & 0xFF;
        motors[0].can->tx_buff[2] = (currents[1] >> 8) & 0xFF;
        motors[0].can->tx_buff[3] = (currents[1]) & 0xFF;
        motors[0].can->tx_buff[4] = (currents[2] >> 8) & 0xFF;
        motors[0].can->tx_buff[5] = (currents[2]) & 0xFF;
        motors[0].can->tx_buff[6] = (currents[3] >> 8) & 0xFF;
        motors[0].can->tx_buff[7] = (currents[3]) & 0xFF;

        CANTransmit(motors[0].can, 2);
    }
    
    // 如果有5-8号电机，打包发送 0x2FF 帧
    if (motor_count > 4 && motors[4].can != NULL) {
        motors[4].can->tx_buff[0] = (currents[4] >> 8) & 0xFF;
        motors[4].can->tx_buff[1] = (currents[4]) & 0xFF;
        motors[4].can->tx_buff[2] = (currents[5] >> 8) & 0xFF;
        motors[4].can->tx_buff[3] = (currents[5]) & 0xFF;
        motors[4].can->tx_buff[4] = (currents[6] >> 8) & 0xFF;
        motors[4].can->tx_buff[5] = (currents[6]) & 0xFF;
        motors[4].can->tx_buff[6] = (currents[7] >> 8) & 0xFF;
        motors[4].can->tx_buff[7] = (currents[7]) & 0xFF;

        CANTransmit(motors[4].can, 2);
    }
}

/* -------------------- CAN反馈回调 -------------------- */
void GM6020_Callback(CANInstance *instance)
{
    if (instance == NULL || instance->id == NULL) return;
    
    GM6020_t *motor = (GM6020_t *)instance->id;
    uint8_t *d = instance->rx_buff;

    // GM6020反馈数据格式：
    // d[0-1]: 转子机械角度 (0-8191) - d[0]为高字节，d[1]为低字节
    // d[2-3]: 转子转速 (rpm) - d[2]为高字节，d[3]为低字节
    // d[4-5]: 转矩电流 (mA) - d[4]为高字节，d[5]为低字节
    // d[6]: 温度
    // d[7]: 保留
    
    int16_t raw_angle   = (int16_t)((d[0] << 8) | d[1]);
    int16_t raw_speed   = (int16_t)((d[2] << 8) | d[3]);
    int16_t raw_current = (int16_t)((d[4] << 8) | d[5]);

    // 更新原始数据
    motor->feedback.angle_raw = raw_angle;
    motor->feedback.speed_rpm = raw_speed;
    motor->feedback.given_current = raw_current;
    motor->feedback.temp = d[6];
    
    // 计算当前角度（0-360度）
    motor->feedback.angle = (float)raw_angle / GM6020_ENCODER_RESOLUTION * 360.0f;
    
    // 多圈角度计算（检测过零点）
    int16_t delta_angle = raw_angle - motor->last_angle_raw;
    
    // 检测过零点：如果角度变化超过半圈（4096），说明过零
    if (delta_angle < -GM6020_ENCODER_RESOLUTION / 2)
    {
        // 正向过零（从8191跳到0附近）
        motor->total_angle_raw += GM6020_ENCODER_RESOLUTION + delta_angle;
    }
    else if (delta_angle > GM6020_ENCODER_RESOLUTION / 2)
    {
        // 反向过零（从0跳到8191附近）
        motor->total_angle_raw += delta_angle - GM6020_ENCODER_RESOLUTION;
    }
    else
    {
        // 正常情况
        motor->total_angle_raw += delta_angle;
    }
    
    motor->last_angle_raw = raw_angle;
    
    // 计算连续角度（度）
    motor->feedback.angle_continuous = (float)motor->total_angle_raw / GM6020_ENCODER_RESOLUTION * 360.0f;

    // 一阶RC低通滤波，压制转速/电流采样噪声
    const float dt = 0.001f;  // 1ms 控制周期
    float alpha_speed   = dt / (GM6020_SPEED_LPF_TAU_S   + dt);
    float alpha_current = dt / (GM6020_CURRENT_LPF_TAU_S + dt);

    motor->feedback.speed_filtered +=
        alpha_speed * ((float)raw_speed - motor->feedback.speed_filtered);

    motor->feedback.current_filtered +=
        alpha_current * ((float)raw_current - motor->feedback.current_filtered);
    
    // 反馈频率检测（仅对第一个电机统计）
    if (motor->id == 1) {
        feedback_count[0]++;
    }
}

/* -------------------- 获取反馈频率 -------------------- */
uint32_t GM6020_GetFeedbackFrequency(uint8_t motor_id)
{
    if (motor_id >= GM6020_MAX_NUM) return 0;
    
    static uint32_t last_check_time[GM6020_MAX_NUM] = {0};
    static uint32_t last_count[GM6020_MAX_NUM] = {0};
    static uint32_t frequency[GM6020_MAX_NUM] = {0};
    
    uint32_t now = HAL_GetTick();
    
    // 每秒更新一次频率
    if (now - last_check_time[motor_id] >= 1000) {
        uint32_t count_diff = feedback_count[motor_id] - last_count[motor_id];
        frequency[motor_id] = count_diff;  // 每秒的反馈次数 = 频率（Hz）
        last_count[motor_id] = feedback_count[motor_id];
        last_check_time[motor_id] = now;
    }
    
    return frequency[motor_id];
}
