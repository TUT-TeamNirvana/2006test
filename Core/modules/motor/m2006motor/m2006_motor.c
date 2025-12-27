//
// Created by 17087 on 25-11-3.
//
#include "m2006_motor.h"
#include <stdio.h>

// 控制周期定义（单位：秒）
#define M2006_CONTROL_PERIOD_S 0.001f  // 1ms = 0.001s

// M2006编码器参数
#define M2006_ENCODER_RESOLUTION 8192  // 编码器分辨率：0-8191

static M2006_t *motor_list[M2006_MAX_NUM] = {0};

// 反馈频率检测相关变量
static uint32_t feedback_count[M2006_MAX_NUM] = {0};

/* -------------------- 初始化所有电机 -------------------- */
void M2006_InitAll(M2006_t *motors, CAN_HandleTypeDef *hcan)
{
    for (int i = 0; i < M2006_MAX_NUM; i++)
    {
        uint32_t rx_id = 0x201 + i;

        CAN_Init_Config_s config = {
            .can_handle = hcan,
            .tx_id = 0x200,
            .rx_id = rx_id,
            .can_module_callback = M2006_Callback,
            .id = &motors[i]
        };

        motors[i].can = CANRegister(&config);
        motors[i].id = i + 1; // 电机编号（1~4）
        
        // 初始化串级PID控制器
        // 外环（位置环）：PD控制，初始参数 Kp=5.0, Ki=0, Kd=0.1
        // 内环（速度环）：PI控制，使用现有调好的参数 Kp=2.5, Ki=0.011, Kd=0
        // 速度限制：5000 RPM，电流限制：10000 mA
        CascadePID_Init(&motors[i].controller,
                       5.0f, 0.0f, 0.1f,      // 外环PD参数（位置环）
                       2.5f, 0.011f, 0.0f,    // 内环PI参数（速度环）
                       5000.0f, 10000.0f);    // 速度限制和电流限制
        
        // 默认为速度环模式（向后兼容）
        motors[i].mode = M2006_MODE_SPEED;        //速度环M2006_MODE_SPEED,速度环-位置环串级M2006_MODE_CASCADE
        motors[i].controller.mode = CASCADE_MODE_SPEED_ONLY;   //控制器模式，两者模式若不一样则会出现误差无法计算的问题，该参数为对应的枚举
        motors[i].target = 0.0f;
        
        // 初始化反馈数据
        motors[i].feedback.angle_raw = 0;
        motors[i].feedback.angle = 0.0f;
        motors[i].feedback.angle_continuous = 0.0f;
        motors[i].feedback.speed_rpm = 0;
        motors[i].feedback.given_current = 0;
        motors[i].feedback.temp = 0;
        motors[i].feedback.speed_filtered = 0.0f;
        motors[i].feedback.current_filtered = 0.0f;
        
        // 初始化多圈计数
        motors[i].total_angle_raw = 0;
        motors[i].last_angle_raw = 0;
        
        // 前馈参数默认为0（禁用）
        motors[i].controller.inner_loop.Kff = 0.0f;
        motors[i].controller.inner_loop.Kaff = 0.0f;
        
        CANSetDLC(motors[i].can, 8);
        motor_list[i] = &motors[i];
    }
}

/* -------------------- 设置控制模式 -------------------- */
void M2006_SetControlMode(M2006_t *motor, M2006_ControlMode_e mode)
{
    motor->mode = mode;
    CascadePID_SetMode(&motor->controller, (CascadeMode_e)mode);
}

/* -------------------- 设置目标速度（仅速度环模式有效） -------------------- */
void M2006_SetSpeedTarget(M2006_t *motor, float target_rpm)
{
    // 只在速度环模式下有效
    if (motor->mode == M2006_MODE_SPEED) {
        motor->target = target_rpm;
    }
    // 非速度环模式下忽略此设置
}

/* -------------------- 设置目标位置（仅串级模式有效） -------------------- */
void M2006_SetPosTarget(M2006_t *motor, float target_angle)
{
    // 只在串级模式下有效
    if (motor->mode == M2006_MODE_CASCADE) {
        motor->target = target_angle;
    }
    // 非串级模式下忽略此设置
}

/* -------------------- 设置目标（通用接口，任何模式都有效） -------------------- */
void M2006_SetTarget(M2006_t *motor, float target)
{
    // 通用接口，不检查模式，直接设置
    motor->target = target;
}

/* -------------------- 更新所有电机 -------------------- */
void M2006_UpdateAll(M2006_t *motors, uint8_t motor_count)
{
    // 安全检查：确保至少有一个电机且can实例有效
    if (motors == NULL || motor_count == 0 || motors[0].can == NULL)
    {
        return;  // 电机未初始化，直接返回
    }
    
    int16_t currents[4] = {0};

    // 计算每个电机的控制输出
    for (int i = 0; i < motor_count && i < 4; i++)
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
                                        M2006_CONTROL_PERIOD_S);
        
        // 输出限幅
        if (out > 10000.0f) out = 10000.0f;
        if (out < -10000.0f) out = -10000.0f;
        currents[i] = (int16_t)out;
    }

    // 打包发送 0x200 帧（4个电机的电流数据）
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

/* -------------------- CAN反馈回调 -------------------- */
void M2006_Callback(CANInstance *instance)
{
    if (instance == NULL || instance->id == NULL) return;
    
    M2006_t *motor = (M2006_t *)instance->id;
    uint8_t *d = instance->rx_buff;

    // M2006反馈数据格式：
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
    motor->feedback.angle = (float)raw_angle / M2006_ENCODER_RESOLUTION * 360.0f;
    
    // 多圈角度计算（检测过零点）
    int16_t delta_angle = raw_angle - motor->last_angle_raw;
    
    // 检测过零点：如果角度变化超过半圈（4096），说明过零
    if (delta_angle < -M2006_ENCODER_RESOLUTION / 2)
    {
        // 正向过零（从8191跳到0附近）
        motor->total_angle_raw += M2006_ENCODER_RESOLUTION + delta_angle;
    }
    else if (delta_angle > M2006_ENCODER_RESOLUTION / 2)
    {
        // 反向过零（从0跳到8191附近）
        motor->total_angle_raw += delta_angle - M2006_ENCODER_RESOLUTION;
    }
    else
    {
        // 正常情况
        motor->total_angle_raw += delta_angle;
    }
    
    motor->last_angle_raw = raw_angle;
    
    // 计算连续角度（度）
    motor->feedback.angle_continuous = (float)motor->total_angle_raw / M2006_ENCODER_RESOLUTION * 360.0f;

    // 一阶RC低通滤波，压制转速/电流采样噪声
    const float dt = 0.001f;  // 1ms 控制周期
    float alpha_speed   = dt / (M2006_SPEED_LPF_TAU_S   + dt);
    float alpha_current = dt / (M2006_CURRENT_LPF_TAU_S + dt);

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
uint32_t M2006_GetFeedbackFrequency(uint8_t motor_id)
{
    if (motor_id >= M2006_MAX_NUM) return 0;
    
    static uint32_t last_check_time[M2006_MAX_NUM] = {0};
    static uint32_t last_count[M2006_MAX_NUM] = {0};
    static uint32_t frequency[M2006_MAX_NUM] = {0};
    
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