//
// Created by 17087 on 25-11-3.
//
#include "m2006_motor.h"
#include <stdio.h>

static M2006_t *motor_list[M2006_MAX_NUM] = {0};

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
        motors[i].target_speed = 0;
        // 初始化反馈数据为0
        /*motors[i].feedback.speed_rpm = 0;
        motors[i].feedback.given_current = 0;
        motors[i].feedback.temp = 0;*/
        PID_Init(&motors[i].pid, 4.0f, 0.01f, 0.0f, 10000.0f);
        motors[i].feedback.speed_filtered = 0.0f;
        motors[i].feedback.current_filtered = 0.0f;
        CANSetDLC(motors[i].can, 8);
        motor_list[i] = &motors[i];
    }
}

/* -------------------- 设置目标转速 -------------------- */
void M2006_SetTarget(M2006_t *motor, float target_rpm)
{
    motor->target_speed = target_rpm;
}

/* -------------------- PID计算并统一发送 -------------------- */
void M2006_UpdateAll(M2006_t *motors, uint8_t motor_count)
{
    // 安全检查：确保至少有一个电机且can实例有效
    if (motors == NULL || motor_count == 0 || motors[0].can == NULL)
    {
        return;  // 电机未初始化，直接返回
    }
    
    int16_t currents[4] = {0};

    // 计算每个电机的PID输出电流
    // 关键修复：先读取反馈数据到局部变量，确保整个PID计算过程中使用同一份数据
    // 这样可以避免CAN中断在PID计算过程中更新反馈数据导致的不一致问题
    for (int i = 0; i < motor_count && i < 4; i++)
    {
        // 禁用中断，原子读取反馈数据
        __disable_irq();
        float current_feedback = motors[i].feedback.speed_filtered;
        __enable_irq();
        
        // 使用读取到的反馈数据进行PID计算
        float out = PID_Calc(&motors[i].pid,
                             motors[i].target_speed,
                             current_feedback);
        if (out > 10000) out = 10000;
        if (out < -10000) out = -10000;
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
    // d[0-1]: 转子机械角度（未使用）
    // d[2-3]: 转子转速 (rpm) - d[2]为高字节，d[3]为低字节
    // d[4-5]: 转矩电流 (mA) - d[4]为高字节，d[5]为低字节
    // d[6]: 温度
    // d[7]: 保留
    int16_t raw_speed   = (int16_t)((d[2] << 8) | d[3]);
    int16_t raw_current = (int16_t)((d[4] << 8) | d[5]);

    motor->feedback.speed_rpm     = raw_speed;
    motor->feedback.given_current = raw_current;
    motor->feedback.temp          = d[6];

    // 一阶低通滤波，压制转速/电流采样噪声
    motor->feedback.speed_filtered =
        (1.0f - M2006_SPEED_SMOOTH_COEF) * motor->feedback.speed_filtered +
        M2006_SPEED_SMOOTH_COEF * (float)raw_speed;

    motor->feedback.current_filtered =
        (1.0f - M2006_CURRENT_SMOOTH_COEF) * motor->feedback.current_filtered +
        M2006_CURRENT_SMOOTH_COEF * (float)raw_current;
}
