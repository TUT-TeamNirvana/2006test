//
// Created by 17087 on 25-11-3.
//
#include "m2006_motor.h"
#include <stdio.h>

// 控制周期定义（单位：秒）
#define M2006_CONTROL_PERIOD_S 0.001f  // 1ms = 0.001s

static M2006_t *motor_list[M2006_MAX_NUM] = {0};

// 反馈频率检测相关变量
static uint32_t feedback_count[M2006_MAX_NUM] = {0};
static uint32_t feedback_last_time[M2006_MAX_NUM] = {0};

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
        PID_Init(&motors[i].pid, 2.5f, 0.011f, 0.0f, 10000.0f);
        motors[i].feedback.speed_filtered = 0.0f;
        motors[i].feedback.current_filtered = 0.0f;
        
        // 设置前馈参数（可根据实际电机特性调整）
        // Kff: 速度前馈增益，单位 mA/RPM
        // Kaff: 加速度前馈增益，单位 mA/(RPM/s)
        // 建议初值：Kff = 0.5~2.0, Kaff = 0.0~0.5
        PID_SetFeedforward(&motors[i].pid, 0.8f, 0.1f);
        
        CANSetDLC(motors[i].can, 8);
        motor_list[i] = &motors[i];
    }
}

/* -------------------- 设置目标转速 -------------------- */
void M2006_SetTarget(M2006_t *motor, float target_rpm)
{
    motor->target_speed = target_rpm;
}

/* -------------------- 设置前馈参数 -------------------- */
void M2006_SetFeedforward(M2006_t *motor, float kff, float kaff)
{
    PID_SetFeedforward(&motor->pid, kff, kaff);
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
        
        // 使用读取到的反馈数据进行PID计算（带前馈）
        // 传入控制周期dt，用于加速度前馈计算
        float out = PID_Calc(&motors[i].pid,
                             motors[i].target_speed,
                             current_feedback,
                             M2006_CONTROL_PERIOD_S);
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

    // 一阶RC低通滤波，压制转速/电流采样噪声
    // 假设控制周期约为 1ms（1kHz），离散化形式：y_k = y_{k-1} + alpha * (x_k - y_{k-1})
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
