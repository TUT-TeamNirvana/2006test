//
// Created by 17087 on 25-11-3.
//

#ifndef M2006_MOTOR_H
#define M2006_MOTOR_H

#include "main.h"
#include "bsp_can.h"
#include "2006pid.h"

// RC一阶低通滤波时间常数（单位：秒）
// 时间常数越小，滤波越轻，响应越快；时间常数越大，滤波越重，响应越慢
#define M2006_SPEED_LPF_TAU_S     0.010f  // 10ms，对应截止频率约16Hz
#define M2006_CURRENT_LPF_TAU_S   0.010f  // 10ms，对应截止频率约16Hz

#define M2006_MAX_NUM 2

typedef struct
{
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temp;
    float   speed_filtered;    // 一阶低通后的转速
    float   current_filtered;  // 一阶低通后的电流
} M2006_Feedback_t;

typedef struct
{
    CANInstance *can;
    M2006_Feedback_t feedback;
    PID_t pid;
    float target_speed;
    uint8_t id;
} M2006_t;

void M2006_InitAll(M2006_t *motors, CAN_HandleTypeDef *hcan);
void M2006_SetTarget(M2006_t *motor, float target_rpm);
void M2006_UpdateAll(M2006_t *motors, uint8_t motor_count);
void M2006_Callback(CANInstance *instance);
uint32_t M2006_GetFeedbackFrequency(uint8_t motor_id);  // 获取反馈频率（Hz）

#endif //M2006_MOTOR_H
