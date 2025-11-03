//
// Created by 17087 on 25-11-3.
//

#ifndef M2006_MOTOR_H
#define M2006_MOTOR_H

#include "main.h"
#include "bsp_can.h"
#include "2006pid.h"

#define M2006_MAX_NUM 2

typedef struct
{
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temp;
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

#endif //M2006_MOTOR_H
