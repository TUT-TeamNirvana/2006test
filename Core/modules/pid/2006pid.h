//
// Created by 17087 on 25-11-2.
//

#ifndef M2006PID_H
#define M2006PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kff;           // 速度前馈增益
    float Kaff;          // 加速度前馈增益
    float integral;
    float last_error;
    float last_ref;      // 上一次的目标值，用于计算加速度前馈
    float output;
    float output_max;
} PID_t;

void PID_Init(PID_t* pid, float kp, float ki, float kd, float max_output);
void PID_SetFeedforward(PID_t* pid, float kff, float kaff);
float PID_Calc(PID_t* pid, float ref, float feedback, float dt);

#endif //M2006PID_H
