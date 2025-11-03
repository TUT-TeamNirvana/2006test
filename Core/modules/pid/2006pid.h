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
    float integral;
    float last_error;
    float output;
    float output_max;
} PID_t;

void PID_Init(PID_t* pid, float kp, float ki, float kd, float max_output);
float PID_Calc(PID_t* pid, float ref, float feedback);

#endif //M2006PID_H
