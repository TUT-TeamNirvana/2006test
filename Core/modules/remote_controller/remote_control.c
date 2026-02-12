//
// Created by AI Assistant
// 遥控器控制模块 - 处理SBUS遥控器输入和映射
// 注意：此模块专用于麦轮项目，不适用于轮腿项目
//

#include "remote_control.h"
#include <math.h>
#include "servo_motor_uart.h"
#include "ring_buffer.h"

// 夹爪控制相关私有变量
static const uint16_t GRIPPER_CHANNEL_INDEX = 4; // CH5 -> rc.channels[4]
static const uint16_t GRIPPER_THRESHOLD = 1400;
static uint8_t gripper_state = 0; // 0 = release, 1 = catch
static uint8_t last_button_on = 0;
static uint32_t gripper_last_toggle_ms = 0;
static const uint32_t GRIPPER_TOGGLE_COOLDOWN_MS = 100; // ms

// 夹爪控制命令（麦轮项目专用，非FSUS舵机）
static uint8_t cmd_catch[] = {0x55, 0x55, 0x0B, 0x03, 0x02, 0x64, 0x00, 0x01, 0x90, 0x01, 0x02, 0x58, 0x02};
static uint8_t cmd_realse[] = {0x55, 0x55, 0x0B, 0x03, 0x02, 0x64, 0x00, 0x01, 0x58, 0x02, 0x02, 0x90, 0x01};

// 外部变量声明（麦轮项目的舵机串口，非FSUS）
extern Usart_COB FSUS_usart1;

/**
 * @brief 映射函数 - 将输入值从一个范围映射到另一个范围
 * @param x 输入值
 * @param in_min 输入最小值
 * @param in_max 输入最大值
 * @param out_min 输出最小值
 * @param out_max 输出最大值
 * @return 映射后的输出值（带限幅）
 */
float RC_MapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief 获取处理后的底盘控制值（包含死区处理）
 * @param sbus_data SBUS数据结构指针
 * @param vx 输出X轴速度（前后）
 * @param vy 输出Y轴速度（左右）
 * @param wz 输出旋转角速度
 * @param deadzone 死区阈值（默认200）
 */
void RC_GetChassisControl(SBUS_Data_t *sbus_data, float *vx, float *vy, float *wz, float deadzone)
{
    if (sbus_data == NULL)
    {
        return;
    }
    
    // 摇杆输入映射 (与main.c中的逻辑一致)
    float vx_temp = RC_MapValue(sbus_data->channels[1], 240, 1800, -8000, +8000);  // 前后（通道2）
    float vy_temp = -RC_MapValue(sbus_data->channels[0], 240, 1800, -8000, +8000); // 左右（通道1）
    float wz_temp = RC_MapValue(sbus_data->channels[3], 240, 1800, +8000, -8000);  // 旋转（通道4）
    
    // 死区处理
    if (vx != NULL) *vx = (fabs(vx_temp) < deadzone) ? 0 : vx_temp;
    if (vy != NULL) *vy = (fabs(vy_temp) < deadzone) ? 0 : vy_temp;
    if (wz != NULL) *wz = (fabs(wz_temp) < deadzone) ? 0 : wz_temp;
}

/**
 * @brief 初始化夹爪控制模块（麦轮项目专用）
 */
void RC_GripperInit(void)
{
    gripper_state = 0;
    last_button_on = 0;
    gripper_last_toggle_ms = 0;
}

/**
 * @brief 夹爪控制处理（非阻塞）
 * @param sbus_data SBUS数据结构指针
 * @note 通过CH5通道控制夹爪，使用环形缓冲区非阻塞发送
 * @warning 此函数专用于麦轮项目的非FSUS舵机，不适用于轮腿项目
 */
void RC_ProcessGripperControl(SBUS_Data_t *sbus_data)
{
    if (sbus_data == NULL)
    {
        return;
    }
    
    uint16_t ch_val = sbus_data->channels[GRIPPER_CHANNEL_INDEX]; // CH5
    uint8_t button_on = (ch_val > GRIPPER_THRESHOLD) ? 1 : 0;

    // 上升沿触发 toggle（防抖冷却）
    if (button_on && !last_button_on) 
    {
        uint32_t now = HAL_GetTick();
        if (now - gripper_last_toggle_ms >= GRIPPER_TOGGLE_COOLDOWN_MS) 
        {
            // 选择要发送的帧（当前状态为 release -> 发送 catch）
            uint8_t *frame = (gripper_state == 0) ? cmd_catch : cmd_realse;
            uint16_t frame_len = (gripper_state == 0) ? sizeof(cmd_catch) : sizeof(cmd_realse);

            // 检查环形缓冲剩余空间，避免覆盖/丢失
            if (RingBuffer_GetByteFree(FSUS_usart1.sendBuf) >= frame_len) 
            {
                for (uint16_t i = 0; i < frame_len; ++i) 
                {
                    RingBuffer_Push(FSUS_usart1.sendBuf, frame[i]);
                }
                // 触发非阻塞发送（若 TX 空闲会马上启动，若忙则由 Tx 回调接力）
                Usart_SendAll(&FSUS_usart1);
            }

            // 切换状态并设置冷却时间
            gripper_state = (gripper_state == 0) ? 1 : 0;
            gripper_last_toggle_ms = now;
        }
    }
    last_button_on = button_on;
}