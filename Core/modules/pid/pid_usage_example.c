/**
 ******************************************************************************
 * @file    pid_usage_example.c
 * @brief   PID控制器使用示例 - 展示如何独立配置位置环和速度环
 * @note    本文件为示例代码，展示各种功能的配置方法
 ******************************************************************************
 */

#include "cascade_pid.h"
#include "2006pid.h"

/**
 * ============================================================================
 * 示例1：单独的PID控制器配置
 * ============================================================================
 */
void Example1_SinglePID(void)
{
    PID_t speed_pid;
    
    // 1. 初始化PID基本参数
    PID_Init(&speed_pid, 1.5f, 0.1f, 0.05f);
    
    // 2. 配置各项功能（每项功能独立配置）
    
    // 启用输出限幅（电流限制 ±16384）
    PID_ConfigOutputLimit(&speed_pid, true, 16384.0f, -16384.0f);
    
    // 启用积分限幅（积分项限制在 ±500）
    PID_ConfigIntegralLimit(&speed_pid, true, 500.0f, -500.0f);
    
    // 启用抗积分饱和（需要先启用输出限幅）
    PID_ConfigAntiWindup(&speed_pid, true);
    
    // 启用误差死区（小于4.0的误差视为0）
    PID_ConfigErrorDeadband(&speed_pid, true, 4.0f);
    
    // 启用速度前馈（前馈增益0.5）
    PID_ConfigFeedforward(&speed_pid, true, 0.5f);
    
    // 启用加速度前馈（加速度前馈增益0.02）
    PID_ConfigAccelFeedforward(&speed_pid, true, 0.02f);
    
    // 启用微分滤波（滤波系数0.3，值越大滤波越强）
    PID_ConfigDerivativeFilter(&speed_pid, true, 0.3f);
    
    // 3. 使用PID计算
    float target_speed = 100.0f;     // 目标速度 100 RPM
    float actual_speed = 95.0f;      // 实际速度 95 RPM
    float dt = 0.001f;               // 控制周期 1ms
    
    float output = PID_Calc(&speed_pid, target_speed, actual_speed, dt);
    
    // output 即为电流输出（mA）
}


/**
 * ============================================================================
 * 示例2：串级PID配置 - 位置环和速度环使用不同的功能参数
 * ============================================================================
 */
void Example2_CascadePID_DifferentConfig(void)
{
    CascadePID_t motor_cascade;
    
    // ===== 第一步：初始化基本PID参数 =====
    CascadePID_Init(&motor_cascade,
                    2.0f, 0.0f, 0.1f,    // 外环（位置环）：PD控制
                    1.5f, 0.1f, 0.0f);   // 内环（速度环）：PI控制
    
    // ===== 第二步：配置位置环（外环）功能 =====
    
    // 位置环输出限幅：±200 RPM
    CascadePID_ConfigOuterOutputLimit(&motor_cascade, true, 200.0f, -200.0f);
    
    // 位置环积分限幅：±100（虽然位置环通常不用积分，但作为示例）
    CascadePID_ConfigOuterIntegralLimit(&motor_cascade, true, 100.0f, -100.0f);
    
    // 位置环抗积分饱和：启用
    CascadePID_ConfigOuterAntiWindup(&motor_cascade, true);
    
    // 位置环误差死区：±2.0度
    CascadePID_ConfigOuterErrorDeadband(&motor_cascade, true, 2.0f);
    
    // 位置环微分滤波：启用，滤波系数0.5
    CascadePID_ConfigOuterDerivativeFilter(&motor_cascade, true, 0.5f);
    
    // 位置环不启用前馈
    CascadePID_ConfigOuterFeedforward(&motor_cascade, false, 0.0f);
    
    
    // ===== 第三步：配置速度环（内环）功能 - 与位置环参数不同 =====
    
    // 速度环输出限幅：±16384 mA（与位置环不同！）
    CascadePID_ConfigInnerOutputLimit(&motor_cascade, true, 16384.0f, -16384.0f);
    
    // 速度环积分限幅：±500（与位置环不同！）
    CascadePID_ConfigInnerIntegralLimit(&motor_cascade, true, 500.0f, -500.0f);
    
    // 速度环抗积分饱和：启用
    CascadePID_ConfigInnerAntiWindup(&motor_cascade, true);
    
    // 速度环误差死区：±4.0 RPM（与位置环不同！）
    CascadePID_ConfigInnerErrorDeadband(&motor_cascade, true, 4.0f);
    
    // 速度环启用速度前馈：增益0.5
    CascadePID_ConfigInnerFeedforward(&motor_cascade, true, 0.5f);
    
    // 速度环启用加速度前馈：增益0.02
    CascadePID_ConfigInnerAccelFeedforward(&motor_cascade, true, 0.02f);
    
    // 速度环微分滤波：启用，滤波系数0.3（与位置环不同！）
    CascadePID_ConfigInnerDerivativeFilter(&motor_cascade, true, 0.3f);
    
    
    // ===== 第四步：设置控制模式并使用 =====
    
    // 设置为串级模式
    CascadePID_SetMode(&motor_cascade, CASCADE_MODE_CASCADE);
    
    // 执行串级PID计算
    float target_position = 180.0f;   // 目标位置 180度
    float actual_position = 175.0f;   // 实际位置 175度
    float actual_speed = 50.0f;       // 实际速度 50 RPM
    float dt = 0.001f;                // 控制周期 1ms
    
    float output = CascadePID_Calculate(&motor_cascade, 
                                       target_position, 
                                       actual_position, 
                                       actual_speed, 
                                       dt);
    
    // output 即为电流输出（mA）
}


/**
 * ============================================================================
 * 示例3：只使用速度环模式
 * ============================================================================
 */
void Example3_SpeedOnlyMode(void)
{
    CascadePID_t motor_cascade;
    
    // 1. 初始化
    CascadePID_Init(&motor_cascade,
                    2.0f, 0.0f, 0.1f,    // 外环参数（速度模式下不使用）
                    1.5f, 0.1f, 0.0f);   // 内环参数（速度环）
    
    // 2. 只配置速度环（内环）
    CascadePID_ConfigInnerOutputLimit(&motor_cascade, true, 16384.0f, -16384.0f);
    CascadePID_ConfigInnerIntegralLimit(&motor_cascade, true, 500.0f, -500.0f);
    CascadePID_ConfigInnerAntiWindup(&motor_cascade, true);
    CascadePID_ConfigInnerErrorDeadband(&motor_cascade, true, 4.0f);
    CascadePID_ConfigInnerFeedforward(&motor_cascade, true, 0.5f);
    
    // 3. 设置为速度模式
    CascadePID_SetMode(&motor_cascade, CASCADE_MODE_SPEED_ONLY);
    
    // 4. 计算（target直接作为速度目标，actual_pos参数被忽略）
    float target_speed = 100.0f;
    float actual_speed = 95.0f;
    float dt = 0.001f;
    
    float output = CascadePID_Calculate(&motor_cascade, 
                                       target_speed,    // 直接设置速度目标
                                       0.0f,            // 位置参数被忽略
                                       actual_speed, 
                                       dt);
}


/**
 * ============================================================================
 * 示例4：运行时动态调整功能参数
 * ============================================================================
 */
void Example4_RuntimeAdjust(void)
{
    CascadePID_t motor_cascade;
    
    // 初始化
    CascadePID_Init(&motor_cascade,
                    2.0f, 0.0f, 0.1f,
                    1.5f, 0.1f, 0.0f);
    
    // 初始配置
    CascadePID_ConfigInnerOutputLimit(&motor_cascade, true, 16384.0f, -16384.0f);
    CascadePID_ConfigInnerIntegralLimit(&motor_cascade, true, 500.0f, -500.0f);
    
    // 运行一段时间后...
    
    // 动态调整积分限幅（比如发现积分饱和问题）
    CascadePID_ConfigInnerIntegralLimit(&motor_cascade, true, 300.0f, -300.0f);
    
    // 动态启用前馈
    CascadePID_ConfigInnerFeedforward(&motor_cascade, true, 0.5f);
    
    // 动态禁用误差死区
    CascadePID_ConfigInnerErrorDeadband(&motor_cascade, false, 0.0f);
    
    // 动态修改PID参数（直接访问）
    motor_cascade.inner_loop.params.Kp = 2.0f;
    motor_cascade.inner_loop.params.Ki = 0.15f;
}


/**
 * ============================================================================
 * 示例5：使用便捷宏直接访问内部PID
 * ============================================================================
 */
void Example5_DirectAccess(void)
{
    CascadePID_t motor_cascade;
    
    CascadePID_Init(&motor_cascade,
                    2.0f, 0.0f, 0.1f,
                    1.5f, 0.1f, 0.0f);
    
    // 使用宏获取PID指针
    PID_t *outer_pid = CASCADE_GET_OUTER_PID(&motor_cascade);
    PID_t *inner_pid = CASCADE_GET_INNER_PID(&motor_cascade);
    
    // 直接配置
    PID_ConfigOutputLimit(inner_pid, true, 16384.0f, -16384.0f);
    PID_ConfigIntegralLimit(inner_pid, true, 500.0f, -500.0f);
    PID_ConfigAntiWindup(inner_pid, true);
    
    // 读取状态
    float current_integral = inner_pid->state.integral;
    float current_output = inner_pid->state.output;
    
    // 直接修改参数
    inner_pid->params.Kp = 2.0f;
    inner_pid->config.integral_max = 600.0f;
}


/**
 * ============================================================================
 * 示例6：M2006电机完整配置示例
 * ============================================================================
 */
void Example6_M2006_Complete(void)
{
    CascadePID_t m2006_cascade;
    
    // ===== 初始化：M2006典型参数 =====
    CascadePID_Init(&m2006_cascade,
                    2.0f, 0.0f, 0.1f,    // 位置环：PD控制
                    1.5f, 0.1f, 0.0f);   // 速度环：PI控制
    
    // ===== 位置环配置 =====
    CascadePID_ConfigOuterOutputLimit(&m2006_cascade, true, 200.0f, -200.0f);     // 速度限制±200 RPM
    CascadePID_ConfigOuterIntegralLimit(&m2006_cascade, true, 50.0f, -50.0f);     // 积分限幅（通常位置环不用积分）
    CascadePID_ConfigOuterAntiWindup(&m2006_cascade, true);                        // 抗积分饱和
    CascadePID_ConfigOuterErrorDeadband(&m2006_cascade, true, 2.0f);               // 误差死区±2度
    CascadePID_ConfigOuterDerivativeFilter(&m2006_cascade, true, 0.5f);            // 微分滤波
    
    // ===== 速度环配置 =====
    CascadePID_ConfigInnerOutputLimit(&m2006_cascade, true, 16384.0f, -16384.0f); // 电流限制±16384
    CascadePID_ConfigInnerIntegralLimit(&m2006_cascade, true, 500.0f, -500.0f);   // 积分限幅±500
    CascadePID_ConfigInnerAntiWindup(&m2006_cascade, true);                        // 抗积分饱和（关键！）
    CascadePID_ConfigInnerErrorDeadband(&m2006_cascade, true, 4.0f);               // 误差死区±4 RPM
    CascadePID_ConfigInnerFeedforward(&m2006_cascade, true, 0.5f);                 // 速度前馈
    CascadePID_ConfigInnerAccelFeedforward(&m2006_cascade, true, 0.02f);           // 加速度前馈
    CascadePID_ConfigInnerDerivativeFilter(&m2006_cascade, false, 0.0f);           // 速度环通常不需要微分滤波
    
    // ===== 使用 =====
    CascadePID_SetMode(&m2006_cascade, CASCADE_MODE_CASCADE);
    
    // 在1ms控制周期中调用
    float output = CascadePID_Calculate(&m2006_cascade, 
                                       180.0f,    // 目标角度
                                       175.0f,    // 当前角度
                                       50.0f,     // 当前转速
                                       0.001f);   // 1ms
}


/**
 * ============================================================================
 * 示例7：验证位置环和速度环参数独立性
 * ============================================================================
 */
void Example7_VerifyIndependence(void)
{
    CascadePID_t cascade;
    
    CascadePID_Init(&cascade, 1.0f, 0.0f, 0.1f, 1.0f, 0.1f, 0.0f);
    
    // 位置环：积分限幅 ±100
    CascadePID_ConfigOuterIntegralLimit(&cascade, true, 100.0f, -100.0f);
    
    // 速度环：积分限幅 ±500（不同的值！）
    CascadePID_ConfigInnerIntegralLimit(&cascade, true, 500.0f, -500.0f);
    
    // 验证：两个环的配置确实不同
    // 位置环的积分最大值是100
    // 速度环的积分最大值是500
    // 它们互不影响！
    
    float outer_int_max = cascade.outer_loop.config.integral_max;  // 应该是 100.0
    float inner_int_max = cascade.inner_loop.config.integral_max;  // 应该是 500.0
    
    // 修改位置环不会影响速度环
    CascadePID_ConfigOuterIntegralLimit(&cascade, true, 200.0f, -200.0f);
    
    // inner_int_max 仍然是 500.0，不受影响
}


/**
 * ============================================================================
 * 示例8：只启用必要功能，其他功能禁用
 * ============================================================================
 */
void Example8_MinimalConfig(void)
{
    PID_t simple_pid;
    
    // 初始化（所有功能默认禁用）
    PID_Init(&simple_pid, 1.0f, 0.1f, 0.05f);
    
    // 只启用输出限幅和抗积分饱和
    PID_ConfigOutputLimit(&simple_pid, true, 10000.0f, -10000.0f);
    PID_ConfigAntiWindup(&simple_pid, true);
    
    // 其他功能保持禁用状态：
    // - 前馈：禁用
    // - 积分限幅：禁用
    // - 微分滤波：禁用
    // - 误差死区：禁用
    
    // 这样配置最简洁，只使用必要的功能
}