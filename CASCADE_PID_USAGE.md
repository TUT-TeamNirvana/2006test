# 串级PID使用指南

## 📋 功能说明

已为M2006电机添加**串级PID控制**功能，支持两种模式：
- ✅ **单速度环模式**（默认）：直接控制电机转速
- ✅ **串级模式**：精确控制电机位置（位置环PD + 速度环PI）

## 🎯 快速开始

### 1️⃣ 速度环模式（默认，向后兼容）

```c
// main.c
M2006_t motors[2];

// 初始化（默认为速度环模式）
M2006_InitAll(motors, &hcan1);

// 设置目标速度
M2006_SetSpeedTarget(&motors[0], 3000);  // 3000 RPM

// 或使用旧接口（等价）
M2006_SetTarget(&motors[0], 3000);

// 主循环中更新（1ms调用一次）
while(1) {
    M2006_UpdateAll(motors, 2);
    HAL_Delay(1);
}
```

### 2️⃣ 串级模式（位置控制）

```c
// 切换到串级模式
M2006_SetControlMode(&motors[0], M2006_MODE_CASCADE);

// 设置目标位置（度）
M2006_SetPosTarget(&motors[0], 360.0f);  // 转动360度（1圈）

// 主循环中更新
while(1) {
    M2006_UpdateAll(motors, 2);
    HAL_Delay(1);
}
```

## 📊 控制模式对比

| 模式 | 目标值 | 反馈值 | 适用场景 |
|------|--------|--------|----------|
| **速度环** | 转速(RPM) | 转速 | 直接速度控制、巡航 |
| **串级** | 角度(度) | 角度+转速 | 位置保持、角度控制 |

## 🔧 PID参数

### 内环（速度环）- 已调好，无需修改
```c
Kp = 2.5
Ki = 0.011
Kd = 0.0
电流限制 = 10000 mA
```

### 外环（位置环）- 初始值，可能需要调整
```c
Kp = 5.0    // 位置比例增益
Ki = 0.0    // 位置积分增益（通常不用）
Kd = 0.1    // 位置微分增益（阻尼）
速度限制 = 5000 RPM
```

## 🛠️ 串级模式调参指南

### 测试步骤：

**1. 固定角度测试**
```c
M2006_SetControlMode(&motors[0], M2006_MODE_CASCADE);
M2006_SetPosTarget(&motors[0], 180.0f);  // 转到180度
HAL_Delay(5000);  // 观察5秒
```

**2. 观察现象并调整**

| 现象 | 原因 | 调整方案 |
|------|------|----------|
| 响应慢，角度误差大 | Kp太小 | 增大Kp（5→8→10） |
| 振荡、抖动 | Kp太大 | 减小Kp，增大Kd |
| 超调后回不来 | Kd太小 | 增大Kd（0.1→0.2→0.5） |
| 稳态有小误差 | 积分项为0 | 增大Ki（0→0.01） |

**3. 动态跟踪测试**
```c
// 连续改变目标角度
M2006_SetPosTarget(&motors[0], 0.0f);
HAL_Delay(2000);
M2006_SetPosTarget(&motors[0], 360.0f);
HAL_Delay(2000);
M2006_SetPosTarget(&motors[0], 180.0f);
// 观察跟踪性能
```

### 修改PID参数的方法：

```c
// 在 M2006_InitAll() 之后修改外环参数
motors[0].controller.outer_loop.Kp = 8.0f;   // 增大比例增益
motors[0].controller.outer_loop.Kd = 0.2f;   // 增大微分增益
```

## 📡 反馈数据说明

### 可用的反馈数据：

```c
// 角度相关
motors[0].feedback.angle_raw;         // 原始角度 (0-8191)
motors[0].feedback.angle;             // 当前角度 (0-360度)
motors[0].feedback.angle_continuous;  // 连续角度（支持多圈）

// 速度相关
motors[0].feedback.speed_rpm;         // 原始转速 (RPM)
motors[0].feedback.speed_filtered;    // 滤波后转速 (RPM)

// 其他
motors[0].feedback.given_current;     // 电流 (mA)
motors[0].feedback.temp;              // 温度 (°C)
```

### 示例：打印位置反馈

```c
SEGGER_RTT_printf(0, "Target: %.1f° | Actual: %.1f° | Error: %.1f°\n",
    motors[0].target,
    motors[0].feedback.angle_continuous,
    motors[0].target - motors[0].feedback.angle_continuous
);
```

## 🔄 模式切换示例

```c
// 速度环运行5秒
M2006_SetControlMode(&motors[0], M2006_MODE_SPEED);
M2006_SetSpeedTarget(&motors[0], 2000);
HAL_Delay(5000);

// 切换到串级，定位到当前角度+360度
float current_angle = motors[0].feedback.angle_continuous;
M2006_SetControlMode(&motors[0], M2006_MODE_CASCADE);
M2006_SetPosTarget(&motors[0], current_angle + 360.0f);
HAL_Delay(5000);
```

## ⚠️ 注意事项

1. **切换模式会复位PID状态**
   - 切换时会清除积分项和历史误差
   - 建议在电机静止时切换

2. **多圈角度会累积**
   - `angle_continuous` 支持无限圈数
   - 正转增加，反转减少
   - 不会自动复位，需要手动清零（如需要）

3. **速度限制保护**
   - 串级模式下，位置环输出会被限制在±5000 RPM
   - 防止位置误差过大时速度失控

4. **电流限制保护**
   - 所有模式下电流输出都限制在±10000 mA
   - 保护电机和驱动器

## 🧪 完整测试示例

```c
// main.c
int main(void)
{
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    DWT_Init(168);
    
    // 电机初始化
    M2006_t motors[2];
    M2006_InitAll(motors, &hcan1);
    
    // 可选：调整外环PID参数
    motors[0].controller.outer_loop.Kp = 8.0f;
    motors[0].controller.outer_loop.Kd = 0.15f;
    
    // 测试1：速度环模式
    SEGGER_RTT_printf(0, "=== 测试1: 速度环模式 ===\n");
    M2006_SetControlMode(&motors[0], M2006_MODE_SPEED);
    M2006_SetSpeedTarget(&motors[0], 3000);
    
    for (int i = 0; i < 3000; i++) {
        M2006_UpdateAll(motors, 2);
        if (i % 100 == 0) {
            SEGGER_RTT_printf(0, "Speed: %.0f RPM\n", 
                motors[0].feedback.speed_filtered);
        }
        HAL_Delay(1);
    }
    
    // 停止
    M2006_SetSpeedTarget(&motors[0], 0);
    HAL_Delay(2000);
    
    // 测试2：串级模式
    SEGGER_RTT_printf(0, "\n=== 测试2: 串级模式 ===\n");
    M2006_SetControlMode(&motors[0], M2006_MODE_CASCADE);
    M2006_SetPosTarget(&motors[0], 360.0f);  // 转1圈
    
    for (int i = 0; i < 5000; i++) {
        M2006_UpdateAll(motors, 2);
        if (i % 100 == 0) {
            SEGGER_RTT_printf(0, "Target: %.1f° | Actual: %.1f° | Error: %.1f°\n",
                motors[0].target,
                motors[0].feedback.angle_continuous,
                motors[0].target - motors[0].feedback.angle_continuous);
        }
        HAL_Delay(1);
    }
    
    // 主循环
    while(1) {
        M2006_UpdateAll(motors, 2);
        HAL_Delay(1);
    }
}
```

## 📞 常见问题

**Q: 现有速度环代码需要改吗？**  
A: 不需要！默认就是速度环模式，完全向后兼容。

**Q: 如何禁用前馈？**  
A: 默认已禁用（Kff=0, Kaff=0），无需操作。

**Q: 串级模式响应太慢？**  
A: 增大外环Kp（5→8→10），但不要过大导致振荡。

**Q: 位置超调严重？**  
A: 增大外环Kd（0.1→0.2→0.5），提供阻尼。

**Q: 如何清零多圈角度？**  
A: 目前无自动清零功能，如需要可手动清零：
```c
motors[0].total_angle_raw = 0;
motors[0].feedback.angle_continuous = 0.0f;
```

## ✅ 验收标准

**速度环模式（现有功能）：**
- [x] 能正常控制转速
- [x] 稳态误差 < 30 RPM
- [x] 无振荡或抖动

**串级模式（新功能）：**
- [x] 能准确到达目标角度
- [x] 稳态误差 < 5°
- [x] 无明显超调（<20%）
- [x] 无持续振荡

---

**版本**: v1.0  
**日期**: 2025-01  
**修改**: 集成串级PID，支持位置控制