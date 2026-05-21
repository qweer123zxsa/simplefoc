# STM32 SimpleFOC 闭环控制详细教程

> 基于 STM32F103 移植 SimpleFOC 实现 BLDC 电机闭环控制
> 作者：loop222 @郑州
> 原文链接：https://blog.csdn.net/loop222/article/details/120471390

---

## 目录

1. [项目概述](#1-项目概述)
2. [项目结构](#2-项目结构)
3. [核心数据结构](#3-核心数据结构)
4. [工具函数模块 (foc_utils)](#4-工具函数模块-foc_utils)
5. [磁编码器模块 (MagneticSensor)](#5-磁编码器模块-magneticsensor)
6. [低通滤波器 (lowpass_filter)](#6-低通滤波器-lowpass_filter)
7. [FOC 电机控制模块 (FOCMotor)](#7-foc-电机控制模块-focmotor)
8. [BLDC 电机驱动模块 (BLDCMotor)](#8-bldc-电机驱动模块-bldcmotor)
9. [PID 控制器模块](#9-pid-控制器模块)
10. [主函数详解](#10-主函数详解)
11. [控制模式详解](#11-控制模式详解)
12. [参数调试指南](#12-参数调试指南)

---

## 1. 项目概述

### 1.1 什么是 SimpleFOC？

SimpleFOC 是一个开源的磁场定向控制（Field Oriented Control）库，用于控制 BLDC（无刷直流）电机和步进电机。本项目将 SimpleFOC 移植到 STM32F103 平台，实现闭环控制。

### 1.2 FOC 控制原理简介

FOC 的核心思想是将三相电流转换到旋转的 d-q 坐标系：

```
三相电流 (Ia, Ib, Ic) → Clark变换 → (Iα, Iβ) → Park变换 → (Id, Iq)
```

- **d 轴（直轴）**：与转子磁场同向，通常控制为 0
- **q 轴（交轴）**：产生力矩的分量，控制电机转矩

### 1.3 支持的控制模式

| 模式 | 枚举值 | 说明 |
|------|--------|------|
| 力矩控制 | `Type_torque` | 直接控制电机力矩 |
| 速度控制 | `Type_velocity` | 闭环速度控制 |
| 位置控制 | `Type_angle` | 闭环位置/角度控制 |
| 开环速度 | `Type_velocity_openloop` | 无传感器速度控制 |
| 开环位置 | `Type_angle_openloop` | 无传感器位置控制 |

---

## 2. 项目结构

```
4_closeloop_M1/
├── user/
│   ├── main.c              # 主函数
│   ├── MyProject.h         # 项目总头文件
│   └── stm32f10x_it.c      # 中断服务函数
├── SimpleFOC/
│   ├── BLDCMotor.h/c       # BLDC电机驱动核心
│   ├── FOCMotor.h/c        # FOC算法相关
│   ├── foc_utils.h/c       # 数学工具函数
│   ├── MagneticSensor.h/c  # 磁编码器驱动
│   ├── lowpass_filter.h/c  # 低通滤波器
│   └── pid.h/c             # PID控制器
├── Hardware/
│   ├── timer.h/c           # PWM和定时器配置
│   ├── usart.h/c           # 串口通信
│   ├── iic.h/c             # I2C驱动(AS5600)
│   ├── spi2.h/c            # SPI驱动(TLE5012B)
│   └── delay.h/c           # 延时函数
└── lib/                    # STM32标准库
```

---

## 3. 核心数据结构

### 3.1 方向枚举 (Direction)

```c
// 文件: BLDCMotor.h
typedef enum
{
    CW      = 1,   // 顺时针 (Clockwise)
    CCW     = -1,  // 逆时针 (Counter Clockwise)
    UNKNOWN = 0    // 未知或无效状态
} Direction;
```

**用途**：在传感器校准时自动检测电机旋转方向。

### 3.2 运动控制类型 (MotionControlType)

```c
// 文件: FOCMotor.h
typedef enum
{
    Type_torque,            // 力矩控制模式
    Type_velocity,          // 速度闭环控制
    Type_angle,             // 位置/角度闭环控制
    Type_velocity_openloop, // 开环速度控制
    Type_angle_openloop     // 开环位置控制
} MotionControlType;
```

### 3.3 力矩控制类型 (TorqueControlType)

```c
// 文件: FOCMotor.h
typedef enum
{
    Type_voltage,    // 电压模式力矩控制（本项目使用）
    Type_dc_current, // DC电流模式（需要电流采样）
    Type_foc_current // FOC电流模式（需要电流采样）
} TorqueControlType;
```

### 3.4 DQ 电压/电流结构体

```c
// 文件: foc_utils.h

// d-q轴电压结构体
typedef struct 
{
    float d;  // d轴电压（通常为0）
    float q;  // q轴电压（产生力矩）
} DQVoltage_s;

// d-q轴电流结构体
typedef struct 
{
    float d;
    float q;
} DQCurrent_s;

// 三相电流结构体
typedef struct 
{
    float a;
    float b;
    float c;
} PhaseCurrent_s;
```

### 3.5 关键全局变量一览

| 变量名 | 类型 | 文件 | 说明 |
|--------|------|------|------|
| `voltage_power_supply` | float | BLDCMotor.c | 电源电压 (V) |
| `voltage_limit` | float | BLDCMotor.c | 电压限制 (V) |
| `voltage_sensor_align` | float | BLDCMotor.c | 传感器校准电压 (V) |
| `pole_pairs` | int | BLDCMotor.c | 电机极对数 |
| `velocity_limit` | float | BLDCMotor.c | 速度限制 (rad/s) |
| `sensor_direction` | long | BLDCMotor.c | 传感器方向 (CW/CCW) |
| `shaft_angle` | float | FOCMotor.c | 当前轴角度 (rad) |
| `shaft_velocity` | float | FOCMotor.c | 当前轴速度 (rad/s) |
| `electrical_angle` | float | FOCMotor.c | 电角度 (rad) |
| `zero_electric_angle` | float | FOCMotor.c | 零点电角度偏移 |
| `voltage` | DQVoltage_s | FOCMotor.c | d-q轴电压设定值 |

---

## 4. 工具函数模块 (foc_utils)

### 4.1 数学常量定义

```c
// 文件: foc_utils.h
#define _2_SQRT3  1.15470053838   // 2/√3
#define _SQRT3    1.73205080757   // √3
#define _1_SQRT3  0.57735026919   // 1/√3
#define _SQRT3_2  0.86602540378   // √3/2
#define _SQRT2    1.41421356237   // √2
#define _120_D2R  2.09439510239   // 120° 转弧度
#define _PI       3.14159265359   // π
#define _PI_2     1.57079632679   // π/2
#define _PI_3     1.0471975512    // π/3 (60°)
#define _2PI      6.28318530718   // 2π
#define _3PI_2    4.71238898038   // 3π/2
#define _PI_6     0.52359877559   // π/6 (30°)
```

### 4.2 工具宏定义

```c
// 符号函数：返回 -1, 0, 1
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

// 四舍五入
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

// 限幅函数：将 amt 限制在 [low, high] 范围内
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 快速平方根（使用近似算法）
#define _sqrt(a) (_sqrtApprox(a))
```

### 4.3 快速正弦函数 _sin()

为了提高 STM32 的运算效率，使用查表法实现正弦函数：

```c
// 文件: foc_utils.c
// 正弦查找表：200个点，覆盖 0~π/2，值放大10000倍存储为整数
const int sine_array[200] = {0,79,158,237,...,10000};

float _sin(float a)
{
    // 输入角度必须在 [0, 2π] 范围内
    if(a < _PI_2){
        // 第一象限：直接查表
        return 0.0001 * sine_array[_round(126.6873 * a)];
    }
    else if(a < _PI){
        // 第二象限：sin(π-a) = sin(a)
        return 0.0001 * sine_array[398 - _round(126.6873*a)];
    }
    else if(a < _3PI_2){
        // 第三象限：sin(π+a) = -sin(a)
        return -0.0001 * sine_array[-398 + _round(126.6873*a)];
    }
    else {
        // 第四象限：sin(2π-a) = -sin(a)
        return -0.0001 * sine_array[796 - _round(126.6873*a)];
    }
}
```

**性能**：约 50μs，精度 ±0.005

### 4.4 快速余弦函数 _cos()

```c
float _cos(float a)
{
    // cos(a) = sin(a + π/2)
    float a_sin = a + _PI_2;
    a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
    return _sin(a_sin);
}
```

### 4.5 角度归一化 _normalizeAngle()

```c
// 将角度归一化到 [0, 2π] 范围
float _normalizeAngle(float angle)
{
    float a = fmod(angle, _2PI);  // 取模运算
    return a >= 0 ? a : (a + _2PI);
}
```

### 4.6 快速平方根 _sqrtApprox()

使用著名的"快速逆平方根"算法（来自 Quake III）：

```c
float _sqrtApprox(float number)
{
    long i;
    float y;
    
    y = number;
    i = *(long *)&y;              // 将浮点数按位解释为整数
    i = 0x5f375a86 - (i >> 1);    // 魔数运算
    y = *(float *)&i;             // 转回浮点数
    return number * y;            // number * (1/√number) = √number
}
```

---

## 5. 磁编码器模块 (MagneticSensor)

本项目支持两种磁编码器：AS5600（I2C接口）和 TLE5012B（SPI接口）。

### 5.1 编码器选择配置

```c
// 文件: MyProject.h
#define M1_AS5600    1    // 使用 AS5600 设为 1
#define M1_TLE5012B  0    // 使用 TLE5012B 设为 1
// 注意：二者只能选一个
```

### 5.2 编码器参数

| 编码器 | 接口 | 分辨率 (CPR) | 说明 |
|--------|------|--------------|------|
| AS5600 | I2C | 4096 | 12位，常用于云台电机 |
| TLE5012B | SPI | 32768 | 15位，精度更高 |

### 5.3 关键变量

```c
// 文件: MagneticSensor.c
long  cpr;                      // 每圈脉冲数 (Counts Per Revolution)
float full_rotation_offset;     // 完整圈数偏移（用于扩展角度范围）
long  angle_data_prev;          // 上一次的原始角度数据
unsigned long velocity_calc_timestamp;  // 速度计算时间戳
float angle_prev;               // 上一次的角度值
```

### 5.4 初始化函数 MagneticSensor_Init()

```c
void MagneticSensor_Init(void)
{
#if M1_AS5600
    cpr = AS5600_CPR;  // 4096
    angle_data_prev = I2C_getRawCount(I2C1);  // 读取初始角度
#elif M1_TLE5012B
    cpr = TLE5012B_CPR;  // 32768
    angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE) & 0x7FFF;
#endif
    
    full_rotation_offset = 0;
    velocity_calc_timestamp = 0;
}
```

### 5.5 获取角度 getAngle()

```c
float getAngle(void)
{
    float angle_data, d_angle;
    
#if M1_AS5600
    angle_data = I2C_getRawCount(I2C1);
#elif M1_TLE5012B
    angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE) & 0x7FFF;
#endif
    
    // 跟踪圈数，将角度范围从 [0, 2π] 扩展到无限
    d_angle = angle_data - angle_data_prev;
    
    // 检测溢出（跨越0点）
    if(fabs(d_angle) > (0.8 * cpr))
        full_rotation_offset += d_angle > 0 ? -_2PI : _2PI;
    
    // 保存当前值供下次使用
    angle_data_prev = angle_data;
    
    // 返回完整角度 = 圈数偏移 + 当前传感器角度
    return (full_rotation_offset + (angle_data / (float)cpr) * _2PI);
}
```

**原理说明**：
- 编码器原始值范围是 [0, CPR-1]
- 当电机转过一圈时，原始值会从最大跳到0（或反向）
- 通过检测这种"跳变"来累计圈数
- 最终角度 = 累计圈数 × 2π + 当前角度

### 5.6 获取速度 getVelocity()

```c
float getVelocity(void)
{
    unsigned long now_us;
    float Ts, angle_c, vel;

    // 计算采样时间间隔
    now_us = SysTick->VAL;
    if(now_us < velocity_calc_timestamp)
        Ts = (float)(velocity_calc_timestamp - now_us) / 9 * 1e-6;
    else
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp) / 9 * 1e-6;
    
    // 防止异常情况
    if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

    // 获取当前角度
    angle_c = getAngle();
    
    // 速度 = 角度变化量 / 时间间隔
    vel = (angle_c - angle_prev) / Ts;

    // 保存变量供下次使用
    angle_prev = angle_c;
    velocity_calc_timestamp = now_us;
    
    return vel;
}
```

**时间计算说明**：
- 使用 SysTick 计数器（向下计数）
- 72MHz 主频，9分频后为 8MHz
- `Ts = 计数差 / 9 * 1e-6` 得到秒数

### 5.7 AS5600 I2C 读取函数

```c
#define AS5600_Address  0x36    // I2C 地址
#define RAW_Angle_Hi    0x0C    // 原始角度高字节寄存器

unsigned short I2C_getRawCount(I2C_TypeDef* I2Cx)
{
    // 1. 发送起始条件
    // 2. 发送设备地址（写模式）
    // 3. 发送寄存器地址 0x0C
    // 4. 重新发送起始条件
    // 5. 发送设备地址（读模式）
    // 6. 读取2字节数据（高字节 + 低字节）
    // 7. 发送停止条件
    return ((dh << 8) + dl);  // 返回12位角度值
}
```

---

## 6. 低通滤波器 (lowpass_filter)

### 6.1 作用

速度信号通常含有噪声，低通滤波器用于平滑速度信号。

### 6.2 实现

```c
// 文件: lowpass_filter.c
float y_vel_prev = 0;  // 上一次的滤波输出

float LPF_velocity(float x)
{
    // 一阶低通滤波：y = α * y_prev + (1-α) * x
    // α = 0.9 表示滤波系数，值越大滤波越强
    float y = 0.9 * y_vel_prev + 0.1 * x;
    
    y_vel_prev = y;
    return y;
}
```

### 6.3 滤波系数说明

| α 值 | 效果 |
|------|------|
| 0.9 | 强滤波，响应慢，噪声小 |
| 0.5 | 中等滤波 |
| 0.1 | 弱滤波，响应快，噪声大 |

---

## 7. FOC 电机控制模块 (FOCMotor)

### 7.1 核心变量

```c
// 文件: FOCMotor.c
float shaft_angle;          // 当前机械角度 (rad)
float electrical_angle;     // 当前电角度 (rad)
float shaft_velocity;       // 当前速度 (rad/s)
float current_sp;           // 电流设定值
float shaft_velocity_sp;    // 速度设定值
float shaft_angle_sp;       // 角度设定值
DQVoltage_s voltage;        // d-q轴电压
DQCurrent_s current;        // d-q轴电流

float sensor_offset = 0;    // 传感器偏移
float zero_electric_angle;  // 零点电角度
```

### 7.2 获取轴角度 shaftAngle()

```c
float shaftAngle(void)
{
    // 轴角度 = 传感器方向 * 传感器角度 - 偏移量
    return sensor_direction * getAngle() - sensor_offset;
}
```

### 7.3 获取轴速度 shaftVelocity()

```c
float shaftVelocity(void)
{
    // 轴速度 = 传感器方向 * 低通滤波后的速度
    return sensor_direction * LPF_velocity(getVelocity());
}
```

### 7.4 计算电角度 electricalAngle()

```c
float electricalAngle(void)
{
    // 电角度 = (机械角度 + 偏移) * 极对数 - 零点电角度
    return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}
```

**电角度与机械角度的关系**：
```
电角度 = 机械角度 * 极对数
```

例如：7极对电机转一圈（机械角度 2PI），电角度变化 7 * 2PI = 14PI

---

## 8. BLDC 电机驱动模块 (BLDCMotor)

这是整个 FOC 控制的核心模块。

### 8.1 全局变量

```c
// 文件: BLDCMotor.c
long sensor_direction;        // 传感器方向 (CW/CCW)
float voltage_power_supply;   // 电源电压 (V)
float voltage_limit;          // 电压限制 (V)
float voltage_sensor_align;   // 传感器校准电压 (V)
int pole_pairs;               // 电机极对数
unsigned long open_loop_timestamp;  // 开环时间戳
float velocity_limit;         // 速度限制 (rad/s)
```

### 8.2 电机初始化 Motor_init()

```c
void Motor_init(void)
{
    printf("MOT: Init\r\n");
    
    // 确保校准电压不超过限制
    if(voltage_sensor_align > voltage_limit) 
        voltage_sensor_align = voltage_limit;
    
    // 初始化传感器方向为未知
    sensor_direction = UNKNOWN;
    
    // 使能电机驱动
    M1_Enable;
    printf("MOT: Enable driver.\r\n");
}
```

### 8.3 FOC 初始化 Motor_initFOC()

```c
void Motor_initFOC(void)
{
    // 1. 传感器校准（检测方向、极对数、零点）
    alignSensor();
    
    // 2. 初始化角度和速度
    angle_prev = getAngle();
    delay_ms(5);
    shaft_velocity = shaftVelocity();  // 确保初始速度为0
    delay_ms(5);
    shaft_angle = shaftAngle();
    
    // 3. 角度模式下，以当前角度为目标
    if(controller == Type_angle)
        target = shaft_angle;
    
    delay_ms(200);
}
```

### 8.4 传感器校准 alignSensor() - 重要函数

这是 FOC 初始化最关键的步骤：

```c
int alignSensor(void)
{
    long i;
    float angle;
    float mid_angle, end_angle;
    float moved;
    
    printf("MOT: Align sensor.\r\n");
    
    // ========== 第一步：检测旋转方向 ==========
    // 正向转动一个电周期
    for(i = 0; i <= 500; i++)
    {
        angle = _3PI_2 + _2PI * i / 500.0;
        setPhaseVoltage(voltage_sensor_align, 0, angle);
        delay_ms(2);
    }
    mid_angle = getAngle();  // 记录中间位置
    
    // 反向转回
    for(i = 500; i >= 0; i--) 
    {
        angle = _3PI_2 + _2PI * i / 500.0;
        setPhaseVoltage(voltage_sensor_align, 0, angle);
        delay_ms(2);
    }
    end_angle = getAngle();  // 记录结束位置
    setPhaseVoltage(0, 0, 0);
    delay_ms(200);
    
    // 判断方向
    moved = fabs(mid_angle - end_angle);
    if((mid_angle == end_angle) || (moved < 0.02))
    {
        printf("MOT: Failed to notice movement.\r\n");
        M1_Disable;  // 检测失败，关闭驱动
        return 0;
    }
    else if(mid_angle < end_angle)
    {
        sensor_direction = CCW;
    }
    else
    {
        sensor_direction = CW;
    }
    
    // ========== 第二步：验证极对数 ==========
    if(fabs(moved * pole_pairs - _2PI) > 0.5)
    {
        // 极对数不正确，自动计算
        pole_pairs = _2PI / moved + 0.5;  // 四舍五入
        printf("Estimated pp: %d\r\n", pole_pairs);
    }
    
    // ========== 第三步：检测零点电角度 ==========
    setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);  // 固定在 270 度位置
    delay_ms(700);
    zero_electric_angle = _normalizeAngle(
        _electricalAngle(sensor_direction * getAngle(), pole_pairs)
    );
    printf("Zero elec. angle: %.4f\r\n", zero_electric_angle);
    
    setPhaseVoltage(0, 0, 0);
    delay_ms(200);
    
    return 1;
}
```

**校准过程图解**：

```
1. 施加电压，电机转动一个电周期
   起点 -------> mid_angle
   
2. 反向转回
   mid_angle -------> end_angle
   
3. 比较 mid_angle 和 end_angle 确定方向
   - mid_angle < end_angle -> CCW
   - mid_angle > end_angle -> CW
   
4. 计算移动量验证极对数
   moved * pole_pairs 约等于 2PI
   
5. 固定在已知电角度位置，读取传感器值作为零点
```

### 8.5 FOC 主循环 loopFOC()

```c
void loopFOC(void)
{
    // 开环模式不需要执行
    if(controller == Type_angle_openloop || controller == Type_velocity_openloop) 
        return;
    
    // 1. 更新轴角度
    shaft_angle = shaftAngle();
    
    // 2. 计算电角度
    electrical_angle = electricalAngle();
    
    // 3. 根据力矩控制类型处理（当前只支持电压模式）
    switch(torque_controller)
    {
        case Type_voltage:
            break;  // 电压模式无需额外处理
        case Type_dc_current:
        case Type_foc_current:
            break;  // 电流模式需要电流采样（未实现）
    }
    
    // 4. 设置相电压 - FOC 核心！
    setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
```


### 8.6 运动控制 move()

```c
void move(float new_target)
{
    // 更新速度
    shaft_velocity = shaftVelocity();
    
    switch(controller)
    {
        case Type_torque:
            // 力矩模式：直接设置 q 轴电压
            if(torque_controller == Type_voltage)
                voltage.q = new_target;
            else
                current_sp = new_target;
            break;
            
        case Type_angle:
            // 位置模式：角度PID -> 速度PID -> 电压
            shaft_angle_sp = new_target;
            shaft_velocity_sp = PID_angle(shaft_angle_sp - shaft_angle);
            current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);
            if(torque_controller == Type_voltage)
            {
                voltage.q = current_sp;
                voltage.d = 0;
            }
            break;
            
        case Type_velocity:
            // 速度模式：速度PID -> 电压
            shaft_velocity_sp = new_target;
            current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity);
            if(torque_controller == Type_voltage)
            {
                voltage.q = current_sp;
                voltage.d = 0;
            }
            break;
            
        case Type_velocity_openloop:
            // 开环速度
            shaft_velocity_sp = new_target;
            voltage.q = velocityOpenloop(shaft_velocity_sp);
            voltage.d = 0;
            break;
            
        case Type_angle_openloop:
            // 开环位置
            shaft_angle_sp = new_target;
            voltage.q = angleOpenloop(shaft_angle_sp);
            voltage.d = 0;
            break;
    }
}
```

### 8.7 设置相电压 setPhaseVoltage() - FOC核心

这是 FOC 算法的核心函数，实现 SVPWM（空间矢量脉宽调制）：

```c
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout;
    uint32_t sector;
    float T0, T1, T2;
    float Ta, Tb, Tc;
    
    // ========== 第一步：计算输出电压幅值和角度 ==========
    if(Ud)  // 如果 Ud 不为0
    {
        Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else  // 只有 Uq（常见情况）
    {
        Uout = Uq / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + _PI_2);  // 加90度
    }
    
    // 限幅：SVPWM 最大调制比为 1/sqrt(3) 约等于 0.577
    if(Uout > 0.577) Uout = 0.577;
    if(Uout < -0.577) Uout = -0.577;
    
    // ========== 第二步：确定扇区 ==========
    // 将 360度 分为 6 个扇区，每个 60度
    sector = (angle_el / _PI_3) + 1;
    
    // ========== 第三步：计算矢量作用时间 ==========
    T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * _sin(angle_el - (sector - 1.0) * _PI_3) * Uout;
    T0 = 1 - T1 - T2;  // 零矢量时间
    
    // ========== 第四步：计算三相占空比 ==========
    switch(sector)
    {
        case 1:
            Ta = T1 + T2 + T0/2;
            Tb = T2 + T0/2;
            Tc = T0/2;
            break;
        case 2:
            Ta = T1 + T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T0/2;
            break;
        case 3:
            Ta = T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T2 + T0/2;
            break;
        case 4:
            Ta = T0/2;
            Tb = T1 + T0/2;
            Tc = T1 + T2 + T0/2;
            break;
        case 5:
            Ta = T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T2 + T0/2;
            break;
        case 6:
            Ta = T1 + T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T0/2;
            break;
        default:
            Ta = Tb = Tc = 0;
    }
    
    // ========== 第五步：设置 PWM 占空比 ==========
    TIM_SetCompare1(TIM2, Ta * PWM_Period);
    TIM_SetCompare2(TIM2, Tb * PWM_Period);
    TIM_SetCompare3(TIM2, Tc * PWM_Period);
}
```

**SVPWM 原理图解**：

```
        扇区2    扇区1
           \    /
            \  /
    扇区3 ---*--- 扇区6
            /  \
           /    \
        扇区4    扇区5

每个扇区 60度，通过组合相邻两个基本矢量
和零矢量来合成任意方向的电压矢量
```

### 8.8 开环速度控制 velocityOpenloop()

```c
float velocityOpenloop(float target_velocity)
{
    unsigned long now_us;
    float Ts, Uq;
    
    // 计算时间间隔
    now_us = SysTick->VAL;
    // ... 时间计算 ...
    
    // 根据目标速度计算角度增量
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    
    Uq = voltage_limit;
    // 设置相电压
    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));
    
    return Uq;
}
```

### 8.9 开环位置控制 angleOpenloop()

```c
float angleOpenloop(float target_angle)
{
    // ... 时间计算 ...
    
    // 以最大速度向目标角度移动
    if(fabs(target_angle - shaft_angle) > velocity_limit * Ts)
    {
        shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
    }
    else
    {
        shaft_angle = target_angle;
    }
    
    Uq = voltage_limit;
    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, pole_pairs));
    
    return Uq;
}
```

---

## 9. PID 控制器模块

### 9.1 PID 参数变量

```c
// 文件: pid.c
float pid_vel_P, pid_ang_P;       // 比例系数
float pid_vel_I, pid_ang_D;       // 积分/微分系数
float integral_vel_prev;          // 上一次积分值
float error_vel_prev, error_ang_prev;  // 上一次误差
float output_vel_ramp;            // 输出斜坡限制
float output_vel_prev;            // 上一次输出
unsigned long pid_vel_timestamp, pid_ang_timestamp;  // 时间戳
```

### 9.2 PID 初始化 PID_init()

```c
void PID_init(void)
{
    // 速度环 PI 参数
    pid_vel_P = 0.1;           // 比例系数
    pid_vel_I = 2;             // 积分系数
    output_vel_ramp = 100;     // 输出变化率限制 [volts/second]
    integral_vel_prev = 0;
    error_vel_prev = 0;
    output_vel_prev = 0;
    pid_vel_timestamp = SysTick->VAL;
    
    // 角度环 PD 参数
    pid_ang_P = 10;            // 比例系数
    pid_ang_D = 0.5;           // 微分系数
    error_ang_prev = 0;
    pid_ang_timestamp = SysTick->VAL;
}
```

### 9.3 速度 PID 控制器 PID_velocity()

速度环使用 **PI 控制**（比例 + 积分）：

```c
float PID_velocity(float error)
{
    unsigned long now_us;
    float Ts;
    float proportional, integral, output;
    float output_rate;
    
    // 计算采样时间
    now_us = SysTick->VAL;
    // ... 时间计算 ...
    
    // ========== 比例项 ==========
    // u_p = P * e(k)
    proportional = pid_vel_P * error;
    
    // ========== 积分项（Tustin 变换）==========
    // u_ik = u_ik_1 + I * Ts/2 * (ek + ek_1)
    integral = integral_vel_prev + pid_vel_I * Ts * 0.5 * (error + error_vel_prev);
    
    // 积分抗饱和
    integral = _constrain(integral, -voltage_limit, voltage_limit);
    
    // ========== 输出 ==========
    output = proportional + integral;
    output = _constrain(output, -voltage_limit, voltage_limit);
    
    // ========== 输出斜坡限制 ==========
    // 防止输出变化过快
    output_rate = (output - output_vel_prev) / Ts;
    if(output_rate > output_vel_ramp)
        output = output_vel_prev + output_vel_ramp * Ts;
    else if(output_rate < -output_vel_ramp)
        output = output_vel_prev - output_vel_ramp * Ts;
    
    // 保存变量
    integral_vel_prev = integral;
    output_vel_prev = output;
    error_vel_prev = error;
    
    return output;
}
```

**PI 控制器框图**：

```
                    +-------+
error ----+------->|   P   |----+
          |        +-------+    |
          |                     |    +-------+
          |        +-------+    +--->|  SUM  |---> output
          +------->|   I   |----+    +-------+
                   +-------+
```

### 9.4 角度 PID 控制器 PID_angle()

角度环使用 **PD 控制**（比例 + 微分）：
原因如下：

角度环用PD（比例+微分），速度环用PI（比例+积分），是因为它们控制目标和响应速度不同：

角度环（外环）主要负责“定位”，对误差的长期积累不敏感，只需要快速响应和抑制超调，所以用PD，微分能抑制超调，比例能快速响应。
速度环（内环）负责“稳速”，对稳态误差很敏感，需要消除长期误差，所以用PI，积分能消除稳态误差，比例能快速响应。
如果角度环也用积分（PI），容易导致系统慢、超调大，甚至不稳定。
而速度环不用积分，电机速度长期偏差无法消除。

总结：

角度环用PD，响应快、超调小，定位准。
速度环用PI，稳态误差小，速度稳。
这是工业伺服控制的经典串级PID设计
```c
float PID_angle(float error)
{
    unsigned long now_us;
    float Ts;
    float proportional, derivative, output;
    
    // 计算采样时间
    // ... 时间计算 ...
    
    // ========== 比例项 ==========
    proportional = pid_ang_P * error;
    
    // ========== 微分项 ==========
    // u_dk = D * (ek - ek_1) / Ts
    derivative = pid_ang_D * (error - error_ang_prev) / Ts;
    
    // ========== 输出 ==========
    output = proportional + derivative;
    output = _constrain(output, -velocity_limit, velocity_limit);
    
    // 保存变量
    error_ang_prev = error;
    
    return output;
}
```

### 9.5 串级 PID 控制结构

位置模式采用串级 PID 控制：

```
                角度环(PD)              速度环(PI)
目标角度 ---> [角度PID] ---> 目标速度 ---> [速度PID] ---> 电压
    ^                           ^
    |                           |
    +--- 实际角度                +--- 实际速度
```
这是一个很好的问题！你观察得很仔细。让我解释一下为什么角度环的输出可以作为速度环的输入。



串级PID中的单位转换


关键点：角度PID的输出不是角度，而是速度！



让我们看代码中的实现：



C
// 文件: BLDCMotor.c - move() 函数中的位置模式
case Type_angle:
    // 位置模式：角度PID -> 速度PID -> 电压
    shaft_angle_sp = new_target;                                    // 目标角度 (rad)
    shaft_velocity_sp = PID_angle(shaft_angle_sp - shaft_angle);   // 输出是速度 (rad/s)
    current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // 输出是电压 (V)
    voltage.q = current_sp;
    break;


单位流转过程


PLAINTEXT
输入: 角度误差 (rad)
  ↓
[角度PID控制器]
  ↓
输出: 目标速度 (rad/s)  ← 这里完成了单位转换！
  ↓
[速度PID控制器]
  ↓
输出: 电压 (V)


为什么角度PID输出是速度？



看角度PID控制器的实现：



C
float PID_angle(float error)  // error 单位: rad
{
    float proportional, derivative, output;
    
    // 比例项: P * error
    proportional = pid_ang_P * error;  // 单位: P系数 * rad
    
    // 微分项: D * (error - error_prev) / Ts
    derivative = pid_ang_D * (error - error_ang_prev) / Ts;  // 单位: D系数 * rad/s
    
    output = proportional + derivative;
    output = _constrain(output, -velocity_limit, velocity_limit);  // 限制在速度范围内！
    
    return output;  // 单位: rad/s
}


关键理解


比例项 (P)：
pid_ang_P = 10 的单位实际上是 (rad/s) / rad = 1/s
proportional = 10 * 0.1rad = 1 rad/s
意思是：角度误差每增加1弧度，就增加10 rad/s的速度去追


微分项 (D)：
pid_ang_D = 0.5 的单位是 (rad/s) / (rad/s) = 无量纲
derivative = 0.5 * (角度变化率)
用于抑制角度变化过快（阻尼作用）


输出限制：
C
   output = _constrain(output, -velocity_limit, velocity_limit);
   

这里限制在 velocity_limit = 20 rad/s，明确说明输出是速度！



物理意义



这种设计符合直觉：



PLAINTEXT
角度误差大 → 需要更快的速度去追 → 角度PID输出大速度
角度误差小 → 只需慢速微调 → 角度PID输出小速度
到达目标 → 速度为0 → 角度PID输出0


完整的控制链路


PLAINTEXT
目标角度: 6.28 rad
当前角度: 0 rad
角度误差: 6.28 rad
    ↓
角度PID: P=10, D=0.5
输出速度: 10 * 6.28 = 62.8 rad/s (会被限制到20 rad/s)
    ↓
速度PID: 目标20 rad/s, 当前0 rad/s
速度误差: 20 rad/s
    ↓
速度PID: P=0.1, I=2
输出电压: 逐渐增加到合适值
    ↓
电机加速，角度逐渐接近目标


总结



角度环的输出单位是速度 (rad/s)，不是角度 (rad)，这是通过PID参数的单位设计实现的。这样设计的好处是：



外环（角度）控制"去哪里"
内环（速度）控制"多快去"
两个环路各司其职，提高控制性能



这就是为什么代码中有 velocity_limit 来限制角度PID的输出范围！
**为什么这样设计？**
- 外环（角度）响应慢，用 PD 控制
- 内环（速度）响应快，用 PI 控制
- 串级结构提高系统稳定性和响应速度

### 9.6 PID 参数调试建议

| 参数 | 作用 | 调试方法 |
|------|------|----------|
| `pid_vel_P` | 速度响应速度 | 从小到大，直到出现振荡再减小 |
| `pid_vel_I` | 消除稳态误差 | 从小到大，过大会振荡 |
| `pid_ang_P` | 位置响应速度 | 从小到大调整 |
| `pid_ang_D` | 抑制超调 | 配合 P 调整，过大会振荡 |

---

## 10. 主函数详解

### 10.1 硬件初始化

```c
// 文件: main.c
void GPIO_Config(void)
{
    // 使能 GPIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // PC13 - LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // PB9 - 电机使能引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);  // 初始禁用
}
```

### 10.2 主函数流程

```c
int main(void)
{
    unsigned int count_i = 0;
    
    // ========== 1. 硬件初始化 ==========
    GPIO_Config();
    uart_init(115200);
    
#if M1_AS5600
    I2C_Init_();               // AS5600 使用 I2C
    printf("AS5600\r\n");
#elif M1_TLE5012B
    SPI2_Init_();              // TLE5012B 使用 SPI
    printf("TLE5012B\r\n");
#endif
    
    TIM2_PWM_Init();           // PWM 初始化 (25kHz)
    TIM4_1ms_Init();           // 1ms 定时中断
    
    delay_ms(1000);            // 等待系统稳定
    MagneticSensor_Init();     // 磁编码器初始化
    
    // ========== 2. FOC 参数配置 ==========
    voltage_power_supply = 12;     // 电源电压 12V
    pole_pairs = 7;                // 电机极对数
    voltage_limit = 6;             // 电压限制 (需小于 12/1.732=6.9)
    velocity_limit = 20;           // 速度限制 rad/s
    voltage_sensor_align = 2.5;    // 校准电压
    torque_controller = Type_voltage;  // 电压力矩模式
    controller = Type_velocity;    // 速度控制模式
    target = 0;                    // 初始目标值
    
    // ========== 3. FOC 初始化 ==========
    Motor_init();
    Motor_initFOC();
    PID_init();
    printf("Motor ready.\r\n");
    
    systick_CountMode();   // 切换 SysTick 为计数模式
    
    // ========== 4. 主循环 ==========
    while(1)
    {
        count_i++;
        
        // LED 闪烁 (0.2s)
        if(time1_cntr >= 200)
        {
            time1_cntr = 0;
            LED_blink;
        }
        
        // 调试计数 (1s)
        if(time2_cntr >= 1000)
        {
            time2_cntr = 0;
            count_i = 0;
        }
        
        // ========== FOC 核心调用 ==========
        move(target);      // 运动控制
        loopFOC();         // FOC 主循环
        commander_run();   // 串口命令处理
    }
}
```

### 10.3 串口命令处理

```c
void commander_run(void)
{
    if((USART_RX_STA & 0x8000) != 0)  // 收到数据
    {
        switch(USART_RX_BUF[0])
        {
            case 'H':   // 测试命令
                printf("Hello World!\r\n");
                break;
                
            case 'T':   // 设置目标值，如 "T6.28"
                target = atof((const char *)(USART_RX_BUF + 1));
                printf("RX=%.4f\r\n", target);
                break;
                
            case 'D':   // 禁用电机
                M1_Disable;
                printf("OK!\r\n");
                break;
                
            case 'E':   // 使能电机
                M1_Enable;
                printf("OK!\r\n");
                break;
        }
        USART_RX_STA = 0;
    }
}
```

### 10.4 PWM 定时器配置

```c
// 文件: timer.c
#define PWM_Period  1440  // PWM 周期

void TIM2_PWM_Init(void)
{
    // TIM2 配置为中心对齐 PWM 模式
    // 72MHz / 1440 / 2 = 25kHz
    
    // PA0, PA1, PA2 为 PWM 输出引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_Period = 1440 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    
    // 配置 3 路 PWM 输出
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
}
```

---

## 11. 控制模式详解

### 11.1 力矩控制模式 (Type_torque)

**原理**：直接设置 q 轴电压，电压与力矩近似成正比。

```c
controller = Type_torque;
target = 2.0;  // 设置 2V 的 q 轴电压
```

**控制框图**：
```
target(电压) ---> [setPhaseVoltage] ---> 电机
```

**适用场景**：
- 需要直接控制力矩的应用
- 作为其他控制模式的内环

### 11.2 速度控制模式 (Type_velocity)

**原理**：通过速度 PID 控制器调节电压，使电机达到目标速度。

```c
controller = Type_velocity;
target = 10.0;  // 目标速度 10 rad/s
```

**控制框图**：
```
target(速度) ---> [速度PID] ---> 电压 ---> [setPhaseVoltage] ---> 电机
    ^                                                              |
    |                                                              |
    +----------------------- 速度反馈 <----------------------------+
```

**适用场景**：
- 风扇、水泵等恒速应用
- 传送带驱动

### 11.3 位置控制模式 (Type_angle)

**原理**：串级 PID 控制，外环角度 PD，内环速度 PI。

```c
controller = Type_angle;
target = 6.28;  // 目标位置 2PI rad (一圈)
```

**控制框图**：
```
target(角度) ---> [角度PD] ---> 目标速度 ---> [速度PI] ---> 电压 ---> 电机
    ^                              ^                                   |
    |                              |                                   |
    +--- 角度反馈 <----------------+--- 速度反馈 <---------------------+
```

**适用场景**：
- 云台控制
- 机械臂关节
- 精确定位应用

### 11.4 开环速度模式 (Type_velocity_openloop)

**原理**：不使用传感器反馈，直接按目标速度递增电角度。

```c
controller = Type_velocity_openloop;
target = 5.0;  // 目标速度 5 rad/s
```

**控制框图**：
```
target(速度) ---> [角度累加] ---> 电角度 ---> [setPhaseVoltage] ---> 电机
```

**适用场景**：
- 传感器故障时的备用模式
- 简单应用

### 11.5 开环位置模式 (Type_angle_openloop)

**原理**：以最大速度向目标角度移动。

```c
controller = Type_angle_openloop;
target = 3.14;  // 目标位置 PI rad
```

**适用场景**：
- 简单定位
- 测试用途

### 11.6 模式选择建议

| 应用场景 | 推荐模式 | 原因 |
|----------|----------|------|
| 云台稳定 | Type_angle | 需要精确位置控制 |
| 风扇驱动 | Type_velocity | 恒速运行 |
| 力反馈 | Type_torque | 直接力矩控制 |
| 无传感器 | Type_velocity_openloop | 不需要编码器 |

---

## 12. 参数调试指南

### 12.1 电机参数设置

```c
// 必须正确设置的参数
voltage_power_supply = 12;   // 实际电源电压
pole_pairs = 7;              // 电机极对数（查看电机规格或自动检测）
voltage_limit = 6;           // 最大电压限制
```

**voltage_limit 计算**：
```
voltage_limit < voltage_power_supply / 1.732
例如：12V 电源，voltage_limit < 6.93V
```

### 12.2 校准电压设置

```c
voltage_sensor_align = 2.5;  // 传感器校准电压
```

| 电机类型 | 建议值 |
|----------|--------|
| 航模电机（大功率） | 0.5 - 1.0 V |
| 云台电机（小功率） | 2.0 - 3.0 V |

**调试方法**：
- 太小：校准时电机不动或抖动
- 太大：校准时电机转动过快，可能损坏

### 12.3 PID 参数调试

#### 速度环 PI 参数

```c
pid_vel_P = 0.1;   // 比例系数
pid_vel_I = 2;     // 积分系数
```

**调试步骤**：
1. 先将 I 设为 0
2. 逐渐增大 P，直到出现振荡
3. 将 P 减小到振荡值的 60-70%
4. 逐渐增大 I，消除稳态误差
5. 如果振荡，减小 I

#### 角度环 PD 参数

```c
pid_ang_P = 10;    // 比例系数
pid_ang_D = 0.5;   // 微分系数
```

**调试步骤**：
1. 先将 D 设为 0
2. 逐渐增大 P，直到响应足够快
3. 如果超调严重，增大 D
4. D 过大会导致高频振荡

### 12.4 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 电机不转 | 极对数错误 | 检查 pole_pairs 设置 |
| 电机抖动 | 校准电压太小 | 增大 voltage_sensor_align |
| 电机发热 | 电压限制太高 | 减小 voltage_limit |
| 速度振荡 | PID 参数不当 | 减小 pid_vel_P 或 pid_vel_I |
| 位置超调 | 角度 PD 参数不当 | 增大 pid_ang_D |
| 校准失败 | 编码器连接问题 | 检查 I2C/SPI 连接 |

### 12.5 串口调试命令

| 命令 | 功能 | 示例 |
|------|------|------|
| H | 测试通信 | 发送 "H" 返回 "Hello World!" |
| T | 设置目标值 | "T10.0" 设置目标为 10 |
| D | 禁用电机 | 发送 "D" |
| E | 使能电机 | 发送 "E" |

### 12.6 调试流程建议

```
1. 开环测试
   controller = Type_velocity_openloop;
   target = 1.0;
   确认电机能正常转动
   
2. 传感器测试
   读取 getAngle() 返回值
   手动转动电机，确认角度变化正确
   
3. 闭环速度测试
   controller = Type_velocity;
   target = 5.0;
   观察速度是否稳定
   
4. 闭环位置测试
   controller = Type_angle;
   target = 6.28;
   观察位置是否准确
```

---

## 附录：引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM_A | PA0 | TIM2_CH1 |
| PWM_B | PA1 | TIM2_CH2 |
| PWM_C | PA2 | TIM2_CH3 |
| Enable | PB9 | 电机使能 |
| LED | PC13 | 状态指示 |
| I2C_SCL | PB6 | AS5600 时钟 |
| I2C_SDA | PB7 | AS5600 数据 |
| SPI_SCK | PB13 | TLE5012B 时钟 |
| SPI_MISO | PB14 | TLE5012B 数据输入 |
| SPI_MOSI | PB15 | TLE5012B 数据输出 |
| SPI_CS | PB8 | TLE5012B 片选 |
| UART_TX | PA9 | 串口发送 |
| UART_RX | PA10 | 串口接收 |

---

## 参考资料

- SimpleFOC 官方文档：https://simplefoc.com/
- 原作者教程：https://blog.csdn.net/loop222/article/details/120471390
- STM32F103 参考手册

---

*教程完*
