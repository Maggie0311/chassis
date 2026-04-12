# 卡尔曼滤波器模块文档

> 文件路径：`applications/chassis_fun/Filter/`  
> 作者：2026 底盘项目组  
> 更新日期：2026/04/11

---

## 目录

1. [模块概述](#1-模块概述)
2. [文件结构](#2-文件结构)
3. [卡尔曼滤波基础原理](#3-卡尔曼滤波基础原理)
4. [滤波器一：一维速度卡尔曼（Kalman_Forward_v_one_dimension）](#4-滤波器一一维速度卡尔曼)
5. [滤波器二：二阶高度卡尔曼（Kalman_Height_t）](#5-滤波器二二阶高度卡尔曼)
6. [滤波器三：速度与角速度融合卡尔曼（Kaliman_v_w_t）](#6-滤波器三速度与角速度融合卡尔曼)
7. [滤波器四：机体运动卡尔曼（Robotmotion_Kalman_t）](#7-滤波器四机体运动卡尔曼)
8. [防打滑处理](#8-防打滑处理)
9. [参数调节指南](#9-参数调节指南)
10. [使用示例](#10-使用示例)

---

## 1. 模块概述

本模块实现了四种针对底盘运动控制场景的卡尔曼滤波器，用于对传感器数据（编码器速度、IMU 加速度/角速度、激光测距等）进行融合与去噪，提升状态估计精度。

| 滤波器 | 状态维度 | 主要用途 |
|--------|----------|----------|
| `Kalman_Forward_v_one_dimension` | 1D | 单轴速度滤波（编码器去噪） |
| `Kalman_Height_t` | 2D（高度 + 速度） | 高度与升降速度估计 |
| `Kaliman_v_w_t` | 2D（线速度 + 角速度） | 双轮线速度/角速度融合 |
| `Robotmotion_Kalman_t` | 2D（位移 + 速度） | 机体运动位移/速度估计（主力） |

---

## 2. 文件结构

```
Filter/
├── Kalman.h        # 结构体定义 + 函数声明
├── Kalman.c        # 所有滤波器的实现
└── SConscript      # RT-Thread 构建脚本
```

---

## 3. 卡尔曼滤波基础原理

卡尔曼滤波分为**预测**和**更新**两个阶段，循环迭代：

### 预测阶段

$$\hat{x}_{k}^{-} = F \hat{x}_{k-1} + B u_k$$

$$P_{k}^{-} = F P_{k-1} F^T + Q$$

- $\hat{x}$：状态估计向量（如位移、速度）
- $F$：状态转移矩阵（物理模型）
- $B$：控制输入矩阵，$u_k$ 为控制量（如加速度）
- $P$：估计误差协方差矩阵
- $Q$：过程噪声协方差（模型不确定性）

### 更新阶段

$$y_k = z_k - H \hat{x}_{k}^{-} \quad \text{（新息/残差）}$$

$$S = H P_{k}^{-} H^T + R \quad \text{（新息协方差）}$$

$$K_k = P_{k}^{-} H^T S^{-1} \quad \text{（卡尔曼增益）}$$

$$\hat{x}_k = \hat{x}_{k}^{-} + K_k y_k$$

$$P_k = (I - K_k H) P_{k}^{-}$$

- $z_k$：传感器观测值
- $H$：观测矩阵（将状态空间映射到观测空间）
- $R$：观测噪声协方差（传感器噪声）
- $K$：卡尔曼增益（决定更信任预测还是观测）

> **关键直觉**：
> - Q 大 → 更信任传感器观测
> - R 大 → 更信任模型预测

---

## 4. 滤波器一：一维速度卡尔曼

### 结构体

```c
typedef struct {
    float v_input;       // 当前传感器测量速度
    float a_input;       // 当前加速度输入
    float last_a_input;  // 上一时刻加速度
    float P_covariance;  // 估计误差协方差（标量）
    float Kalman_gain;   // 卡尔曼增益
    float v_prior;       // 先验速度估计
    float Q;             // 过程噪声
    float R;             // 观测噪声
    float v_observe;     // 最终后验速度估计（输出）
    float period;        // 控制周期 [s]
    char  Inited;        // 初始化标志
} Kalman_Forward_v_one_dimension;
```

### 算法流程（标量简化版）

```
先验估计：v_prior = v_observe_last + period × a_last
协方差预测：P = P + Q
卡尔曼增益：K = P / (P + R)
后验估计：v_observe = v_prior + K × (v_input - v_prior)
协方差更新：P = (1 - K) × P
```

### 接口

```c
float Kalman_calculate(Kalman_Forward_v_one_dimension *Kalman,
                       float v_now,   // 当前测量速度 [m/s 或 rad/s]
                       float a_now);  // 当前加速度
// 返回值：滤波后速度
```

### 初始化方法

无专用初始化函数，首次调用时自动初始化（`Inited == 0`）。使用前需手动设置：

```c
Kalman_Forward_v_one_dimension kf = {0};
kf.Q = 0.01f;     // 过程噪声，越大跟踪越快但噪声越多
kf.R = 0.1f;      // 观测噪声，越大越平滑
kf.period = 0.001f; // 控制周期 1ms
```

---

## 5. 滤波器二：二阶高度卡尔曼

### 应用场景

适用于哨兵等需要估计升降机构高度的场景，状态向量为：

$$x = \begin{bmatrix} \text{高度} \\ \text{升降速度} \end{bmatrix}$$

### 结构体关键字段

```c
typedef struct {
    float input[2];           // [高度测量值, 速度测量值]
    float a_input;            // 当前垂直加速度
    float State_prior[2];     // 先验状态：[预测高度, 预测速度]
    float State_observe[2];   // 后验状态：[估计高度, 估计速度]（输出）
    float P_covariance[2][2]; // 2×2 协方差矩阵
    float Kalman_gain[2][2];  // 2×2 卡尔曼增益矩阵
    float Q[2][2];            // 过程噪声矩阵
    float R[2][2];            // 观测噪声矩阵
    float period;             // 控制周期 [s]
    char  Inited;
} Kalman_Height_t;
```

### 状态转移模型

$$F = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix}, \quad Bu = \begin{bmatrix} \frac{1}{2}dt^2 \\ dt \end{bmatrix} a$$

### 接口

```c
// 初始化
void Kalman_height_init(Kalman_Height_t *Kalman,
                        float period,   // 控制周期
                        float Q1,       // 高度过程噪声
                        float Q2,       // 速度过程噪声
                        float R1,       // 高度观测噪声
                        float R2);      // 速度观测噪声

// 滤波计算
void Kalman_height_calculate(Kalman_Height_t *Kalman,
                             float height,    // 高度测量值
                             float height_v,  // 速度测量值
                             float height_a); // 加速度输入
```

### 输出读取

```c
float height   = Kalman.State_observe[0]; // 估计高度
float velocity = Kalman.State_observe[1]; // 估计速度
```

---

## 6. 滤波器三：速度与角速度融合卡尔曼

### 应用场景

用于差速驱动底盘，融合左右轮速度（编码器）和陀螺仪角速度，状态向量为：

$$x = \begin{bmatrix} v_{total} \\ \omega \end{bmatrix}$$

观测方程（通过车体运动学关联状态到左右轮速）：

$$z = \begin{bmatrix} v_{left} \\ v_{right} \end{bmatrix} = \begin{bmatrix} 1 & -r \\ 1 & r \end{bmatrix} \begin{bmatrix} v_{total} \\ \omega \end{bmatrix}$$

其中 $r$ 为车身半径（轮距一半）。

### 结构体关键字段

```c
typedef struct {
    // 状态变量 x = [v_total, w]
    float v_total_state;  // 估计线速度（输出）
    float w_state;        // 估计角速度（输出）

    // 观测变量 z = [v_left, v_right]
    float v_left_ob;
    float v_right_ob;

    // 观测矩阵 H = [1, -r; 1, r]
    float constant_1;    // = 1
    float r_positive;    // = +r
    float constant_2;    // = 1
    float r_negetive;    // = -r

    float Q1, Q2;        // 线速度/角速度过程噪声
    float R1, R2;        // 左轮/右轮观测噪声
    float P[2][2];       // 协方差矩阵
    float K[2][2];       // 卡尔曼增益
    float period;
    char  Inited;
} Kaliman_v_w_t;
```

### 接口

```c
// 初始化
void Kalman_v_w_init(Kaliman_v_w_t *Kalman,
                     float period, // 控制周期
                     float Q1,     // 线速度过程噪声
                     float Q2,     // 角速度过程噪声
                     float R,      // 观测噪声（左右轮相同）
                     float r);     // 车身半径

// 滤波计算
void Kalman_v_w_calculate(Kaliman_v_w_t *Kalman,
                          float v_left,   // 左轮速度 [m/s]
                          float v_right,  // 右轮速度 [m/s]
                          float a_input,  // 纵向加速度
                          float w_input); // 陀螺仪角速度 [rad/s]
```

---

## 7. 滤波器四：机体运动卡尔曼

**这是本模块最完善、注释最详细的滤波器，推荐优先使用。**

### 应用场景

估计底盘的**位移**和**速度**，加速度计作为控制输入，轮毂编码器速度作为观测量。适用于所有轮式底盘机型。

### 状态向量与矩阵设计

**状态向量**：

$$x = \begin{bmatrix} s \\ v \end{bmatrix}$$

**状态转移矩阵**（匀加速运动模型）：

$$F = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix}$$

**控制输入矩阵**（加速度 → 位移/速度）：

$$B = \begin{bmatrix} \frac{1}{2}dt^2 \\ dt \end{bmatrix}$$

**观测矩阵**（只能观测速度）：

$$H = \begin{bmatrix} 0 & 1 \end{bmatrix}$$

**过程噪声矩阵**（由加速度方差 $\sigma_a^2$ 推导）：

$$Q = \sigma_a^2 \begin{bmatrix} \frac{1}{4}dt^4 & \frac{1}{2}dt^3 \\ \frac{1}{2}dt^3 & dt^2 \end{bmatrix}$$

**观测噪声**：

$$R = \sigma_v^2$$

### 结构体

```c
typedef struct {
    float x[2];      // 状态向量：[位移(m), 速度(m/s)]
    float P[2][2];   // 协方差矩阵
    float F[2][2];   // 状态转移矩阵
    float B[2];      // 控制输入矩阵
    float H[2];      // 观测矩阵 = [0, 1]
    float Q[2][2];   // 过程噪声协方差
    float R;         // 观测噪声方差（可动态修改用于防打滑）
    float dt;        // 控制周期
    float acc_noise; // 记录初始加速度噪声标准差
    float vel_noise; // 记录初始速度噪声标准差
    int   initialized;
} Robotmotion_Kalman_t;
```

### 接口详解

#### 初始化

```c
void RobotMotion_Kalman_Init(Robotmotion_Kalman_t *kf,
                             float dt,                // 控制周期 [s]，如 0.001f
                             float accel_noise_std,   // 加速度标准差 [m/s²]，Q 由此推导
                             float velocity_noise_std);// 速度测量标准差 [m/s]，R 由此推导
```

#### 预测（仅用 IMU 加速度推进状态）

```c
void RobotMotionKalman_Predict(Robotmotion_Kalman_t *kf,
                               float acceleration); // IMU 测量加速度 [m/s²]
```

#### 更新（融合编码器速度）

```c
void RobotMotionKalman_Update(Robotmotion_Kalman_t *kf,
                              float measured_velocity); // 编码器测量速度 [m/s]
```

#### 一步调用（预测 + 更新）

```c
void RobotMotionKalman_Process(Robotmotion_Kalman_t *kf,
                               float measured_velocity, // 编码器速度
                               float acceleration);      // IMU 加速度
```

#### 读取结果

```c
float RobotMotionKalman_Getdisplacement(Robotmotion_Kalman_t *kf); // 返回位移 [m]
float RobotMotionKalman_Getvelocity(Robotmotion_Kalman_t *kf);     // 返回速度 [m/s]
```

### 预测阶段代码解析

```c
// 状态预测
float x0_prior = F[0][0]*x[0] + F[0][1]*x[1] + B[0]*accel;  // s = s + v*dt + 0.5*a*dt²
float x1_prior = F[1][0]*x[0] + F[1][1]*x[1] + B[1]*accel;  // v = v + a*dt

// 协方差预测：P = F*P*F^T + Q
// 展开为标量运算避免矩阵库依赖
```

### 更新阶段代码解析

```c
// 新息（观测残差）
float y = measured_v - (H[0]*x[0] + H[1]*x[1]);  // y = v_measured - v_estimated

// 新息协方差（简化：H=[0,1] 时 S = P[1][1] + R）
float S = HP1 + R;

// 卡尔曼增益
K[0] = P[0][1] / S;  // 位移增益
K[1] = P[1][1] / S;  // 速度增益

// 状态更新
x[0] += K[0] * y;    // 修正位移
x[1] += K[1] * y;    // 修正速度

// 协方差更新：P = (I - K*H) * P
```

---

## 8. 防打滑处理

**函数**：`Anti_Slip_Process(Robotmotion_Kalman_t *kf, float Innovation)`

### 工作原理

通过监测卡尔曼**新息**（观测值 − 预测值）的大小，判断是否发生打滑：

```
if |Innovation| > SLIP_THRESHOLD (0.5 m/s):
    R = INORMAL_R (1000.0)   ← 人为放大观测噪声
else:
    R = vel_noise            ← 恢复正常噪声
```

**效果**：当检测到打滑（速度突变导致新息异常大），将 R 增大 1000 倍，使卡尔曼增益 K 趋近于 0，从而**忽略打滑时的异常编码器读数**，靠 IMU 加速度维持状态估计。

> **注意**：当前代码中 `Anti_Slip_Process` 调用处已被注释（`Kalman.c` 第 140 行），可根据实际情况取消注释启用。

### 未来扩展（TODO 注释中提及）

- 结合 `rt_tick_get()` 计时器，对短时内的速度突变加入时间判据，减少误触发
- 互补滤波方案：`v_integrated = v_encoder*(1-α) + v_accel_integral*α`，综合两路信号判断打滑

---

## 9. 参数调节指南

### Q 和 R 的选取原则

| 参数 | 含义 | 增大效果 | 减小效果 |
|------|------|----------|----------|
| Q（过程噪声） | 对模型的不信任程度 | 更信任传感器，响应更快，噪声更多 | 更信任模型，更平滑，滞后增加 |
| R（观测噪声） | 对传感器的不信任程度 | 更信任模型预测，抗抖动能力强 | 更信任传感器，跟踪更快 |

### `Robotmotion_Kalman_t` 推荐初始参数

| 场景 | `accel_noise_std` | `velocity_noise_std` | dt |
|------|-------------------|----------------------|----|
| 正常地面行驶 | 0.5 ~ 1.0 | 0.05 ~ 0.1 | 0.001 |
| 高速/频繁加减速 | 1.0 ~ 2.0 | 0.1 ~ 0.2 | 0.001 |
| 粗糙地面（振动大） | 2.0 ~ 5.0 | 0.1 | 0.001 |

> **调参步骤**：
> 1. 先设 Q 较小、R 较大，观察输出是否平滑
> 2. 逐步减小 R，使响应变快
> 3. 若存在稳态误差，适当增大 Q

---

## 10. 使用示例

### 使用 `Robotmotion_Kalman_t`（推荐）

```c
#include "Kalman.h"

// 声明全局滤波器实例
Robotmotion_Kalman_t chassis_kf;

// 初始化（在系统初始化时调用一次）
void chassis_kalman_init(void)
{
    RobotMotion_Kalman_Init(&chassis_kf,
                            0.001f,  // 控制周期 1ms
                            1.0f,    // 加速度标准差 1 m/s²
                            0.05f);  // 速度测量标准差 0.05 m/s
}

// 控制周期内调用（1kHz 循环中）
void chassis_kalman_update(void)
{
    float accel = Get_Chassis_IMU_Data(IMU_X_ACCL); // 读取 IMU 加速度
    float speed = Get_Wheel_Speed();                 // 读取编码器速度

    RobotMotionKalman_Process(&chassis_kf, speed, accel);

    float estimated_v = RobotMotionKalman_Getvelocity(&chassis_kf);
    float estimated_s = RobotMotionKalman_Getdisplacement(&chassis_kf);
}
```

### 使用一维速度卡尔曼

```c
#include "Kalman.h"

Kalman_Forward_v_one_dimension speed_kf = {0};

void speed_kf_init(void)
{
    speed_kf.Q = 0.01f;
    speed_kf.R = 0.1f;
    speed_kf.period = 0.001f;
}

float get_filtered_speed(float raw_speed, float accel)
{
    return Kalman_calculate(&speed_kf, raw_speed, accel);
}
```

---

## 附：各滤波器对比

| 特性 | 一维速度 | 二阶高度 | 速度/角速度融合 | 机体运动 |
|------|----------|----------|-----------------|----------|
| 状态维度 | 1 | 2 | 2 | 2 |
| 依赖矩阵库 | 否 | 是 | 是 | 否 |
| 观测量数量 | 1（速度） | 2（高度+速度） | 2（左右轮速） | 1（速度） |
| 控制输入 | 加速度 | 垂直加速度 | 纵向加速度 | IMU 加速度 |
| 防打滑 | 否 | 否 | 否 | 支持（可选） |
| 推荐用途 | 单轴速度去噪 | 哨兵升降估计 | 差速底盘里程计 | 底盘主运动估计 |
