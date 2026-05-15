# 过洞步兵洞对准 UI 设计文档

> **状态**: 待标定  
> **目标机型**: 2026 过洞步兵 (`CORE_USING_2026_INFANTRY_2`)  
> **功能**: 在图传画面上叠加洞对准辅助图形，帮助操作手将机器人对准洞口

---

## 一、设计思路

过洞步兵需要通过狭窄洞口。操作手在图传画面中看到洞口，但缺少参照物判断机器人是否正对洞口中心。本 UI 在屏幕中心区域绘制一组对准辅助图形：

```
┌──────────────────────────────────────────────────────────┐
│                    1920 × 1080                           │
│                                                          │
│                     ┌─────────┐                          │
│                     │ 洞口轮廓 │  ← 矩形框 (标定洞口预期  │
│                     │  矩形   │     位置和大小)           │
│                     │         │                          │
│              ───┼───┼─────────┼───┼───  ← 水平对准线     │
│                │   │    ◎    │   │                       │
│                │   │ 中心点  │   │    ← 中心十字          │
│                │   │         │   │                       │
│              ───┼───┼─────────┼───┼───  ← 水平对准线     │
│                     │         │                          │
│                     │  DONG   │                          │
│                     │  MODE   │  ← 洞模式状态文字        │
│                     └─────────┘                          │
│                                                          │
│  右上角: 对准状态 / 角度偏差                               │
│  左上角: dongMode 激活标志                                │
└──────────────────────────────────────────────────────────┘
```

### 图形元素清单

| 序号 | 元素 | 类型 | 用途 | 标定内容 |
|------|------|------|------|---------|
| 1 | 洞口外框 | `UiRect` | 标记洞口在图传中的预期位置和大小 | 左上角坐标 + 右下角坐标 |
| 2 | 中心十字-横线 | `UiLine` | 对准中心水平参考线 | 中心点坐标、线长 |
| 3 | 中心十字-竖线 | `UiLine` | 对准中心垂直参考线 | 中心点坐标、线长 |
| 4 | 中心圆点 | `UiCircle` | 对准中心标记点 | 圆心坐标、半径 |
| 5 | 左对准辅助线 | `UiLine` | 洞口左边缘标记 | 线端点坐标 |
| 6 | 右对准辅助线 | `UiLine` | 洞口右边缘标记 | 线端点坐标 |
| 7 | 洞模式状态 | `UiStr` | 显示 "DONG MODE" 或 "NORMAL" | 文字坐标 |
| 8 | 对准角度偏差 | `UiFloat` | 显示当前 yaw 偏差角度 | 文字坐标 |

---

## 二、坐标系说明

- 屏幕分辨率: **1920 × 1080**
- 原点: 左上角 (0, 0)
- X 轴: 向右为正, Y 轴: 向下为正
- 屏幕中心: `(960, 540)` 定义为 `X_OFFSET`, `Y_OFFSET`

```
(0,0)
  ┌──────────────────────────────(1920,0)
  │                                      │
  │         (960, 540) 中心              │
  │                                      │
  └──────────────────────────────(1920,1080)
```

---

## 三、标定步骤概览

1. 将机器人放在洞口正前方理想位置
2. 通过图传观察洞口在画面中的位置
3. 记录洞口的四个关键坐标（左上、右下）
4. 填入 `CustomUI_GraphParameters.h` 中对应的宏定义
5. 编译烧录，验证对准线是否与图传画面中的洞口吻合
6. 微调参数直到满意

---

## 四、代码实现清单

以下代码均需要**手动添加到对应文件**中，所有标定值用 `TODO` 标记。

### 4.1 文件 1: CustomUI_GraphParameters.h

在 `Infantry_UI/CustomUI_GraphParameters.h` 末尾（`#endif` 之前）添加：

```c
/************************过洞对准UI 标定参数*********************** */

// 洞口轮廓矩形 -- TODO: 标定洞口在画面中的位置
// 站在机器人正前方理想过洞位置，记录图传中洞口的左上和右下坐标
#define UI_HOLE_RECT_X1       600     // TODO: 标定 — 洞口左上角 X
#define UI_HOLE_RECT_Y1       300     // TODO: 标定 — 洞口左上角 Y
#define UI_HOLE_RECT_X2       1320    // TODO: 标定 — 洞口右下角 X
#define UI_HOLE_RECT_Y2       780     // TODO: 标定 — 洞口右下角 Y

// 对准中心点 -- TODO: 标定洞口中心在图传中的位置
// 通常 = 洞口矩形的几何中心
#define UI_HOLE_CENTER_X      ((UI_HOLE_RECT_X1 + UI_HOLE_RECT_X2) / 2)
#define UI_HOLE_CENTER_Y      ((UI_HOLE_RECT_Y1 + UI_HOLE_RECT_Y2) / 2)

// 中心十字线半长 -- TODO: 根据洞口大小调整
#define UI_HOLE_CROSS_HALF    40      // 十字线半长（像素）

// 中心圆点半径
#define UI_HOLE_DOT_RADIUS    6

// 洞口左右边缘标记线长度 -- TODO: 标定
#define UI_HOLE_EDGE_LINE_LEN 30      // 边缘标记线长度

// 洞模式状态文字位置 -- TODO: 标定
#define UI_HOLE_STATUS_X      740     // 状态文字 X
#define UI_HOLE_STATUS_Y      850     // 状态文字 Y
#define UI_HOLE_STATUS_SIZE   25      // 状态文字字号

// 对准偏差角度文字位置 -- TODO: 标定
#define UI_HOLE_ANGLE_X       740     // 偏差角度 X
#define UI_HOLE_ANGLE_Y       880     // 偏差角度 Y
#define UI_HOLE_ANGLE_SIZE    20      // 偏差角度字号

// 洞对准线宽
#define UI_HOLE_LINE_WIDTH    3

// 洞对准图层
#define UI_HOLE_LAYER         Ui_layer_6
```

---

### 4.2 文件 2: app_UI.h

在 `Infantry_UI/app_UI.h` 的 `UI_Used_Data_t` 结构体中，找到合适位置添加：

```c
    // === 过洞对准相关 ===
    unsigned DongMode_Active : 1;    // 洞模式是否激活, 1 为激活
    float Hole_Align_Yaw_Err;        // 洞对准 yaw 角度偏差 (度), TODO: 标定数据来源
    // ====================
```

**注意**: `DongMode_Active` 的数据源函数已在项目中存在 — `Get_if_DongMode()` 返回洞模式状态。
路径: `applications/ExternalData/Gimbal/Infantry_Gimbal/drv_Gimbal_Com.c:91`

---

### 4.3 文件 3: app_UI.c — 回调函数

在 `app_UI.c` 中添加以下回调函数（放在其他回调函数附近，如 `UI_Frequency_Info` 之后）：

```c
/* ==================== 过洞对准 UI 回调函数 ==================== */

// 1. 洞口轮廓矩形
void UI_Hole_Rect(UI_Object *object)
{
    UI_Set_Rect(object, UiGreen, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                UI_HOLE_RECT_X1, UI_HOLE_RECT_Y1,
                UI_HOLE_RECT_X2, UI_HOLE_RECT_Y2);
}

// 2. 中心十字 — 横线
void UI_Hole_Cross_H(UI_Object *object)
{
    UI_Set_Line(object, UiCyan, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                UI_HOLE_CENTER_X - UI_HOLE_CROSS_HALF, UI_HOLE_CENTER_Y,
                UI_HOLE_CENTER_X + UI_HOLE_CROSS_HALF, UI_HOLE_CENTER_Y);
}

// 3. 中心十字 — 竖线
void UI_Hole_Cross_V(UI_Object *object)
{
    UI_Set_Line(object, UiCyan, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                UI_HOLE_CENTER_X, UI_HOLE_CENTER_Y - UI_HOLE_CROSS_HALF,
                UI_HOLE_CENTER_X, UI_HOLE_CENTER_Y + UI_HOLE_CROSS_HALF);
}

// 4. 中心圆点
void UI_Hole_Dot(UI_Object *object)
{
    UI_Set_Circle(object, UiCyan, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                  UI_HOLE_CENTER_X, UI_HOLE_CENTER_Y, UI_HOLE_DOT_RADIUS);
}

// 5. 左边缘标记线 (洞口左上角向下延伸)
void UI_Hole_LeftEdge(UI_Object *object)
{
    UI_Set_Line(object, UiYellow, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                UI_HOLE_RECT_X1, UI_HOLE_RECT_Y1,
                UI_HOLE_RECT_X1, UI_HOLE_RECT_Y1 + UI_HOLE_EDGE_LINE_LEN);
}

// 6. 右边缘标记线 (洞口右上角向下延伸)
void UI_Hole_RightEdge(UI_Object *object)
{
    UI_Set_Line(object, UiYellow, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                UI_HOLE_RECT_X2, UI_HOLE_RECT_Y1,
                UI_HOLE_RECT_X2, UI_HOLE_RECT_Y1 + UI_HOLE_EDGE_LINE_LEN);
}

// 7. 洞模式状态文字
void UI_Hole_Status(UI_Object *object)
{
    static char txt[2][15] = {"NORMAL", "DONG MODE"};
    if (UiExData.DongMode_Active)
    {
        UI_Set_String(object, UiGreen, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                      UI_HOLE_STATUS_X, UI_HOLE_STATUS_Y,
                      UI_HOLE_STATUS_SIZE, txt[1], sizeof(txt[1]));
    }
    else
    {
        UI_Set_String(object, UiWhite, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                      UI_HOLE_STATUS_X, UI_HOLE_STATUS_Y,
                      UI_HOLE_STATUS_SIZE, txt[0], sizeof(txt[0]));
    }
}

// 8. 对准角度偏差
void UI_Hole_Angle_Err(UI_Object *object)
{
    float yaw_err = UiExData.Hole_Align_Yaw_Err;
    UI_Set_Float(object, UiCyan, UI_HOLE_LINE_WIDTH, UI_HOLE_LAYER,
                 UI_HOLE_ANGLE_X, UI_HOLE_ANGLE_Y,
                 UI_HOLE_ANGLE_SIZE, yaw_err);
}

/* ==================== 过洞对准 UI 回调函数 END ==================== */
```

---

### 4.4 文件 4: app_UI.c — 注册（Attach_UI_init）

在 `Attach_UI_init()` 函数中，找到合适位置（建议放在菜单系统注册之前或之后），添加：

```c
    // ========== 过洞对准 UI 注册 ==========
    // 洞口轮廓矩形
    Attach_UI_Object(UiRect,  UiGreen,  TRANS_NULL, 0, UI_Hole_Rect);
    // 中心十字线 (水平+垂直)
    Attach_UI_Object(UiLine,  UiCyan,   TRANS_NULL, 0, UI_Hole_Cross_H);
    Attach_UI_Object(UiLine,  UiCyan,   TRANS_NULL, 0, UI_Hole_Cross_V);
    // 中心圆点
    Attach_UI_Object(UiCircle, UiCyan, TRANS_NULL, 0, UI_Hole_Dot);
    // 左右边缘标记
    Attach_UI_Object(UiLine,  UiYellow, TRANS_NULL, 0, UI_Hole_LeftEdge);
    Attach_UI_Object(UiLine,  UiYellow, TRANS_NULL, 0, UI_Hole_RightEdge);
    // 洞模式状态
    Attach_UI_Object(UiStr,   UiGreen,  TRANS_NULL, 0, UI_Hole_Status);
    // 对准角度偏差
    Attach_UI_Object(UiFloat, UiCyan,   TRANS_NULL, 0, UI_Hole_Angle_Err);
    // ==========================================
```

> **说明**: 全部使用 `TRANS_NULL`（触发式发送），因为洞对准 UI 只在进入洞模式时才需要显示。

---

### 4.5 文件 5: app_UI.c — 显示/隐藏逻辑

在 `app_UI.c` 中添加洞模式切换检测函数（放在 `UI_If_Stuck_Change()` 等类似函数附近）：

```c
/**
 * @brief 检查洞模式状态变化并切换洞对准 UI 的显示
 * @note  进入洞模式 → 显示全部对准图形
 *        退出洞模式 → 删除全部对准图形
 */
void UI_If_DongMode_Change(void)
{
    static uint8_t last_dong_mode = 0;
    if (last_dong_mode != UiExData.DongMode_Active)
    {
        if (UiExData.DongMode_Active)
        {
            // 进入洞模式 — 显示所有对准辅助图形
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_Rect);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_Cross_H);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_Cross_V);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_Dot);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_LeftEdge);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_RightEdge);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_Status);
            Drv_Set_Object_Visualable(UI_CREATE, UI_Hole_Angle_Err);
        }
        else
        {
            // 退出洞模式 — 删除所有对准辅助图形
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_Rect);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_Cross_H);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_Cross_V);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_Dot);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_LeftEdge);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_RightEdge);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_Status);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Hole_Angle_Err);
        }
    }
    last_dong_mode = UiExData.DongMode_Active;
}
```

---

### 4.6 文件 6: app_UI.c — 数据采集（Get_UI_Data）

在 `Get_UI_Data()` 函数中，找到合适位置添加：

```c
    // === 过洞对准数据采集 ===
    UiExData.DongMode_Active = Get_if_DongMode();   // 读取洞模式状态
    UI_If_DongMode_Change();                        // 检查并切换洞对准 UI
    // TODO: 标定 — 对准角度偏差数据来源（可能是视觉发送的偏差值、IMU 角度等）
    // UiExData.Hole_Align_Yaw_Err = Get_Hole_Align_Yaw_Err();
    // ========================
```

> **注意**: `Hole_Align_Yaw_Err` 的数据来源需要根据实际的传感器/视觉数据链路确定，可能是：
> - 云台视觉模块通过通信协议发送的洞口偏角
> - 底盘 IMU 与目标角度的差值
> - 操作手遥控器的微调输入
>
> 标定阶段可以先设为 0，等确认数据来源后再接入。

---

## 五、标定操作流程

### 5.1 准备工作

1. 编译烧录固件到过洞步兵
2. 将机器人放在平整地面，正对洞口，保持理想过洞姿态
3. 确保图传画面正常（连接裁判系统或图传接收端）
4. 通过遥控器/键盘将机器人切换到洞模式（dongMode）

### 5.2 标定洞口矩形位置

1. 进入洞模式后，观察图传画面中洞口的位置
2. 记录洞口四个角的**图传像素坐标**：
   - 左上角: `(UI_HOLE_RECT_X1, UI_HOLE_RECT_Y1)` = `(___, ___)`
   - 右下角: `(UI_HOLE_RECT_X2, UI_HOLE_RECT_Y2)` = `(___, ___)`
3. 将数值填入 `CustomUI_GraphParameters.h`

### 5.3 标定中心十字

中心十字的坐标由洞口矩形自动计算 `(X1+X2)/2`，只需验证：
1. 十字中心是否落在图传画面中洞口的几何中心
2. 如有偏移，在标定中微调 `UI_HOLE_RECT_X1/Y1` 和 `UI_HOLE_RECT_X2/Y2`

### 5.4 标定边缘标记线

1. 观察左右边缘标记线是否对齐洞口的左右边缘
2. 调整 `UI_HOLE_EDGE_LINE_LEN` 使标记线长度合适

### 5.5 标定文字位置

1. 调整 `UI_HOLE_STATUS_X/Y` 使洞模式状态文字不遮挡洞口画面
2. 调整 `UI_HOLE_ANGLE_X/Y` 使偏差角度文字清晰可见

### 5.6 最终验证

1. 微调机器人位置（左右偏移），观察对准线与洞口的相对关系是否正确
2. 确认退出洞模式后 UI 图形全部消失
3. 确认重新进入洞模式后 UI 图形恢复显示

---

## 六、标定值记录表

| 参数宏 | 当前值 | 标定值 | 说明 |
|--------|--------|--------|------|
| `UI_HOLE_RECT_X1` | 600 | ______ | 洞口左上角 X |
| `UI_HOLE_RECT_Y1` | 300 | ______ | 洞口左上角 Y |
| `UI_HOLE_RECT_X2` | 1320 | ______ | 洞口右下角 X |
| `UI_HOLE_RECT_Y2` | 780 | ______ | 洞口右下角 Y |
| `UI_HOLE_CROSS_HALF` | 40 | ______ | 十字线半长 |
| `UI_HOLE_DOT_RADIUS` | 6 | ______ | 中心点半径 |
| `UI_HOLE_EDGE_LINE_LEN` | 30 | ______ | 边缘标记线长 |
| `UI_HOLE_STATUS_X` | 740 | ______ | 状态文字 X |
| `UI_HOLE_STATUS_Y` | 850 | ______ | 状态文字 Y |
| `UI_HOLE_STATUS_SIZE` | 25 | ______ | 状态文字字号 |
| `UI_HOLE_ANGLE_X` | 740 | ______ | 偏差角度 X |
| `UI_HOLE_ANGLE_Y` | 880 | ______ | 偏差角度 Y |
| `UI_HOLE_ANGLE_SIZE` | 20 | ______ | 偏差角度字号 |
| `UI_HOLE_LINE_WIDTH` | 3 | ______ | 线宽 |
| 角度偏差数据源 | 未接入 | ______ | `Hole_Align_Yaw_Err` 来源 |

---

## 七、涉及文件汇总

| 文件 | 操作 | 内容 |
|------|------|------|
| `Infantry_UI/CustomUI_GraphParameters.h` | 添加宏定义 | 洞口位置、尺寸、文字位置等参数 |
| `Infantry_UI/app_UI.h` | 添加数据字段 | `DongMode_Active`, `Hole_Align_Yaw_Err` |
| `Infantry_UI/app_UI.c` | 添加回调函数 | 8 个 UI 绘制回调 |
| `Infantry_UI/app_UI.c` | 修改 `Attach_UI_init` | 8 个 `Attach_UI_Object` 注册 |
| `Infantry_UI/app_UI.c` | 添加切换函数 | `UI_If_DongMode_Change` |
| `Infantry_UI/app_UI.c` | 修改 `Get_UI_Data` | 采集洞模式数据 |

---

## 八、附录: 常用颜色对照

| 枚举值 | 颜色 | 建议用途 |
|--------|------|---------|
| `UiTeam` | 红/蓝 (队伍色) | 关键信息高亮 |
| `UiGreen` | 绿色 | 正常状态、洞口轮廓 |
| `UiCyan` | 青色 | 对准十字线 |
| `UiYellow` | 黄色 | 边缘标记线 |
| `UiOrange` | 橙色 | 警告/注意 |
| `UiWhite` | 白色 | 普通文字 |
| `UiMagenta` | 紫色 | 次要标识 |
| `UiPink` | 粉色 | 辅助标识 |
