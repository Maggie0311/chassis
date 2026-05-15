# 绘制 UI 图形操作指南

## 1. 整体架构

UI 绘制系统采用三层架构，自底向上分为：

| 层级 | 文件 | 职责 |
|------|------|------|
| 底层（发送层） | [drv_send_UI.c](D:\Hero\heroec-2026\2025\rt-thread_-hero\bsp\src\PublicLib\Software\RefSystem\RefSystem_V2\UI\drv_send_UI.c) / [drv_send_UI.h](D:\Hero\heroec-2026\2025\rt-thread_-hero\bsp\src\PublicLib\Software\RefSystem\RefSystem_V2\UI\drv_send_UI.h) | 打包 UI 数据并通过串口发送给裁判系统 |
| 中间层（绘制层） | [drv_draw_UI.c](D:\Hero\heroec-2026\2025\rt-thread_-hero\bsp\src\PublicLib\Software\RefSystem\RefSystem_V2\UI\drv_draw_UI.c) / [drv_draw_UI.h](D:\Hero\heroec-2026\2025\rt-thread_-hero\bsp\src\PublicLib\Software\RefSystem\RefSystem_V2\UI\drv_draw_UI.h) | 注册 UI 元素、提供设置图形/文字属性的 API |
| 上层（应用层） | [app_UI.c](applications/ExternalData/Refsystem/Infantry_dog/app_UI.c) / [app_UI.h](applications/ExternalData/Refsystem/Infantry_dog/app_UI.h) | 兵种相关的回调函数，绘制具体 UI 内容 |
| 参数配置 | [UI_data.h](applications/ExternalData/Refsystem/Infantry_dog/UI_data.h) | 定义坐标、尺寸等图形参数常量 |

## 2. 可绘制的图形类型

系统支持 8 种 UI 元素类型（定义在 `drv_send_UI.h`）：

| 类型 | 枚举值 | 说明 |
|------|--------|------|
| `UiLine` | 直线 | 两点确定一条线段 |
| `UiRect` | 矩形 | 由左上角 (x1,y1) 和右下角 (x2,y2) 确定 |
| `UiCircle` | 正圆 | 圆心 + 半径 |
| `UiEllipse` | 椭圆 | 圆心 + x半轴 + y半轴 |
| `UiArc` | 圆弧 | 圆心 + 半轴 + 起始角 + 终止角 |
| `UiFloat` | 浮点数 | 在屏幕上显示浮点数值 |
| `UiInt` | 整数 | 在屏幕上显示整数值 |
| `UiStr` | 字符串 | 在屏幕上显示文本 |

## 3. 颜色与图层

### 3.1 颜色枚举（共 9 种）

```c
UiTeam     // 队伍主色（红方为红，蓝方为蓝）
UiYellow   // 黄色
UiGreen    // 绿色
UiOrange   // 橙色
UiMagenta  // 紫色
UiPink     // 粉色
UiCyan     // 青色
UiBlack    // 黑色
UiWhite    // 白色
```

### 3.2 图层（0 ~ 9）

图层数字越大越靠前显示（`Ui_layer_9` 在最上层）。绘制固定背景元素用低图层（如 `Ui_layer_0`），动态变化的数据用高图层。

## 4. 发送模式

注册 UI 元素时需指定发送模式（`UI_TransType_t`）：

| 发送类型 | 值 | 说明 |
|----------|-----|------|
| `TRANS_FOREVER` | -1 | 永久循环发送，适合需要持续刷新的动态数据（如角度、电量） |
| `TRANS_ONCE` | 1（或更多） | 有限次发送，适合固定不变的背景图形 |
| `TRANS_NULL` | 0 | 初始不发送，由外部条件触发显示/隐藏（如卡弹警告、菜单） |

发送周期参数 `send_period` 表示每 N 次 UI 循环发送一次（0 表示每次循环都发送）。

## 5. 屏幕坐标系

- 屏幕分辨率：**1920 × 1080**（裁判系统图传标准分辨率）
- 坐标原点：左上角 (0, 0)
- 中心点：**(960, 540)**

## 6. 绘制静态 UI（只需初始化，无需更新）

静态 UI 指绘制后内容不再变化的图形元素（如边框、背景线、固定文字标签等）。这类元素的回调函数**只使用 `UI_data.h` 中的宏常量，不访问 `UiExData`**，因此也**不需要在 `UI_Used_Data_t` 中添加任何字段**。

### 与动态 UI 的对比

| | 静态 UI | 动态 UI |
|------|----------|----------|
| 发送模式 | `TRANS_ONCE` | `TRANS_FOREVER` |
| 回调函数数据来源 | 仅 `UI_data.h` 宏常量 | `UiExData` 全局变量 |
| 是否需要在 `UI_Used_Data_t` 加字段 | **不需要** | 需要 |
| 典型用途 | 边框、背景圆、固定标签 | 电量数值、角度指示、状态文字 |
| 图层建议 | `Ui_layer_0`（底层） | `Ui_layer_4` 及以上 |

### 绘制静态 UI 的简化步骤

静态 UI 只需三步，比动态 UI 少了 `UI_Used_Data_t` 定义和数据获取两步：

#### 第一步：在 UI_data.h 中定义坐标常量

```c
// 云台姿态背景圆
#define X_OFFSET 960
#define Y_OFFSET 540
#define SCOPE_CIRCLE_RADIUS 180

// 车道线角度
#define LANE_LINE_ANGLE (40.0f / 180.0f * PI)
```

#### 第二步：编写回调函数（仅使用常量，不读 UiExData）

```c
// 绘制云台姿态背景圆 —— 纯静态，无任何外部数据依赖
void UI_GimbalPosture_Circle(UI_Object *object)
{
    UI_Set_Circle(object, UiTeam, 4, Ui_layer_0,
                  X_OFFSET, Y_OFFSET, SCOPE_CIRCLE_RADIUS);
}

// 绘制车道线 —— 纯静态
void UI_LaneLine_1(UI_Object *object)
{
    float angle = 40.0f / 180.0f * PI;
    UI_Set_Line(object, UiTeam, 4, Ui_layer_0,
                X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(angle),
                Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(angle),
                X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(angle),
                Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(angle));
}

// 绘制固定文字标签
void UI_Aimbot_Mode_Word(UI_Object *object)
{
    static char prio[] = "aim mode";
    UI_Set_String(object, UiGreen, 2, Ui_layer_0,
                  UI_AIMBOT_PRIO_CENTER_X - UI_AIMBOT_PRIO_SIZE * 8,
                  UI_AIMBOT_PRIO_CENTER_Y,
                  UI_AIMBOT_PRIO_SIZE, prio, sizeof(prio));
}
```

#### 第三步：注册为 TRANS_ONCE

```c
void Attach_UI_init()
{
    // 静态元素：TRANS_ONCE，发送一次即可
    Attach_UI_Object(UiCircle, UiTeam, TRANS_ONCE, 1, UI_GimbalPosture_Circle);
    Attach_UI_Object(UiLine,   UiTeam, TRANS_ONCE, 1, UI_LaneLine_1);
    Attach_UI_Object(UiStr,    UiGreen, TRANS_ONCE, 1, UI_Aimbot_Mode_Word);

    // 动态元素：TRANS_FOREVER，持续刷新
    Attach_UI_Object(UiFloat, UiTeam, TRANS_FOREVER, 1, UI_GimbalPosture_Pitch);
}
```

> **注意**：静态 UI 注册时仍然需要提供回调函数，因为 UI 复位（`UI_Reset_All()`）后会重新调用所有回调函数来重绘全部内容。`TRANS_ONCE` 只是控制正常运行时不再重复发送，复位时仍会被触发一次。

---

## 7. 绘制动态 UI 的完整步骤

### 步骤一：定义外部数据结构体（app_UI.h）

创建 `UI_Used_Data_t` 结构体，包含**仅动态 UI**需要用到的运行时数据字段。静态 UI 不需要在此添加任何内容。

```c
typedef struct UI_Used_Data_t {
    float RemainCapacity;       // 超级电容电量
    float GM_YAW_360F;         // 云台偏航角
    float GM_Pitch;            // 云台俯仰角
    uint8_t Strike_Stuck : 1;  // 卡弹标志
    // ... 其他动态数据字段（静态 UI 不需要加）
} UI_Used_Data_t;
```

声明全局变量 `extern UI_Used_Data_t UiExData;`。

### 步骤二：定义图形参数常量（UI_data.h）

为每个 UI 元素的位置、大小定义宏常量：

```c
// 屏幕中心
#define X_OFFSET 960
#define Y_OFFSET 540

// 超级电容
#define UI_SC_BAR_SIZE 15
#define UI_SC_BAR_CENTER_X 650
#define UI_SC_BAR_CENTER_Y 100
#define Draw_SC_Bar_WIDTH 2

// 瞄准圆
#define SCOPE_CIRCLE_RADIUS 180
```

### 步骤三：编写回调函数（app_UI.c）

每个 UI 元素对应一个回调函数，函数签名统一为 `void FuncName(UI_Object *object)`。在回调函数中调用中间层的 Set 函数设置图形属性。

#### 绘制直线
```c
void UI_Example_Line(UI_Object *object)
{
    UI_Set_Line(object, UiTeam,     // 颜色
                4,                  // 线宽
                Ui_layer_0,         // 图层
                x1, y1,             // 起点坐标
                x2, y2);            // 终点坐标
}
```

#### 绘制矩形
```c
void UI_Example_Rect(UI_Object *object)
{
    UI_Set_Rect(object, UiGreen, 2, Ui_layer_0,
                x1, y1,             // 左上角
                x2, y2);            // 右下角
}
```

#### 绘制圆
```c
void UI_Example_Circle(UI_Object *object)
{
    UI_Set_Circle(object, UiTeam, 4, Ui_layer_0,
                  x, y,             // 圆心
                  radius);          // 半径
}
```

#### 绘制圆弧
```c
void UI_Example_Arc(UI_Object *object)
{
    UI_Set_Arc(object, UiGreen, 5, Ui_layer_5,
               x, y,                // 圆心
               xSemiAxis, ySemiAxis,// x半轴, y半轴
               startAngle,          // 起始角度（度）
               endAngle);           // 终止角度（度）
}
```

#### 绘制椭圆
```c
void UI_Example_Ellipse(UI_Object *object)
{
    UI_Set_Ellipse(object, UiGreen, 3, Ui_layer_0,
                   x, y,            // 圆心
                   xSemiAxis,       // x半轴
                   ySemiAxis);      // y半轴
}
```

#### 显示浮点数
```c
void UI_Example_Float(UI_Object *object)
{
    float value = UiExData.GM_Pitch;
    UI_Set_Float(object, UiTeam, 2, Ui_layer_5,
                 x, y,              // 显示位置
                 fontSize,          // 字体大小
                 value);            // 浮点值
}
```

#### 显示整数
```c
void UI_Example_Int(UI_Object *object)
{
    int value = 100;
    UI_Set_Int(object, UiGreen, 4, Ui_layer_5,
               x, y, fontSize, value);
}
```

#### 显示字符串
```c
void UI_Example_String(UI_Object *object)
{
    static char txt[] = "Hello";
    UI_Set_String(object, UiGreen, 2, Ui_layer_4,
                  x, y,             // 显示位置
                  fontSize,         // 字体大小
                  txt,              // 字符串指针
                  sizeof(txt));     // 字符串长度（含 '\0'）
}
```

#### 动态修改颜色（不改变其他属性）
```c
void UI_Example_ColorChange(UI_Object *object)
{
    // 先设置图形，再根据条件改色
    UI_Set_Line(object, UiGreen, 4, Ui_layer_5, ...);
    if (value > 40)
        UI_Set_Color(object, UiGreen);
    else if (value > 20)
        UI_Set_Color(object, UiYellow);
    else
        UI_Set_Color(object, UiTeam);
}
```

#### 动态修改线宽
```c
UI_Set_Width(object, newWidth);
```

### 步骤四：注册 UI 对象（Attach_UI_init）

在 `Attach_UI_init()` 函数中调用 `Attach_UI_Object()` 注册每个 UI 元素：

```c
void Attach_UI_init()
{
    // Attach_UI_Object(元素类型, 颜色, 发送模式, 发送周期, 回调函数);
    Attach_UI_Object(UiLine,  UiTeam, TRANS_FOREVER, 1, UI_Example_Line);
    Attach_UI_Object(UiRect,  UiGreen, TRANS_ONCE,    1, UI_Example_Rect);
    Attach_UI_Object(UiFloat, UiTeam, TRANS_FOREVER, 1, UI_Example_Float);
    Attach_UI_Object(UiStr,   UiCyan, TRANS_NULL,    0, UI_Example_String);
}
```

**参数说明：**
- 第 1 参数：元素类型（`UiLine` / `UiRect` / `UiCircle` / `UiEllipse` / `UiArc` / `UiFloat` / `UiInt` / `UiStr`）
- 第 2 参数：默认颜色
- 第 3 参数：发送模式（`TRANS_FOREVER` / `TRANS_ONCE` / `TRANS_NULL`）
- 第 4 参数：发送周期（每 N 次循环发 1 次，0 = 每次发）
- 第 5 参数：回调函数指针

### 步骤五：编写数据获取函数（Get_UI_Data）

在 `Get_UI_Data()` 中从云台 / 底盘获取最新数据并写入 `UiExData`：

```c
void Get_UI_Data(void)
{
    UiExData.GM_YAW_360F = Get_YAW_ANGLE_360f();
    UiExData.GM_Pitch = Get_PITCH_ANGLE_360f();
    UiExData.RemainCapacity = ...;
    // ... 更新其他数据字段
}
```

### 步骤六：处理触发式显示的元素

对于需要动态显示/隐藏的元素（如卡弹警告、菜单、能量机关图形），有两种处理方式：

#### 方式一：使用 UI_Checkout_Object_Change（简单状态切换）

```c
void UI_If_Stuck_Change()
{
    static uint8_t last_state = 0;
    UI_Checkout_Object_Change(
        UiExData.Strike_Stuck,  // 当前值
        last_state,             // 上次值
        1,                      // 需要显示时的值（相等则创建，不等则删除）
        UI_StrikeStuck_Info     // 回调函数
    );
    last_state = UiExData.Strike_Stuck;
}
```

#### 方式二：使用 Drv_Set_Object_Visualable（复杂逻辑控制）

```c
void UI_If_Aimbot_Change()
{
    static uint8_t last_mode = 0;
    if (last_mode != UiExData.Aimbot_Mode)
    {
        Drv_Set_Object_Visualable(UI_MODIFY, UI_Aimbot_Mode);

        if (AIMBUFF_MODE == UiExData.Aimbot_Mode)
        {
            // 显示能量机关图形
            Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Circle);
            Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_1);
        }
        else
        {
            // 隐藏能量机关图形
            Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Circle);
            Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_1);
        }
    }
    last_mode = UiExData.Aimbot_Mode;
}
```

操作类型说明：
- `UI_CREATE` — 创建/显示元素
- `UI_MODIFY` — 修改元素属性（颜色、位置等）
- `UI_DELDTE` — 删除/隐藏元素

### 步骤七：绑定操作函数并初始化

在应用层的初始化函数中绑定注册函数和数据更新函数：

```c
void UI_Send_Init(void)
{
    UI_Set_Operation(Attach_UI_init, Get_UI_Data);
}
```

### 步骤八：在 main 中调用 UI_Init

```c
int main(void)
{
    // ... 其他初始化
    UI_Send_Init();
    if (UI_Init() != RT_EOK)
    {
        // 错误处理
    }
    // ...
}
```

## 8. 完整示例：绘制超级电容电量条

以下是一个完整的绘制示例，展示超电电量（矩形框 + 柱状填充线 + 数值 + 开关状态字符串）：

```c
// === UI_data.h ===
#define UI_SC_BAR_SIZE 15
#define UI_SC_BAR_CENTER_X 650
#define UI_SC_BAR_CENTER_Y 100
#define Draw_SC_Bar_WIDTH 2

// === app_UI.c ===

// 1. 绘制超电数值
void UI_SuperCap_Value(UI_Object *object)
{
    float remain_cap = UiExData.RemainCapacity;
    UI_Set_Float(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_5,
                 UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 7,
                 UI_SC_BAR_CENTER_Y + 0.7f * UI_SC_BAR_SIZE,
                 UI_SC_BAR_SIZE, remain_cap);
}

// 2. 绘制超电矩形边框（固定不变，仅需发送一次）
void UI_SuperCap_Border(UI_Object *object)
{
    UI_Set_Rect(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_0,
                UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 6,
                UI_SC_BAR_CENTER_Y - UI_SC_BAR_SIZE * 0.7f,
                UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 6,
                UI_SC_BAR_CENTER_Y + UI_SC_BAR_SIZE * 0.7f);
}

// 3. 绘制内部柱状线（根据电量动态变化长度和颜色）
void UI_SuperCap_Shape(UI_Object *object)
{
    float remain_cap = UiExData.RemainCapacity;
    UI_Set_Line(object, UiGreen, UI_SC_BAR_SIZE, Ui_layer_5,
                UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 5.5f,
                UI_SC_BAR_CENTER_Y,
                UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 5.5f
                    + remain_cap / 9.1f * UI_SC_BAR_SIZE,
                UI_SC_BAR_CENTER_Y);
    // 根据电量值动态改变颜色
    if (remain_cap > 40)
        UI_Set_Color(object, UiGreen);
    else if (remain_cap > 20)
        UI_Set_Color(object, UiYellow);
    else
        UI_Set_Color(object, UiTeam);
}

// 4. 绘制开关字符
void UI_SuperCap_Key(UI_Object *object)
{
    static char txt[2][10] = {" Off ", " ON "};
    uint8_t open = UiExData.IfSuperCap_open;
    UI_Set_String(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_4,
                  UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 3,
                  UI_SC_BAR_CENTER_Y + UI_SC_BAR_SIZE * 3,
                  UI_SC_BAR_SIZE, txt[open], sizeof(txt[open]));
}

// 5. 注册（在 Attach_UI_init 中调用）
Attach_UI_Object(UiFloat, UiGreen, TRANS_FOREVER, 1, UI_SuperCap_Value);
Attach_UI_Object(UiRect,  UiGreen, TRANS_ONCE,    1, UI_SuperCap_Border);
Attach_UI_Object(UiLine,  UiGreen, TRANS_FOREVER, 1, UI_SuperCap_Shape);
Attach_UI_Object(UiStr,   UiGreen, TRANS_FOREVER, 4, UI_SuperCap_Key);
```

## 9. UI 复位

当云台状态变化需要重置整个 UI 时，系统会自动调用 `UI_Reset_All()`，该函数会：
1. 删除所有图层
2. 将运行计数器归零
3. 将所有已注册元素的操作类型重置为创建，确保下次循环重绘全部内容

上层代码通过检测 `UI_Reset_Flag` 的变化来触发复位。

## 10. 注意事项

1. **刷新率限制**：裁判系统 UI 刷新率上限为 **30Hz**（间隔约 33ms），本项目设置为 25Hz，超过会导致丢帧卡顿。
2. **图形打包**：底层每次最多打包 **7 个图形元素**为一个数据包发送，超过 7 个会自动分批。
3. **字符串固定长度**：字符串发送时固定占 30 字节，超长部分会被截断。
4. **元素数量上限**：`UI_OBJECT_NUM_MAX` 默认为 150，注册的图形+字符串总数不能超过此值。
5. **浮点数精度**：底层将浮点数乘以 1000 后转为整数发送（保留 3 位小数精度）。
6. **坐标范围**：x 坐标占 11 位（0~2047），y 坐标占 11 位（0~2047），屏幕实际可用范围为 1920×1080。
7. **线宽与字号比例**：建议线宽与字体大小的比例为 **1:10**，视觉效果最佳。
8. **回调函数中不要做耗时操作**：回调在 UI 发送线程中执行，耗时过长会阻塞发送。
