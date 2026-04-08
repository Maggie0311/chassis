
#include <rtthread.h>
#include <rtdevice.h>
#include "app_RefSystem.h" // 包含裁判系统通信协议头文件
#include "app_monitor.h"
#include "ThreadCtrl.h"
#include <math.h>
#include "drv_draw_UI.h"
#include "drv_send_UI.h"
#include "app_UI.h"
#include "CustomUI_GraphParameters.h"
#include "drv_GimMotor.h"
#include "robot_data.h"

extern rt_device_t serial_UI;
static rt_thread_t ui_thread = RT_NULL;
static struct rt_timer ui_SendTimer; // 定时器
// UI需要使用到的外部数据
UI_Used_Data_t UiExData;

/*线程定时器*/
static struct rt_semaphore UI_sem;
static void UI_IRQHandler(void *parameter)
{
    rt_sem_release(&UI_sem);
}
void Get_UI_Data(void);

// UI 线程入口函数
static void ui_thread_entry(void *parameter)
{
    CREAT_ID(id);
    ADDTOMONITOR_ID("ui_thread_entry", 1000, MONITOR_DEHANDLER, ALARM_RED, 0, id);
    SWDG_START(id);
    while (RT_TRUE)
    {
        rt_sem_take(&UI_sem, RT_WAITING_FOREVER);
        Get_UI_Data();
        Drv_Find_Send_UI_Object(); // 发送UI数据
    }
}

// 绘制超电数值
void UI_SuperCap_Value(UI_Object *object)
{
    float remain_cap = UiExData.RemainCapacity; // 获取剩余容量
    UI_Set_Float(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_5, UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 7,
                 UI_SC_BAR_CENTER_Y + 0.7f * UI_SC_BAR_SIZE, UI_SC_BAR_SIZE, remain_cap);
}

// 绘制超电矩形框
void UI_SuperCap_Border(UI_Object *object)
{
    UI_Set_Rect(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_0, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 6,
                UI_SC_BAR_CENTER_Y - UI_SC_BAR_SIZE * 0.7f,
                UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 6,
                UI_SC_BAR_CENTER_Y + UI_SC_BAR_SIZE * 0.7f);
}

// 绘制超电矩形框内部的柱状线
void UI_SuperCap_Shape(UI_Object *object)
{
    float remain_cap = UiExData.RemainCapacity; // 获取剩余容量
    UI_Set_Line(object, UiGreen, UI_SC_BAR_SIZE, Ui_layer_5, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 5.5f, UI_SC_BAR_CENTER_Y,
                UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 5.5f + remain_cap / 9.1f * UI_SC_BAR_SIZE,
                UI_SC_BAR_CENTER_Y); // 值大于40时显示绿色
    if (remain_cap > 40)
        UI_Set_Color(object, UiGreen); // 值大于40时显示绿色
    else if (remain_cap > 20)
        UI_Set_Color(object, UiYellow); // 值位于20-40时显示黄色
    else
        UI_Set_Color(object, UiTeam); // 值小于20时显示红蓝主色
}

// 绘制超电的开关字符
void UI_SuperCap_Key(UI_Object *object)
{
    static char txt1[2][10] = {" Off ", " ON "};
    uint8_t sc_remain = UiExData.IfSuperCap_open;
    if (sc_remain == 0)
    {
        UI_Set_String(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_4, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 3,
                      UI_SC_BAR_CENTER_Y + UI_SC_BAR_SIZE * 3, UI_SC_BAR_SIZE, txt1[0], sizeof(txt1[0]));
    }
    else if (sc_remain == 1)
    {
        UI_Set_String(object, UiGreen, Draw_SC_Bar_WIDTH, Ui_layer_4, UI_SC_BAR_CENTER_X - UI_SC_BAR_SIZE * 3,
                      UI_SC_BAR_CENTER_Y + UI_SC_BAR_SIZE * 3, UI_SC_BAR_SIZE, txt1[1], sizeof(txt1[1]));
    }
}

// 绘制无线充电标识
void UI_Wireless_Charge_Arc_1(UI_Object *object)
{ // 绘制在超电条的右侧
    UI_Set_Arc(object, UiGreen, 5, Ui_layer_5, UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 11, UI_SC_BAR_CENTER_Y - UI_SC_BAR_SIZE * 0.7f, 10, 10, 320, 40);
}
void UI_Wireless_Charge_Arc_2(UI_Object *object)
{ // 绘制在超电条的右侧
    UI_Set_Arc(object, UiGreen, 5, Ui_layer_5, UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 11, UI_SC_BAR_CENTER_Y - UI_SC_BAR_SIZE * 0.7f, 20, 20, 320, 40);
}
void UI_Wireless_Charge_Arc_3(UI_Object *object)
{ // 绘制在超电条的右侧
    UI_Set_Arc(object, UiGreen, 5, Ui_layer_5, UI_SC_BAR_CENTER_X + UI_SC_BAR_SIZE * 11, UI_SC_BAR_CENTER_Y - UI_SC_BAR_SIZE * 0.7f, 30, 30, 320, 40);
}

// 绘制云台的姿态信息（圆）
void UI_GimbalPosture_Circle(UI_Object *object)
{
    UI_Set_Circle(object, UiTeam, 4, Ui_layer_0, X_OFFSET, Y_OFFSET, SCOPE_CIRCLE_RADIUS);
}

// 绘制云台的姿态信息（三条短线）
// 绘制云台的姿态信息（第一条短线）
void UI_GimbalPosture_Line_1(UI_Object *object)
{
    float yaw_rel = -UiExData.GM_YAW_360F; // 获取偏航角
    rt_int8_t i = 0;
    float k = (i == 1) ? 0.6f : 0.8f;
    UI_Set_Line(object, UiTeam, 4, Ui_layer_5, X_OFFSET - SCOPE_CIRCLE_RADIUS * k * sinf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                Y_OFFSET + SCOPE_CIRCLE_RADIUS * k * cosf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                Y_OFFSET + SCOPE_CIRCLE_RADIUS * cosf(yaw_rel * angleTorad + (i - 1) * PI / 2));
}
// 绘制云台的姿态信息（第二条长线）
void UI_GimbalPosture_Line_2(UI_Object *object)
{
    float yaw_rel = -UiExData.GM_YAW_360F; // 获取偏航角
    rt_int8_t i = 1;
    float k = (i == 1) ? 0.6f : 0.8f;
    UI_Set_Line(object, UiTeam, 6, Ui_layer_5, X_OFFSET - SCOPE_CIRCLE_RADIUS * k * sinf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                Y_OFFSET + SCOPE_CIRCLE_RADIUS * k * cosf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                Y_OFFSET + SCOPE_CIRCLE_RADIUS * cosf(yaw_rel * angleTorad + (i - 1) * PI / 2));

} // 绘制云台的姿态信息（第三条短线）
void UI_GimbalPosture_Line_3(UI_Object *object)
{
    float yaw_rel = -UiExData.GM_YAW_360F; // 获取偏航角
    rt_int8_t i = 2;
    float k = (i == 1) ? 0.6f : 0.8f;
    UI_Set_Line(object, UiTeam, 4, Ui_layer_5, X_OFFSET - SCOPE_CIRCLE_RADIUS * k * sinf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                Y_OFFSET + SCOPE_CIRCLE_RADIUS * k * cosf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(yaw_rel * angleTorad + (i - 1) * PI / 2),
                Y_OFFSET + SCOPE_CIRCLE_RADIUS * cosf(yaw_rel * angleTorad + (i - 1) * PI / 2));
}
// 绘制云台的姿态信息（pitch字符）
void UI_GimbalPosture_Pitch_word(UI_Object *object)
{
    static char txt1[10] = "Pitch"; // 俯仰角
    UI_Set_String(object, UiTeam, 2, Ui_layer_5, UI_ABS_PITCH_CENTER_X, UI_ABS_PITCH_CENTER_Y + UI_PITCH_SIZE * 1.5f,
                  UI_PITCH_SIZE, txt1, sizeof(txt1));
}

// 绘制云台的姿态信息（pitch具体数值）
void UI_GimbalPosture_Pitch(UI_Object *object)
{
    float pitch = UiExData.GM_Pitch; // 获取俯仰角
    UI_Set_Float(object, UiTeam, 2, Ui_layer_5, UI_ABS_PITCH_CENTER_X, UI_ABS_PITCH_CENTER_Y,
                 UI_PITCH_SIZE, pitch);
}

// 绘制堵弹标志
void UI_StrikeStuck_Info(UI_Object *object)
{
    static char txt[] = "STuck ! ! !";
    uint8_t stuck = UiExData.Strike_Stuck;
    if (stuck)
    {
        // 堵弹后，回调函数设置堵弹字符显示
        UI_Set_String(object, UiOrange, 3, Ui_layer_5, 120, 820, 17, txt, sizeof(txt));
    }
}

// int x1_aimbot=690;
// int y1_aimbot=410;
// int x2_aimbot=1260;
// int y2_aimbot=660;
// 相机位置辅助矩形框
void UI_Camera_Assist_Rect(UI_Object *object)
{
    // if(AIMBUFF_CONST_MODE==UiExData.Aimbot_Mode||AIMBUFF_VAR_MODE==UiExData.Aimbot_Mode)
    // {
    //     UI_Set_Rect(object, UiWhite, 2, Ui_layer_5, x1_energy, y1_energy, x2_energy, y2_energy);
    // }
    // else
    // {
    // UI_Set_Rect(object, UiWhite, 2, Ui_layer_5, x1_aimbot, y1_aimbot, x2_aimbot, y2_aimbot);
//    if (UiExData.visual_target_found)
//    {
//        UI_Set_Rect(object, UiTeam, 2, Ui_layer_5, 690, 410, 1260, 660);
//    }
//    else if (UiExData.visual_target_found == 0)
//    {
        UI_Set_Rect(object, UiWhite, 2, Ui_layer_5, 690, 410, 1260, 660);
//    }
    // }
}
// int motion_mode_x=740;
// int motion_mode_y=800;
// int motion_mode_fontsize=60;
// 显示当前底盘运动模式
void UI_Motion_Mode(UI_Object *object)
{
    static char txt[5][15] = {"No Follow", "Follow", "Slow Gyro", "Fast Gyro", "Change Gyro"};
    int motionmode = 0;
    motionmode = UiExData.Motion_Mode; // 得到当前运动模式
    if (motionmode >= UI_NO_FOLLOW && motionmode <= UI_Change_GYRO)
    {
        UI_Set_String(object, motionmode, 6, Ui_layer_5, 740, 800, 60, txt[motionmode], sizeof(txt[motionmode]));
    }
}

// 自瞄错误显示
void UI_Aimbot_Err_Info(UI_Object *object)
{
    static char statechar[3][25] = {"Aimbot ComErr!", "Aimbot Work Failure!"};
    Visual_Error_State_Enum Vsstate = UiExData.Visual_Error_State; // 云台认为当前自瞄是否工作正常
    if (Vsstate == Visual_Working_EOK)
    {
        UI_Set_String(object, UiYellow, 3, Ui_layer_2, 120, 780, UI_AIMBOT_MODE_SIZE, statechar[0], sizeof(statechar[0]));
    }
    else if (Vsstate == Visual_Com_Failure)
    {
        UI_Set_String(object, UiYellow, 3, Ui_layer_2, 120, 780, UI_AIMBOT_MODE_SIZE, statechar[1], sizeof(statechar[1]));
    }
    else if (Vsstate == Visual_Working_Failure)
    {
        UI_Set_String(object, UiYellow, 3, Ui_layer_2, 120, 780, UI_AIMBOT_MODE_SIZE, statechar[2], sizeof(statechar[2]));
    }
}
// 自瞄模式显示
void UI_Aimbot_Mode(UI_Object *object)
{
    static char string[3][15] = {"aimbot", "aimbuf small", "aimbuf large"};
    int mode = 0;
    mode = UiExData.Aimbot_Mode; // 得到当前自瞄模式
    if (mode >= AIMBOT_MODE && mode <= AIMBUFF_VAR_MODE)
    {
        UI_Set_String(object, UiCyan, 2, Ui_layer_5, UI_AIMBOT_PRIO_CENTER_X + UI_AIMBOT_MODE_SIZE * 8, UI_AIMBOT_PRIO_CENTER_Y,
                      UI_AIMBOT_MODE_SIZE, string[mode], sizeof(string[mode]));
    }
}

// 自瞄模式显示（字符）
void UI_Aimbot_Mode_Word(UI_Object *object)
{
    static char prio[] = "aim mode";
    UI_Set_String(object, UiGreen, 2, Ui_layer_0, UI_AIMBOT_PRIO_CENTER_X - UI_AIMBOT_PRIO_SIZE * 8, UI_AIMBOT_PRIO_CENTER_Y, UI_AIMBOT_PRIO_SIZE, prio, sizeof(prio));
}

// 自瞄颜色显示（字符）
void UI_Aimbot_Color_Word(UI_Object *object)
{
    static char color_txt[] = "aim color";
    UI_Set_String(object, UiGreen, 2, Ui_layer_0, UI_AIMBOT_COLOR_CENTER_X - UI_AIMBOT_MODE_SIZE * 4, UI_AIMBOT_COLOR_CENTER_Y,
                  UI_AIMBOT_COLOR_SIZE, color_txt, sizeof(color_txt));
}

// 自瞄颜色显示
void UI_Aimbot_Color(UI_Object *object)
{
    static char color_txt[2][8] = {"BLUE", "RED"};
    int color = 0;
    color = UiExData.Color_Myself; // 得到当前自瞄的己方颜色
    UI_Set_String(object, color, 2, Ui_layer_5, UI_AIMBOT_COLOR_CENTER_X + UI_AIMBOT_COLOR_SIZE * 8, UI_AIMBOT_COLOR_CENTER_Y,
                  UI_AIMBOT_COLOR_SIZE, color_txt[color], sizeof(color_txt[color]));
}

// 缓冲能量显示数值 当数值变成15一下变色
void UI_PowerBuff_Value(UI_Object *object)
{
    uint16_t powerbuff = UiExData.powerbuff;
    if (powerbuff <= 15)
    {
        UI_Set_Int(object, UiGreen,4,Ui_layer_5,550,200,80,powerbuff);
        UI_Set_Color(object, UiOrange);
    }
    else
    {
        UI_Set_Int(object, UiGreen,4,Ui_layer_5,550,200,30,powerbuff);
        UI_Set_Color(object, UiGreen);
    }
}
//  绘制车道线第1条
void UI_LaneLine_1(UI_Object *object)
{
    float angle = 40.0f / 180.0f * PI;
    UI_Set_Line(object, UiTeam, 4, Ui_layer_0, X_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * sinf(angle), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(angle), X_OFFSET - SCOPE_CIRCLE_RADIUS * sinf(angle), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(angle));
}

//  绘制车道线第2条
void UI_LaneLine_2(UI_Object *object)
{
    float angle = 40.0f / 180.0f * PI;
    UI_Set_Line(object, UiTeam, 4, Ui_layer_0, X_OFFSET + OUTER_SCOPE_CIRCLE_RADIUS * sinf(angle), Y_OFFSET - OUTER_SCOPE_CIRCLE_RADIUS * cosf(angle), X_OFFSET + SCOPE_CIRCLE_RADIUS * sinf(angle), Y_OFFSET - SCOPE_CIRCLE_RADIUS * cosf(angle));
}

//  绘制子弹模式--第一颗子弹半圆弧
void Strike_Mode_Info_Arc_1(UI_Object *object)
{
    UI_Set_Arc(object, UiCyan, 2, Ui_layer_0, UI_STRIKE_MODE_CENTER_X, UI_STRIKE_MODE_CENTER_Y + UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_SIZE, 2 * UI_STRIKE_MODE_SIZE, 270, 90);
}

//  绘制子弹模式--第二颗子弹半圆弧
void Strike_Mode_Info_Arc_2(UI_Object *object)
{
    UI_Set_Arc(object, UiCyan, 2, Ui_layer_0, UI_STRIKE_MODE_CENTER_X - UI_STRIKE_MODE_SIZE * 3, UI_STRIKE_MODE_CENTER_Y + UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_SIZE, 2 * UI_STRIKE_MODE_SIZE, 270, 90);
}

//  绘制子弹模式--第三颗子弹半圆弧
void Strike_Mode_Info_Arc_3(UI_Object *object)
{
    UI_Set_Arc(object, UiCyan, 2, Ui_layer_0, UI_STRIKE_MODE_CENTER_X + UI_STRIKE_MODE_SIZE * 3, UI_STRIKE_MODE_CENTER_Y + UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_SIZE, 2 * UI_STRIKE_MODE_SIZE, 270, 90);
}
//  绘制子弹模式--第一颗子弹矩形
void Strike_Mode_Info_Rect_1(UI_Object *object)
{
    UI_Set_Rect(object, UiCyan, 2, Ui_layer_0, UI_STRIKE_MODE_CENTER_X - UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_CENTER_Y - 3 * UI_STRIKE_MODE_SIZE,
                UI_STRIKE_MODE_CENTER_X + UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_CENTER_Y + UI_STRIKE_MODE_SIZE);
}
//  绘制子弹模式--第二颗子弹矩形
void Strike_Mode_Info_Rect_2(UI_Object *object)
{
    UI_Set_Rect(object, UiCyan, 2, Ui_layer_0, UI_STRIKE_MODE_CENTER_X - UI_STRIKE_MODE_SIZE * 3 - UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_CENTER_Y - 3 * UI_STRIKE_MODE_SIZE,
                UI_STRIKE_MODE_CENTER_X - UI_STRIKE_MODE_SIZE * 3 + UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_CENTER_Y + UI_STRIKE_MODE_SIZE);
}

//  绘制子弹模式--第三颗子弹矩形
void Strike_Mode_Info_Rect_3(UI_Object *object)
{
    UI_Set_Rect(object, UiCyan, 2, Ui_layer_0, UI_STRIKE_MODE_CENTER_X + UI_STRIKE_MODE_SIZE * 3 - UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_CENTER_Y - 3 * UI_STRIKE_MODE_SIZE,
                UI_STRIKE_MODE_CENTER_X + UI_STRIKE_MODE_SIZE * 3 + UI_STRIKE_MODE_SIZE, UI_STRIKE_MODE_CENTER_Y + UI_STRIKE_MODE_SIZE);
}
int aim_point_energy_x = 960;
int aim_point_energy_y = 495;
int aim_point_normal_x = 945;
int aim_point_normal_y = 490;
//! 云台(视觉)目前还没加些功能
// 绘制准心位置（这里分为打符模式和普通自瞄模式两种情况，并且在能量机关拟合结束，或者自瞄最佳距离时会自动变色为队伍颜色）
void UI_Aim_Point_Info(UI_Object *object)
{
    if (UiExData.Aimbot_Mode == AIMBUFF_CONST_MODE || UiExData.Aimbot_Mode == AIMBUFF_VAR_MODE)
    {
        // if (UiExData.Energy_is_already == 1)
        // {
        //     UI_Set_Circle(object, UiTeam, 3, Ui_layer_0, aim_point_energy_x, aim_point_energy_y, 15);
        // }
        // else
        // {
        //     UI_Set_Circle(object, UiWhite, 3, Ui_layer_0, aim_point_energy_x, aim_point_energy_y, 15);
        // }
    }
    else
    {
        //!云台(视觉)目前还没加些功能
        // if (UiExData.if_aimbot_best_dis) // 附加功能：用来判断是否为最佳距离，如果是最佳距离，则将颜色置为队伍颜色，否则置为白色
        // {
        //     UI_Set_Circle(object, UiTeam, 3, Ui_layer_0, aim_point_normal_x, aim_point_normal_y, 15);
        // }
        // else
        // {
        //     UI_Set_Circle(object, UiWhite, 3, Ui_layer_0, aim_point_normal_x, aim_point_normal_y, 15);
        // }
    }
}
// 绘制视觉能量机关拟合成功后的能量机关图形
// 绘制能量机关中心圆圈
// void UI_EnergyR_Circle(UI_Object *object)
// {
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Circle(object, UiTeam, 10, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, 8);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Circle(object, UiWhite, 10, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, 8);
//     }
// }
// 绘制能量机关第一条竖直线
// void UI_EnergyR_Line_1(UI_Object *object)
// {
//     float angle = 0.f;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Line(object, UiTeam, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Line(object, UiWhite, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
// }
// 绘制能量机关第一条竖直线上的圆圈
// void UI_EnergyR_Line_Circle_1(UI_Object *object)
// {
//     float angle = 0.f;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Circle(object, UiTeam, 2, Ui_layer_0, x2, y2, 4);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Circle(object, UiWhite, 2, Ui_layer_0, x2, y2, 4);
//     }
// }
// 绘制能量机关第二条竖直线
// void UI_EnergyR_Line_2(UI_Object *object)
// {
//     float angle = 72.f;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Line(object, UiTeam, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Line(object, UiWhite, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
// }
// 绘制能量机关第二条竖直线上的圆圈
// void UI_EnergyR_Line_Circle_2(UI_Object *object)
// {
//     float angle = 72.f * 1;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Circle(object, UiTeam, 2, Ui_layer_0, x2, y2, 4);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Circle(object, UiWhite, 2, Ui_layer_0, x2, y2, 4);
//     }
// }
// 绘制能量机关第三条竖直线
// void UI_EnergyR_Line_3(UI_Object *object)
// {
//     float angle = 72.f * 2;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Line(object, UiTeam, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Line(object, UiWhite, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
// }
// 绘制能量机关第三条竖直线上的圆圈
// void UI_EnergyR_Line_Circle_3(UI_Object *object)
// {
//     float angle = 72.f * 2;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Circle(object, UiTeam, 2, Ui_layer_0, x2, y2, 4);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Circle(object, UiWhite, 2, Ui_layer_0, x2, y2, 4);
//     }
// }
// 绘制能量机关第四条竖直线
// void UI_EnergyR_Line_4(UI_Object *object)
// {
//     float angle = 72.f * 3;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Line(object, UiTeam, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Line(object, UiWhite, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
// }
// 绘制能量机关第四条竖直线上的圆圈
// void UI_EnergyR_Line_Circle_4(UI_Object *object)
// {
//     float angle = 72.f * 3;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Circle(object, UiTeam, 2, Ui_layer_0, x2, y2, 4);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Circle(object, UiWhite, 2, Ui_layer_0, x2, y2, 4);
//     }
// }
// 绘制能量机关第五条竖直线
// void UI_EnergyR_Line_5(UI_Object *object)
// {
//     float angle = 72.f * 4;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Line(object, UiTeam, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Line(object, UiWhite, 3, Ui_layer_0, UI_ENERGY_CENTER_X, UI_ENERGY_CENTER_Y, x2, y2);
//     }
// }
// 绘制能量机关第五条竖直线上的圆圈
// void UI_EnergyR_Line_Circle_5(UI_Object *object)
// {
//     float angle = 72.f * 4;
//     float x2 = UI_ENERGY_CENTER_X + UI_ENERGY_LINE_LENGTH * sinf(angle * PI / 180);
//     float y2 = UI_ENERGY_CENTER_Y + UI_ENERGY_LINE_LENGTH * cosf(angle * PI / 180);
//     if (UiExData.Energy_is_already) // 如果能量机关拟合结束，则绘制能量机关图形，设置颜色为队伍颜色
//     {
//         UI_Set_Circle(object, UiTeam, 2, Ui_layer_0, x2, y2, 4);
//     }
//     else // 如果能量机关拟合未结束，且打开能量机关自瞄模式，则绘制能量机关图形，设置颜色为白色
//     {
//         UI_Set_Circle(object, UiWhite, 2, Ui_layer_0, x2, y2, 4);
//     }
// }
// 菜单文本数组（第一个下标表示8种菜单，第二个下标表示7行数据，第三个下标表示每行20个字符（注意结尾‘\0’））
static char menu_txt[9][7][20] = {
    {"", "", "", "", "", "", ""},
    {"MOTIONMODE", "G. NOFOLLOW", "W. FOLLOW", "R. SLOWGYRO", "E. FASTGYRO", "C. MOVEBACK_GIMBAL", "Z. MOVEBACK_CHASSIS"},
    {"AIMBOTMODE", "G. AIMBOT", "Q. aimbuf small", "V.aimbuf large", "", "", ""},
    {"MISC", "Z. UI_RESET", "G. SC_CTL", "R. ROBOT_RESET", "", "", ""},
    {"MISC-RESET", "G. GIMBAL", "C. CHASSIS", "B. ALL_RESET", "", "", ""},
    {"MISC-SC_CTL", "C. OPEN_SC", "V. ClOSE_SC", "", "", "", ""},
    {"RUB_ROTATE_SPEED", "Z. +10", "X. -10", "C. +35", "V. -35", "", ""},
    {"Performance", "Z. Level 1", "X. Level 2", "C. Level 3", "B. Other_SET", "", ""},
    {"Performance_Choose", "Z. Burst", "C. Speed", "V. Power", "B. HP", "G. REF_YES", "R. REF_NO"}};
// 绘制菜单系统
void MenuSystem_Info_Line1(UI_Object *object)
{
    int i = 0; // 第一行是第0个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiYellow, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}
void MenuSystem_Info_Line2(UI_Object *object)
{
    int i = 1; // 第二行是第1个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiGreen, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}
void MenuSystem_Info_Line3(UI_Object *object)
{
    int i = 2; // 第三行是第2个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiOrange, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}
void MenuSystem_Info_Line4(UI_Object *object)
{
    int i = 3; // 第四行是第3个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiMagenta, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}
void MenuSystem_Info_Line5(UI_Object *object)
{
    int i = 4; // 第五行是第4个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiPink, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}
void MenuSystem_Info_Line6(UI_Object *object)
{
    int i = 5; // 第六行是第5个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiCyan, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}
void MenuSystem_Info_Line7(UI_Object *object)
{
    int i = 6; // 第七行是第6个元素
    if (UiExData.MenuOption >= MainMenu && UiExData.MenuOption <= PerformanceChooseMenu)
    {
        UI_Set_String(object, UiBlack, 2, Ui_layer_0, UI_MENU_START_X, UI_MENU_START_Y - UI_MENU_LINE_SPACE * i, UI_MENU_FONT_SIZE, menu_txt[UiExData.MenuOption][i], sizeof(menu_txt[UiExData.MenuOption][i]));
    }
}

// UI帧率显示
void UI_Frequency_Info(UI_Object *object)
{
    rt_time_t current_time = rt_tick_get();                                     // 获取当前系统时间
    float frequency = 1000.0f / (current_time - UiExData.Last_UI_refresh_Time); // 计算帧率
    UiExData.Last_UI_refresh_Time = current_time;                               // 更新上次UI时间
    UI_Set_Int(object, UiTeam, 4, Ui_layer_5, 80, 540,
               20, frequency); // 绘制帧率信息
}

//英雄辅助瞄准框 分别对应1m 3m 5m 7m时装甲板的大小
void UI_Hero_Aim_Rect_1(UI_Object *object)
{
    UI_Set_Rect(object, UiGreen, 2, Ui_layer_0, 875, 430, 980, 490);
}

void UI_Hero_Aim_Rect_2(UI_Object *object)
{
    UI_Set_Rect(object, UiOrange, 2, Ui_layer_0, 905, 430, 955, 460);
}

void UI_Hero_Aim_Rect_3(UI_Object *object)
{
    UI_Set_Rect(object, UiMagenta, 2, Ui_layer_0, 912, 380, 945, 395);
}

void UI_Hero_Aim_Rect_4(UI_Object *object)
{
    UI_Set_Rect(object, UiPink, 2, Ui_layer_0, 915, 297, 942, 307);
}

// 英雄是否允许超热量射击
void UI_Hero_Allow_Heat_Shoot(UI_Object *object)
{
    static char txt[2][15] = {"Allow Shoot", "Forbid Shoot"};
    if (UiExData.IfAllow_Heat_Shoot == 1)
    {
        UI_Set_String(object, UiGreen, 2, Ui_layer_0, 960, 200, 25, txt[0], sizeof(txt[0]));
    }
    else
    {
        UI_Set_String(object, UiOrange, 2, Ui_layer_0, 960, 200, 25, txt[1], sizeof(txt[1]));
    }
}
/// <summary>
///                 注册部分初始化
/// </summary>
void Attach_UI_init()
{
    // 超级电容UI
    Attach_UI_Object(UiFloat, UiGreen, TRANS_FOREVER, 1, UI_SuperCap_Value); // 绘制超电数值
    Attach_UI_Object(UiRect, UiGreen, TRANS_ONCE, 1, UI_SuperCap_Border);    // 绘制超电矩形框
    Attach_UI_Object(UiLine, UiGreen, TRANS_FOREVER, 1, UI_SuperCap_Shape);  // 绘制超电矩形框内部的柱状线
    Attach_UI_Object(UiStr, UiGreen, TRANS_FOREVER, 4, UI_SuperCap_Key);     // 绘制超电的开关字符
    // // 绘制无线充电标志
    // Attach_UI_Object(UiArc, UiCyan, TRANS_NULL, 0, UI_Wireless_Charge_Arc_1); //
    // Attach_UI_Object(UiArc, UiCyan, TRANS_NULL, 0, UI_Wireless_Charge_Arc_2); //
    // Attach_UI_Object(UiArc, UiCyan, TRANS_NULL, 0, UI_Wireless_Charge_Arc_3);

    // 云台姿态UI
    Attach_UI_Object(UiCircle, UiTeam, TRANS_ONCE, 1, UI_GimbalPosture_Circle);  // 绘制云台的姿态信息（圆）
    Attach_UI_Object(UiLine, UiTeam, TRANS_FOREVER, 1, UI_GimbalPosture_Line_1); // 绘制云台的姿态信息（三条短线）
    Attach_UI_Object(UiLine, UiTeam, TRANS_FOREVER, 1, UI_GimbalPosture_Line_2); // 绘制云台的姿态信息（三条短线）
    Attach_UI_Object(UiLine, UiTeam, TRANS_FOREVER, 1, UI_GimbalPosture_Line_3); // 绘制云台的姿态信息（三条短线）
    Attach_UI_Object(UiStr, UiTeam, TRANS_ONCE, 1, UI_GimbalPosture_Pitch_word); // 绘制云台的姿态信息（pitch字符）
    Attach_UI_Object(UiFloat, UiTeam, TRANS_FOREVER, 1, UI_GimbalPosture_Pitch); // 绘制云台的姿态信息（pitch具体数值）

    // 堵弹标志UI
    Attach_UI_Object(UiStr, UiOrange, TRANS_NULL, 0, UI_StrikeStuck_Info); // 绘制堵弹标志

    // 相机位置辅助矩形框UI
    //Attach_UI_Object(UiRect, UiWhite, TRANS_ONCE, 1, UI_Camera_Assist_Rect); // 绘制相机位置辅助矩形框

    // 显示当前底盘运动模式
    Attach_UI_Object(UiStr, UiYellow, TRANS_FOREVER, 5, UI_Motion_Mode); // 显示当前底盘运动模式

    // 自瞄相关信息
    Attach_UI_Object(UiStr, UiYellow, TRANS_NULL, 1, UI_Aimbot_Err_Info);  // 自瞄错误显示
    Attach_UI_Object(UiStr, UiCyan, TRANS_ONCE, 1, UI_Aimbot_Mode);        // 自瞄模式显示
    Attach_UI_Object(UiStr, UiGreen, TRANS_ONCE, 1, UI_Aimbot_Mode_Word);  // 自瞄模式显示（字符）
    Attach_UI_Object(UiStr, UiGreen, TRANS_ONCE, 1, UI_Aimbot_Color_Word); //  自瞄颜色显示（字符）
    Attach_UI_Object(UiStr, UiTeam, TRANS_ONCE, 1, UI_Aimbot_Color);       //  自瞄颜色显示

    //绘制缓冲能量相关提示
    Attach_UI_Object(UiInt,UiGreen,TRANS_FOREVER,1,UI_PowerBuff_Value);//缓冲能量显示数值 当数值变成15一下变色

    // 绘制车道线
    Attach_UI_Object(UiLine, UiTeam, TRANS_ONCE, 1, UI_LaneLine_1); //  绘制车道线第1条
    Attach_UI_Object(UiLine, UiTeam, TRANS_ONCE, 1, UI_LaneLine_2); //  绘制车道线第2条

    // 能量机关(视觉拟合状态和开启能量机关自瞄模式时显示) 能量机关和准心
    //Attach_UI_Object(UiCircle, UiWhite, TRANS_ONCE, 1, UI_Aim_Point_Info);        // 能量机关中心图标圆圈
    
    // 绘制菜单选项卡(这里的每一行都是一个单独的菜单元素，因此不同的菜单实际上需要修改每一行的内容即可)
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line1);
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line2);
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line3);
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line4);
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line5);
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line6);
    Attach_UI_Object(UiStr, UiCyan, TRANS_NULL, 0, MenuSystem_Info_Line7); // 一共有7行菜单选项

    // UI帧率显示
    Attach_UI_Object(UiFloat, UiTeam, TRANS_FOREVER, 1, UI_Frequency_Info); // 帧率显示
    // 绘制英雄需要的瞄准线
    Attach_UI_Object(UiRect, UiGreen, TRANS_ONCE, 1, UI_Hero_Aim_Rect_1);   // 绘制英雄瞄准线第1条
    Attach_UI_Object(UiRect, UiOrange, TRANS_ONCE, 1, UI_Hero_Aim_Rect_2);  // 绘制英雄瞄准线第2条
    Attach_UI_Object(UiRect, UiMagenta, TRANS_ONCE, 1, UI_Hero_Aim_Rect_3); // 绘制英雄瞄准线第3条
    Attach_UI_Object(UiRect, UiPink, TRANS_ONCE, 1, UI_Hero_Aim_Rect_4);    // 绘制英雄瞄准线第4条
    // 绘制英雄是否允许超热量射击的矩形框
    Attach_UI_Object(UiStr, UiOrange, TRANS_ONCE, 1, UI_Hero_Allow_Heat_Shoot); // 绘制英雄是否允许超热量射击
    // todo英雄特有需求UI绘制

    // Attach_UI_Object(UiCircle, UiGreen, TRANS_FOREVER, 1, UI_Guide_Light_Circle); // 英雄吊射基地引导灯
    // Attach_UI_Object(UiCircle, UiTeam, TRANS_FOREVER, 1, UI_Big_Bullet_Circle);   // 英雄大弹丸
}

// 检查无线充电状态变化并更新UI显示
// void UI_If_Wireless_Charge_Change()
// {
//     static uint8_t last_wireless_charge = 0; // 无线充电上次保存状态
//     UI_Checkout_Object_Change(UiExData.IfWireless_Charge, last_wireless_charge, 1, UI_Wireless_Charge_Arc_1);
//     UI_Checkout_Object_Change(UiExData.IfWireless_Charge, last_wireless_charge, 1, UI_Wireless_Charge_Arc_2);
//     UI_Checkout_Object_Change(UiExData.IfWireless_Charge, last_wireless_charge, 1, UI_Wireless_Charge_Arc_3);
//     last_wireless_charge = UiExData.IfWireless_Charge; // 更新无线充电状态
// }
// 检查堵弹状态变化并更新UI显示
void UI_If_Stuck_Change()
{
    static uint8_t last_strike_stuck = 0; // 卡弹上次保存状态
    UI_Checkout_Object_Change(UiExData.Strike_Stuck, last_strike_stuck, 1, UI_StrikeStuck_Info);
    last_strike_stuck = UiExData.Strike_Stuck; // 更新卡弹状态，堵弹时标志位为1
}
// 检查自瞄状态并更新UI显示（包括更新右下角的字符，如果是能量机关模式，会显示能量机关的UI，如果能量机关拟合状态完成，则还会将颜色设置为队伍颜色）
void UI_If_Aimbot_Change()
{
    static uint8_t last_aimbot_state = 0;          // 自瞄上次保存状态
    if (last_aimbot_state != UiExData.Aimbot_Mode) // 自瞄状态变化
    {
        Drv_Set_Object_Visualable(UI_MODIFY, UI_Aimbot_Mode);    // 更新自瞄模式
        Drv_Set_Object_Visualable(UI_MODIFY, UI_Aim_Point_Info); // 更新准心位置和颜色

    //     if (AIMBUFF_CONST_MODE == UiExData.Aimbot_Mode || AIMBUFF_VAR_MODE == UiExData.Aimbot_Mode)
    //     {
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Circle);        // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_1);        // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_2);        // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_3);        // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_4);        // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_5);        // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_Circle_1); // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_Circle_2); // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_Circle_3); // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_Circle_4); // 显示能量机关图形
    //         Drv_Set_Object_Visualable(UI_CREATE, UI_EnergyR_Line_Circle_5); // 显示能量机关图形
    //     }
    //     else
    //     {
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Circle);        // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_1);        // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_2);        // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_3);        // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_4);        // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_5);        // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_Circle_1); // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_Circle_2); // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_Circle_3); // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_Circle_4); // 删除能量机关图形
    //         Drv_Set_Object_Visualable(UI_DELDTE, UI_EnergyR_Line_Circle_5); // 删除能量机关图形
    //     }
    }
    last_aimbot_state = UiExData.Aimbot_Mode; // 更新自瞄状态
}
// 检查视觉错误状态变化并更新UI显示
void UI_If_Visual_Error_State_Change()
{
    static uint8_t last_visual_error_state = 0; // 视觉错误上次保存状态
    UI_Checkout_Object_Change(UiExData.Visual_Error_State, last_visual_error_state, Visual_Working_EOK, UI_Aimbot_Err_Info);
    last_visual_error_state = UiExData.Visual_Error_State; // 更新视觉错误状态
}
// // 检查视觉最佳距离状态变化并更新中心弹道落点圆圈颜色
// void UI_If_Aimbot_Best_Dis_Change()
// {
//      static uint8_t last_visual_target_found = 0; // 自瞄最佳距离上次保存状态
//     if (last_visual_target_found != UiExData.visual_target_found)
//     {
//         Drv_Set_Object_Visualable(UI_MODIFY, UI_Aim_Point_Info); // 更新准心颜色
//     }
//     last_visual_target_found = UiExData.visual_target_found; // 更新视觉识别标志
// }
// 检查视觉是否正常识别到敌人并更新准心颜色
//void UI_If_Visual_Target_Found_Change()
//{
//    static uint8_t last_visual_target_found = 0; // 自瞄最佳距离上次保存状态
//    if (last_visual_target_found != UiExData.visual_target_found)
//    {
//        Drv_Set_Object_Visualable(UI_MODIFY, UI_Aim_Point_Info); // 更新准心颜色
//    }
//    last_visual_target_found = UiExData.visual_target_found; // 更新视觉识别标志
//}
// 检查能量机关状态变化并更新UI显示
void UI_If_Energy_is_already_Change()
{
    // static uint8_t last_energy_is_already = 0; // 能量机关上次保存状态
    // if (last_energy_is_already != UiExData.Energy_is_already)
    // {
        // 这里由于在自瞄状态切换至能量机关状态的时候已经显示出来了，修改颜色即可
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Circle);        // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_1);        // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_2);        // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_3);        // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_4);        // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_5);        // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_Circle_1); // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_Circle_2); // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_Circle_3); // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_Circle_4); // 更新能量机关状态
        // Drv_Set_Object_Visualable(UI_MODIFY, UI_EnergyR_Line_Circle_5); // 更新能量机关状态
        Drv_Set_Object_Visualable(UI_MODIFY, UI_Aim_Point_Info);        // 更新准心位置和颜色
    // }
    // last_energy_is_already = UiExData.Energy_is_already; // 更新能量机关状态
}
// 检查菜单选项状态变化并更新UI显示
void UI_If_Option_Flag_Change()
{
    static uint8_t last_option_flag = 0; // 选项标志上次保存状态
    if (last_option_flag != UiExData.MenuOption)
    {
        if (UiExData.MenuOption == MainMenu) // 主菜单下需要将所有的元素删除掉
        {
            // 删除菜单选项卡
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line1);
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line2);
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line3);
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line4);
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line5);
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line6);
            Drv_Set_Object_Visualable(UI_DELDTE, MenuSystem_Info_Line7);
        }
        else // 在菜单状态改变且其他菜单下需要显示菜单选项卡
        {
            // 显示菜单选项卡
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line1);
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line2);
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line3);
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line4);
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line5);
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line6);
            Drv_Set_Object_Visualable(UI_CREATE, MenuSystem_Info_Line7);
        }
    }
    last_option_flag = UiExData.MenuOption; // 更新选项标志状态
}

static rt_uint8_t getIfBuildAttacked(void)
{
    if (IsRefDataValid(Index_game_robot_survivors, 20) != Ref_ERROR)
        hurt.build_if_attacked = 0;
    return hurt.build_if_attacked;
}

void UI_If_Allow_Heat_Shoot_Change()
{
    static uint8_t last_allow_heat_shoot = 0; // 英雄是否允许超热量射击上次保存状态
    if (last_allow_heat_shoot != UiExData.IfAllow_Heat_Shoot)
    {
        Drv_Set_Object_Visualable(UI_MODIFY, UI_Hero_Allow_Heat_Shoot); // 更新英雄是否允许超热量射击状态
    }
    last_allow_heat_shoot = UiExData.IfAllow_Heat_Shoot; // 更新英雄是否允许超热量射击状态
}

// todo 英雄专属UI绘制  能量机关和准心的绘制
//  // 英雄专属UI绘制
//  /**英雄专属基地引导灯绘制 */
//  void UI_Guide_Light_Circle(UI_Object *object)
//  {
//      int guide_light_x = (int)((float)UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_x * 1920.0f / 1440.0f); // TODO测试引导灯x轴坐标 1920/1440比例映射
//      int guide_light_y = UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_y;                                   // TODO引导灯y轴坐标 需进行映射
//      int guide_light_radius = UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_radius;                         // TODO引导灯半径 需进行映射
//      UI_Set_Circle(object, UiGreen, 4, Ui_layer_0, guide_light_x, guide_light_y, guide_light_radius);
//  }

// /**英雄大弹丸绘制 */
// void UI_Big_Bullet_Circle(UI_Object *object)
// {
//     int bullet_x = (int)((float)UiExData.Hero_UI_Data.Ui_Bullet_Circle_x * 1920.0f / 1440.0f); // TODO大弹丸x轴坐标 需进行映射
//     int bullet_y = UiExData.Hero_UI_Data.Ui_Bullet_Circle_y;                                   // TODO大弹丸y轴坐标 需进行映射
//     int bullet_radius = UiExData.Hero_UI_Data.Ui_Bullet_Circle_radius;                         // TODO大弹丸半径 需进行映射
//     UI_Set_Circle(object, UiTeam, 4, Ui_layer_0, bullet_x, bullet_y, bullet_radius);
// }
/********测试UI_If_Hero_Bullet_Change********** */
// /**
//  * @brief
//  * @param argc 参数个数
//  * @param argv 参数数组，argv[1]为目标位置
//  */
// static void test_bullet_change(int argc, char **argv)
// {
//     if (argc != 4)
//     {
//         rt_kprintf("Usage: test_bullet_change [x] [y] [r] \n");
//         rt_kprintf("Example: test_bullet_change 50 50 10\n");
//         return;
//     }

//     float x = atof(argv[1]);
//     float y = atof(argv[2]);
//     float r = atof(argv[3]);
//     UiExData.Hero_UI_Data.Ui_Bullet_Circle_x = x;
//     UiExData.Hero_UI_Data.Ui_Bullet_Circle_y = y;
//     UiExData.Hero_UI_Data.Ui_Bullet_Circle_radius = r;
//     UiExData.Hero_UI_Data.Ui_Bullet_Circle_freshtick = rt_tick_get();
//     rt_kprintf("change UI :x: %d y:%d r:%d\n", (int)x, (int)y, (int)r);
// }
// MSH_CMD_EXPORT_ALIAS(test_bullet_change, bullet, "test bullet change");

// void UI_If_Hero_Bullet_Change(void)
// {
//     static uint8_t hero_bullet_create_flag = 0; // 标志位为1时创jian
//     static uint8_t hero_bullet_tick = 0;
//     if (hero_bullet_create_flag) // 如果现在已经创建，则检查刷新时间
//     {
//         if (rt_tick_get() - UiExData.Hero_UI_Data.Ui_Bullet_Circle_freshtick > 10000) // 如果超时，则删除
//         {
//             hero_bullet_create_flag = 0;
//             Drv_Set_Object_Visualable(UI_DELDTE, UI_Big_Bullet_Circle);
//             return;
//         }
//         else // 负责刷新UI
//         {
//             Drv_Set_Object_Visualable(UI_MODIFY, UI_Big_Bullet_Circle);
//             return;
//         }
//     }

//     else // 如果没有创建，则检查更新时间，时间合法，则创建，否则不管
//     {
//         if (rt_tick_get() - UiExData.Hero_UI_Data.Ui_Bullet_Circle_freshtick > 10000) // 如果超时，则不管
//         {
//             return;
//         }
//         else
//         {
//             Drv_Set_Object_Visualable(UI_CREATE, UI_Big_Bullet_Circle); // 创建大弹丸识别的圆形
//             hero_bullet_create_flag = 1;
//         }
//     }
// }

// /**
//  * @brief
//  * @param argc 参数个数
//  * @param argv 参数数组，argv[1]为目标位置
//  */
// static void test_light_change(int argc, char **argv)
// {
//     if (argc != 4)
//     {
//         rt_kprintf("Usage: test_light_change [x] [y] [r] \n");
//         rt_kprintf("Example: test_light_change 50 50 10\n");
//         return;
//     }

//     float x = atof(argv[1]);
//     float y = atof(argv[2]);
//     float r = atof(argv[3]);
//     UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_x = x;
//     UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_y = y;
//     UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_radius = r;
//     UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_freshtick = rt_tick_get();
//     rt_kprintf("change light UI :x: %d y:%d r:%d\n", (int)x, (int)y, (int)r);
// }
// MSH_CMD_EXPORT_ALIAS(test_light_change, light, "test light change");

// void UI_If_Hero_Light_Change()
// {
//     static uint8_t hero_light_create_flag = 0; // 标志位为1时创建

//     if (hero_light_create_flag) // 如果现在已经创建，则检查时间，如果超时，则删除，否则，修改
//     {
//         if (rt_tick_get() - UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_freshtick > 10000) //
//         {
//             hero_light_create_flag = 0;
//             Drv_Set_Object_Visualable(UI_DELDTE, UI_Guide_Light_Circle);
//             return;
//         }
//         else
//         {
//             Drv_Set_Object_Visualable(UI_MODIFY, UI_Guide_Light_Circle);
//             return;
//         }
//     }
//     else
//     {
//         if (rt_tick_get() - UiExData.Hero_UI_Data.Ui_Guide_Light_Circle_freshtick > 10000) // 如果超时，则不管
//         {
//             return;
//         }
//         else
//         {
//             Drv_Set_Object_Visualable(UI_CREATE, UI_Guide_Light_Circle); // 创建基地引导灯
//             hero_light_create_flag = 1;
//             return;
//         }
//     }
// }

void Get_UI_Data(void)
{
    /******************************已用****************************************/
    UiExData.Gimbal_Motor_Offline = Get_Gimbal_Motor_Offline_State();
    UiExData.Chassis_Motor_Offline = (Read_Wheels_Offline_State() | ((Get_Yaw_Motor_Offline_State() & 0x01) << 4));
    UiExData.Rub_Started = Read_Rub_Started();
    UiExData.Gimbal_Online = (Get_Gim_FreshTick()) && (rt_tick_get() - Get_Gim_FreshTick() < 800);
    UiExData.Super_Capacity_Online = Read_Super_Capacity_Online();
    UiExData.Chassis_Reset_Warning = (rt_tick_get() < 3000);
    UiExData.Gimbal_Reset_Warning = Get_Gim_Reset_Status();
    UiExData.RefSystem_Offline = (!GetRefDataRefreshTick(Index_power_heat_data)) || (!GetRefDataRefreshTick(Index_game_robot_status)) ||
                                 (!IsRefDataValid(Index_power_heat_data, 1000)) || (!IsRefDataValid(Index_game_robot_status, 1000)); //! 修改使用新的取时间和判断函数

    // UI复位标志，云台发送变化时，需要复位
    if (UiExData.UI_Reset_Flag != Get_UI_Reset_Flg())
    {
        UI_Reset_All();                              // 复位UI函数
        UiExData.UI_Reset_Flag = Get_UI_Reset_Flg(); // 更新UI复位标志位
    }
    UiExData.gimErrState = getGimErrState();
    // 云台认为视觉是否正常
    if (UiExData.Visual_Error_State != Get_Visual_Working_Error_State())
    {
        if (Visual_Working_EOK != Get_Visual_Working_Error_State())
        {
            UiExData.Visual_Error_State = Get_Visual_Working_Error_State(); // 更新状态
            Drv_Set_Object_Visualable(UI_CREATE, UI_Aimbot_Err_Info);       // 显示视觉错误信息
        }
        else if (Visual_Working_EOK == Get_Visual_Working_Error_State())
        {
            UiExData.Visual_Error_State = Get_Visual_Working_Error_State(); // 更新状态
            Drv_Set_Object_Visualable(UI_DELDTE, UI_Aimbot_Err_Info);       // 显示视觉错误信息
        }
    }

    UiExData.Strike_Stuck = Get_Gimbal_Stuck_State(); // 刷新卡弹标志位
    UI_If_Stuck_Change();

    Get_RemainCapcity(&UiExData.RemainCapacity);        // 画出超级电容剩余电量使用
    UiExData.ScRate = getScRate();                      // 得到超级电容电流衰减比例
    UiExData.IfSuperCap_open = get_Super_Cap_if_open(); // 读取当前云台是否允许超电放电
  

    UiExData.GM_YAW_360F = Get_YAW_ANGLE_360f(); // 获取云台,yaw --绘制辅助瞄准线
    UiExData.GM_Pitch = Get_PITCH_ANGLE_360f();  // 云台电机,pitch--绘制辅助瞄准线

    UiExData.Motion_Mode = Get_Motion_Mode(); // 显示当前运动模式(跟随/不跟随/小陀螺/快陀螺/变速陀螺)

    UiExData.Aimbot_Mode = Get_Aimbot_Mode(); // 自瞄模式
    UI_If_Aimbot_Change();                    // 检查自瞄状态是否改变,并进行相应的检查和处理

    UiExData.Color_Myself = Get_Color_Myself(); // 当前自瞄的己方颜色

    getRefData(memberGet_parm(UiExData.powerbuff, ext_power_heat_data, chassis_power_buffer)); // 缓冲能量J

    // 更新菜单状态
    UiExData.MenuOption = Get_UI_Option_Flag();
    UI_If_Option_Flag_Change();

    // UiExData.Local_Chassis_Mode = Get_Local_Chassis_Type(); // 当前的本地底盘模式(0功率优先 1血量优先)
    UiExData.Robot_Hurted = hurt.robot_if_hurt;
    UI_If_Allow_Heat_Shoot_Change(); // 检查英雄是否允许超热量射击状态是否改变,并进行相应的检查和处理

    //!云台视觉未加,先不加
    // UI_If_Visual_Target_Found_Change();                           // 检查自瞄识别敌方状态是否改变,并进行相应的检查和处理
    // UiExData.Hero_UI_Data = get_hero_ui_data(); // 英雄视觉识别的基地引导灯和弹丸数据
    // UI_If_Hero_Bullet_Change();                 // 检查英雄弹丸识别数据是否改变,并进行相应的检查和处理
    // UI_If_Hero_Light_Change();                  // 检查英雄基地引导灯识别数据是否改变,并进行相应的检查和处理

/*******************************未用**************************************** */
    // UiExData.GM_IMU_Pitch = Get_Gimbal_IMU_Data(IMU_PITCH); // 云台imu pitch--显示云台Pitch轴角度
    // UiExData.Local_AmmoBooster_Mode = Get_Local_AmmoBooster_Type();
    // UiExData.Local_Robot_Level = Get_Local_Robot_Level();
    // UiExData.Client_Mode = Get_Client_Status();
    // UiExData.GimIfSuperCapOutput = Get_if_supercap_output();
    // UiExData.RefSystem_Forced_Offline = Get_Ref_Offline_Cmd();
    // // 无线充电是否开启
    // UiExData.IfWireless_Charge = Get_Wireless_Charge(); // 刷新无线充电状态
    // UI_If_Wireless_Charge_Change();                     // 检查无线充电状态是否改变,并进行相应的检查和处理
    // UiExData.Bullet_Speed = Get_Bullet_Speed(); // 获得弹速 未使用
    // UiExData.Ref_AmmoRemain = Ref_AmmoRemain();         // 剩余弹量 未使用
    // UiExData.Magazine_Status = Get_Magazine_Status(); // 显示弹仓状态
    // UiExData.Bulid_If_Hurt = getIfBuildAttacked();    // 当建筑受到攻击时警告
    // UiExData.HeatLimit_Status = Get_HeatLimit_Status(); // 画出热量限制开关状态
    // UiExData.visual_target_found = Get_Gim_Visual_Target_Found(); // 视觉是否正常识别到敌方

    // UiExData.Energy_is_already = Get_Energy_is_already(); // 能量机关视觉拟合状态
    // UI_If_Energy_is_already_Change();                     // 检查能量机关状态是否改变,并进行相应的检查和处理
    // UiExData.if_aimbot_best_dis = Get_if_aimbot_best_dis(); // 是否开启自瞄最佳距离
    // UI_If_Aimbot_Best_Dis_Change();                         // 检查自瞄最佳距离状态是否改变,并进行相应的检查和处理
    // UiExData.Ch_IMU_Data = get_chassis_imu_data(); // 得到底盘三轴欧拉角--绘制辅助瞄准线
    //getRefData(structGet_pram(UiExData.hurtInfo, ext_robot_hurt)); // 获取伤害状态,发送频率--当建筑受到攻击时警告
    // UiExData.Capacity_Charge_Cmd = Get_Charge_Cmd();
// #ifdef BSP_USING_CHASSIS_IMU
//     UiExData.Ch_IMU_Data = Get_Chassis_IMU_Elur(NULL); // 得到底盘三轴欧拉角--绘制辅助瞄准线
// #else
// #endif
    // UiExData.Strike_Mode = Get_StrikeMode(); // 更新子弹模式,有单发与三连发模式(这里已经不再使用该变量来作为单发和连发的判断依据)
}

// 用户初始化函数
rt_err_t UI_Init(void)
{
    rt_err_t res = RT_EOK;

    /* step1：查找串口设备 */
    serial_UI = rt_device_find(BSP_REF_USART_NAME);
    Attach_UI_init(); // 注册UI对象

    // while (REF_ROBO_ID == 0 || REF_CLIENT_ID == 0)
    // {
    // }

    // 初始化信号量
    res = rt_sem_init(&UI_sem, "Cctrl_sem", 0, RT_IPC_FLAG_FIFO);
    if (res != RT_EOK)
        return res;
    // 创建 UI 线程
    ui_thread = rt_thread_create("UI Send",
                                 ui_thread_entry,
                                 RT_NULL,
                                 THREAD_STACK_UI,
                                 THREAD_PRIO_UI,
                                 THREAD_TICK_UI);

    if (ui_thread != RT_NULL)
    {

        res = rt_thread_startup(ui_thread);
        if (res != RT_EOK)
            return res;
    }
    else
        return RT_ERROR;

    // 创建线程定时器
    rt_timer_init(&ui_SendTimer,
                  "UI_time_task",
                  UI_IRQHandler,
                  RT_NULL,
                  UI_SEND_INTERVAL_TIME,
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    // 启动定时器
    res = rt_timer_start(&ui_SendTimer);
    if (res != RT_EOK)
        return res;

    return RT_EOK;
}