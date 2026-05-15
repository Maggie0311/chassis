#ifndef __CUSTOMUI_GRAPHPARAMETERS_H
#define __CUSTOMUI_GRAPHPARAMETERS_H

#include "board.h"

#ifndef PI
#define PI 3.14159265f
#endif
#ifndef angleTorad
#define angleTorad (PI / 180.0f) // 角度转弧度
#endif
#ifndef radToAngle
#define radToAngle (180.0f / PI) // 弧度转角度
#endif

// 超级电容
#define UI_SC_BAR_SIZE 15 // 图形大小
#define UI_SC_BAR_CENTER_X 650
#define UI_SC_BAR_CENTER_Y 100
#define Draw_SC_Bar_WIDTH 2 // 为了方便之后统一将宽度设为2 字体大小与线宽比例为 10:1

// 屏幕中心
#define X_OFFSET 960
#define Y_OFFSET 540

#define SCOPE_CIRCLE_RADIUS 180       // 中心瞄准圆半径大小，所有界面图形的绘制都与它成正比
#define OUTER_SCOPE_CIRCLE_RADIUS 340 // 外圈

// 子弹大小
#define UI_STRIKE_MODE_SIZE 8

// 子弹中心位置
#define UI_STRIKE_MODE_CENTER_X 1350
#define UI_STRIKE_MODE_CENTER_Y 100

// 显示当前运动模式(跟随/不跟随/陀螺)
#define UI_MOTION_MODE_SIZE 30
#define UI_MOTION_MODE_TRANS_SIZE 30
#define UI_MOTION_MODE_CENTER_X 900
#define UI_MOTION_MODE_CENTER_Y 1000
#define UI_MOTION_MODE_BAR_SIZE 12
#define UI_MOTION_MODE_BAR_X_START 1473
#define UI_MOTION_MODE_BAR_X_END 1488
#define UI_MOTION_MODE_BAR_Y 480

//自瞄模式相关
#define UI_AIMBOT_PRIO_SIZE 11
#define UI_AIMBOT_PRIO_CENTER_X 1600
#define UI_AIMBOT_PRIO_CENTER_Y 540
// 自瞄颜色相关
#define UI_AIMBOT_COLOR_SIZE 10
#define UI_AIMBOT_MODE_SIZE 10
#define UI_AIMBOT_COLOR_CENTER_X 1600
#define UI_AIMBOT_COLOR_CENTER_Y 520

// 能量机关
#define UI_ENERGY_LINE_LENGTH 75
#define UI_ENERGY_CENTER_X 200
#define UI_ENERGY_CENTER_Y 600

// 选项卡菜单位置参数
#define UI_MENU_START_X 1610    //菜单开始的位置x
#define UI_MENU_START_Y 860    //菜单开始的位置y
#define UI_MENU_FONT_SIZE 11    //菜单字体大小
#define UI_MENU_LINE_SPACE (UI_MENU_FONT_SIZE * 2.0f) //菜单行间距

#define UI_STRIKE_MODE_CENTER_X 1350 // 子弹中心位置
#define UI_STRIKE_MODE_CENTER_Y 100  // 子弹中心位置

// 显示Pitch轴角度
#define UI_PITCH_SIZE 15
#define UI_ABS_PITCH_CENTER_X 1200
#define UI_ABS_PITCH_CENTER_Y 450

//履带开关
#define UI_TRACK_SIZE 15 // 图形大小
#define UI_TRACK_STATE_CENTER_X 700
#define UI_TRACK_STATE_CENTER_Y 100
#define Draw_TRACK_STATE_WIDTH 2 

//狗洞
#define UI_HOLE_RECT_X1  0   // TODO: 标定
#define UI_HOLE_RECT_Y1  500   // TODO: 标定
#define UI_HOLE_RECT_X2  600  // TODO: 标定
#define UI_HOLE_RECT_Y2  780   // TODO: 标定
#define UI_HOLE_LINE_WIDTH 3
#define UI_HOLE_LAYER  Ui_layer_6

#endif