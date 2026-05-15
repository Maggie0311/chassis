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

/************************步兵UI 已用*********************** */
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
/************************步兵UI 未用*********************** */
// // 基地宽度
// #define BASEMENT_HEIGHT 42.0f
// // 哨兵宽度
// #define SENTRY_HEIGHT 130.0f
// // 前哨站宽度
// #define OUTPOST_HEIGHT 120.0f

// #define AMMOR_HEIGHT 20.0f // 装甲板高

// #define h_max 540.0f      // UI最大的vision_y
// #define angle_max 35.574f // 图传最大的张
// // #define speed0  1500.0f                        //子弹初速度(cm/s)

// // 绘制弹道线
// #define UI_AUXAIM_BKGD_SIZE 40

// // 画出此时车身的姿态(云台与底盘的夹角)
// #define UI_ATTITUDE_ANGLE_SIZE

// //底盘IMU姿态
// #define UI_CHASSIS_IMU_LENGTH 70
// #define UI_CHASSIS_IMU_X 1600
// #define UI_CHASSIS_IMU_Y 700

// // 显示弹速
// #define UI_BULLET_SPEED_SIZE 12
// #define UI_BULLET_SPEED_CENTER_X 1650
// #define UI_BULLET_SPEED_CENTER_Y 800

// // 显示剩余弹量
// #define UI_REMAIN_AMMO_SIZE 12
// #define UI_REMAIN_AMMO_CENTER_X 1650
// #define UI_REMAIN_AMMO_CENTER_Y 750

// // 显示弹仓开关
// #define UI_MAGAZINE_STATUS_SIZE 11
// #define UI_MAGAZINE_STATUS_CENTER_X 1650
// #define UI_MAGAZINE_STATUS_CENTER_Y 510

// #define UI_MONITOR_SIZE 10
// #define UI_MONITOR_CENTER_X 150
// #define UI_MONITOR_CENTER_Y 860

// // H绘制敌方单位的信息
// #define UI_ENEMY_SIZE 20
// #define UI_ENEMY_RESERVING_TIME 4000 /// 标识显示时间 单位ms

// // 当受攻击时计算敌人位置信息用于下一步出处理
// #define UI_ROTATE_TIPS_SIZE 20
// #define UI_ROTATE_TIPS_CENTER_X 860
// #define UI_ROTATE_TIPS_CENTER_Y 800
// #define UI_ROTATE_TIPS_RESERVING_TIME 8000

// #define UI_ROTATE_TIPS_CIRCLE_RADIUS 10

// #define UI_WALL_SIZE 40

// // 当建筑受到攻击时警告
// #define UI_WARNING_SIZE 16
// #define UI_WARNING_CENTER_X 150
// #define UI_WARNING_CENTER_Y 650
// #define UI_WARNING_RESERVING_TIME 6000

// #define UI_HEATLIMIT_SIZE 11
// #define UI_HEATLIMIT_CENTER_X 1600
// #define UI_HEATLIMIT_CENTER_Y 600

// #define UI_VELOCITY_DYN_COLOR GRAPH_COLOR_CYAN
// #define UI_VELOCITY_BKGD_COLOR GRAPH_COLOR_GREEN
// #define UI_VELOCITY_SIZE 10
// #define UI_VELOCITY_CENTER_X 1400
// #define UI_VELOCITY_CENTER_Y 450

// #define UI_HEIGHT_DYN_COLOR GRAPH_COLOR_CYAN
// #define UI_HEIGHT_BKGD_COLOR GRAPH_COLOR_GREEN
// #define UI_HEIGHT_SIZE 11
// #define UI_HEIGHT_CENTER_X 1650
// #define UI_HEIGHT_CENTER_Y 450

// // 英雄相机位置辅助
// #if defined CORE_USING_2025_INFANTRY_1 // 25赛季步兵1直角
// // 相机位置辅助矩形框
// #define UI_CREAM_HELP_CENTER_X_1 360 // 384 // 起始（左下角）
// #define UI_CREAM_HELP_CENTER_Y_1 150 // 180
// #define UI_CREAM_HELP_CENTER_X_2 650 // 523 // 结束（右上角）
// #define UI_CREAM_HELP_CENTER_Y_2 360 // 288
// #endif

// #if defined CORE_USING_2025_INFANTRY_2 // 25赛季步兵2倒角
// // 相机位置辅助矩形框
// #define UI_CREAM_HELP_CENTER_X_1 360 // 384 // 起始
// #define UI_CREAM_HELP_CENTER_Y_1 150 // 180
// #define UI_CREAM_HELP_CENTER_X_2 650 // 523 // 结束
// #define UI_CREAM_HELP_CENTER_Y_2 360 // 288

/************************过洞对准UI 标定参数*********************** */
// 洞口轮廓矩形 -- TODO: 标定洞口在图传中的左上角和右下角
#define UI_HOLE_RECT_X1  0   // TODO: 标定
#define UI_HOLE_RECT_Y1  500   // TODO: 标定
#define UI_HOLE_RECT_X2  600  // TODO: 标定
#define UI_HOLE_RECT_Y2  780   // TODO: 标定
#define UI_HOLE_LINE_WIDTH 3
#define UI_HOLE_LAYER      Ui_layer_6

#endif