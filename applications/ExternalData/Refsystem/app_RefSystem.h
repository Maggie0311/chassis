#ifndef __APP_REFSYSTEM_H__
#define __APP_REFSYSTEM_H__
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "ThreadCtrl.h"//初始化线程要用
#include "mod_RefSystem.h"

/* 英雄弹速上限档位 */
#define HERO_BS_1 10
#define HERO_BS_2 16

/* 步兵弹速上限档位 */
#define INFANTRY_BS_1 15
#define INFANTRY_BS_2 18
#define INFANTRY_BS_3 30

typedef enum
{
    LOW_SPEED = 0,
    MID_SPEED,
    HIGH_SPEED,

} Ref_bullet_speed_mode_e;

// 红蓝方
typedef enum
{
    REF_ERROR,
    RED_TEAM = 1,
    BLUE_TEAM,

} Ref_team_color_e;

typedef enum
{
    REF_OUTPOST,  // 前哨站
    REF_SENTRY,   // 哨兵
    REF_BASEMENT, // 基地
    BUILDING_NUM
} Ref_Build_e;   
         
typedef enum
{
    event_self_supplyarea_no_overlap = 0,             //bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领
    event_self_supplyarea_overlap = 1,                //bit 1：己方与兑换区重叠的补给区占领状态，1为已占领
    event_self_supplyarea = 2,                        //bit 2：己方补给区的占领状态，1为已占领（仅 RMUL 适用） 
    event_self_small_energybuff = 3,                  //bit 3：己方小能量机关的激活状态，1为已激活
    event_self_large_energybuff = 4,                  //bit 4：己方大能量机关的激活状态，1为已激活
    event_self_center_highland = 5,                   //bit 5-6：己方中央高地的占领状态，1为被己方占领，2为被对方占领
    event_self_trapezium_highland = 7,                //bit 7-8：己方梯形高地的占领状态，1为已占领
    event_self_hurted_by_dart_lasttime = 9,           //bit 9-17：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
    event_self_hurted_by_dart_goal = 18,              //bit 18-20：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，1为击中前哨站，2为击中基地固定目标，3
    event_center_gain_points = 21,                    //bit 21-22：中心增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领。（仅RMUL适用）
    event_self_fort_gain_points=23,                   //bit 23-24：己方堡垒增益点的占领状态，0为未被占领，1为被己方占领，2为被对方占领，3为被双方占领。
} Ref_Event_e;  // 场地事件数据


typedef enum
{
    RFID_self_base = 0,                   // bit 0：己方基地增益点 
    RFID_self_center_highland = 1,        //  bit 1：己方中央高地增益点 
    RFID_enemy_center_highland = 2,       //  bit 2：对方中央高地增益点 
    RFID_self_trapezium_highland = 3,     //  bit 3：己方梯形高地增益点 
    RFID_enemy_trapezium_highland = 4,    //  bit 4：对方梯形高地增益点 
    RFID_self_ramp_front = 5,             //  bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前） 
    RFID_self_ramp_back = 6,              //  bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后） 
    RFID_enemy_ramp_front = 7,            //  bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前） 
    RFID_enemy_ramp_back = 8,             //  bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后） 
    RFID_self_center_highland_down = 9,   //  bit 9：己方地形跨越增益点（中央高地下方） 
    RFID_self_center_highland_up = 10,    //  bit 10：己方地形跨越增益点（中央高地上方） 
    RFID_enemy_center_highland_down = 11, //  bit 11：对方地形跨越增益点（中央高地下方） 
    RFID_enemy_center_highland_up = 12,   //  bit 12：对方地形跨越增益点（中央高地上方） 
    RFID_self_road_down = 13,             //  bit 13：己方地形跨越增益点（公路下方）
    RFID_self_road_up = 14,               // bit 14：己方地形跨越增益点（公路上方） 
    RFID_enemy_road_down = 15,            // bit 15：对方地形跨越增益点（公路下方） 
    RFID_enemy_road_up = 16,              // bit 16：对方地形跨越增益点（公路上方） 
    RFID_self_fort = 17,                  // bit 17：己方堡垒增益点 
    RFID_self_outpost = 18,               // bit 18：己方前哨站增益点 
    RFID_self_supplyarea_no_overlap = 19, // bit 19：己方与兑换区不重叠的补给区/RMUL补给区 
    RFID_self_supplyarea_overlap = 20,    // bit 20：己方与兑换区重叠的补给区 
    RFID_self_large_resource_island = 21, // bit 21：己方大资源岛增益点 
    RFID_enemy_large_resource_island = 22,// bit 22：对方大资源岛增益点 
    RFID_center_point = 23,                // bit 23：中心增益点（仅 RMUL 适用） 
    RFID_enemy_fortess_point=24,          //bit 24: 对方堡垒增益点
} Ref_RFID_e;  // RFID检测情况

typedef enum
{
    pos_x,      //x坐标
    pos_y       //y坐标
} Ref_Pos_e;  // 位置信息

typedef enum
{
    robot_hero,
    robot_engineer,
    robot_standard_3,
    robot_standard_4
} Ref_Robot_e;  // 机器人种类



typedef struct
{
    rt_uint8_t robot_if_hurt;     // 机器人是否受伤
    rt_uint8_t robot_hurt_armor;  // 机器人受伤的装甲板ID
    rt_uint8_t build_if_attacked; // 建筑物是否受伤，低三位代表哨兵，前哨站，基地。位数据为1：掉血，为0：不掉血
    rt_uint16_t sentry_hp;        // 哨兵血量
    rt_uint16_t outpost_hp;       // 前哨站血量
    rt_uint16_t base_hp;          // 基地血量
    uint32_t tick;

} Ref_Hurt_t;

typedef struct
{
    rt_uint16_t outpost_hp; // 敌方前哨站血量
} EnemyInfo_t;

/*------------------------------------XXX----------------------------------------------------------*/

#define RADAR_ROBOT_NUM_MAX 10
typedef enum
{
    radar_robotPos = 0x201, // 坐标
    radar_alarm = 0x202,    // 预警

} radarInfo_e;

typedef enum
{
    robot_ID1,
    robot_ID2,
    robot_ID3,
    robot_ID4,
    robot_ID5,
    robot_IDSentry,
    robot_IDAll,
} radarRobotID_e;

// 位置信息
typedef __packed struct
{
    uint16_t robotID;
    float x;
    float y;
    float z;
} m_radarData_t;

/*------------------------------------XXX----------------------------------------------------------*/

extern rt_uint16_t REF_ROBO_ID;   // 该机器人ID
extern rt_uint16_t REF_CLIENT_ID; // 该客户端ID
extern rt_uint16_t REF_RADAR_ID;  // 己方雷达站ID
extern rt_uint8_t REF_TEAM_COLOR; // 己方队颜色

/**
 * @brief    获取当前机器人ID和客户端ID
 * @param    None
 * @return   Ref_ROBO_CLIENT_ID_e
 */
void Ref_Robot_ID(void *data);

/**
 * @brief    更新机器人受伤状态
 * @param    None
 * @return   None
 */
void Ref_Robot_Set_Hurt(void *data);

/**
 * @brief    获取枪口上限速度
 * @param    None
 * @return   单位 m/s
 */
rt_uint16_t Ref_Bullet_Speed_Limit(void);

/**
 * @brief    获取剩余弹量
 *		  	发送频率： 10Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   剩余弹量
 */
rt_uint16_t Ref_AmmoRemain(void);
/**
 * @brief 获取当前机器人是否飞坡增益
 *        根据规则：同一台机器人需在 10 秒内检测到一方场地两处飞坡增益点的场地交互模块卡，才能触发飞坡增益
 *        飞坡增益持续时间：20秒
 * @return uint8_t
 */
uint8_t RefGetIsFlySlope(void);
//获取各个机器人的位置
float get_robot_pos(Ref_Pos_e x_y, Ref_Robot_e robot_kind);
/**
 * @brief    判断当前建筑(前哨站，哨兵，基地)是否掉血，并刷新掉血数据
 * @param    None
 * @return   None
 */
void Ref_Bulid_If_Hurt(void *data);

rt_err_t DJI_Init(void);
extern Ref_Hurt_t hurt;
extern EnemyInfo_t EnemyInfo;

#endif
