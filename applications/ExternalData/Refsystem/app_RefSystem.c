#include "app_RefSystem.h"

/*线程要用*/
#include "fun_protocolGim.h"
#include "app_monitor.h"
#include "func_MonHandling.h"

rt_uint16_t REF_ROBO_ID;   // 该机器人ID
rt_uint16_t REF_CLIENT_ID; // 该客户端ID
rt_uint16_t REF_RADAR_ID;  // 己方雷达站ID
rt_uint8_t REF_TEAM_COLOR;

Ref_Hurt_t hurt =
{
    .robot_if_hurt = 0,
    .robot_hurt_armor = 0,
    .build_if_attacked = 0,
    .sentry_hp = 0,
    .outpost_hp = 0,
    .base_hp = 0,
    .tick = 0,
};
EnemyInfo_t EnemyInfo;

/*回调函数*/
/**
 * @brief    更新当前机器人ID和客户端ID
 * @param    数据地址
 * @return   Ref_ROBO_CLIENT_ID_e
 */
void Ref_Robot_ID(void *data)
{
    ext_game_robot_status_t *temp = (ext_game_robot_status_t *)data;
    switch (temp->robot_id)
    {
    case REF_ROBO_R1_HERO:
        REF_ROBO_ID = REF_ROBO_R1_HERO;
        REF_CLIENT_ID = REF_CLIENT_R1_HERO;
        break;
    case REF_ROBO_R2_ENGINEER:
        REF_ROBO_ID = REF_ROBO_R2_ENGINEER;
        REF_CLIENT_ID = REF_CLIENT_R2_ENGINEER;
        break;
    case REF_ROBO_R3_STANDARD:
        REF_ROBO_ID = REF_ROBO_R3_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_R3_STANDARD;
        break;
    case REF_ROBO_R4_STANDARD:
        REF_ROBO_ID = REF_ROBO_R4_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_R4_STANDARD;
        break;
    case REF_ROBO_R5_STANDARD:
        REF_ROBO_ID = REF_ROBO_R5_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_R5_STANDARD;
        break;
    case REF_ROBO_R6_AERIAL:
        REF_ROBO_ID = REF_ROBO_R6_AERIAL;
        REF_CLIENT_ID = REF_CLIENT_R6_AERIAL;
        break;
    case REF_ROBO_R7_SENTRY:
        REF_ROBO_ID = REF_ROBO_R7_SENTRY;
        break;
    case REF_ROBO_B1_HERO:
        REF_ROBO_ID = REF_ROBO_B1_HERO;
        REF_CLIENT_ID = REF_CLIENT_B1_HERO;
        break;
    case REF_ROBO_B2_ENGINEER:
        REF_ROBO_ID = REF_ROBO_B2_ENGINEER;
        REF_CLIENT_ID = REF_CLIENT_B2_ENGINEER;
        break;
    case REF_ROBO_B3_STANDARD:
        REF_ROBO_ID = REF_ROBO_B3_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_B3_STANDARD;
        break;
    case REF_ROBO_B4_STANDARD:
        REF_ROBO_ID = REF_ROBO_B4_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_B4_STANDARD;
        break;
    case REF_ROBO_B5_STANDARD:
        REF_ROBO_ID = REF_ROBO_B5_STANDARD;
        REF_CLIENT_ID = REF_CLIENT_B5_STANDARD;
        break;
    case REF_ROBO_B6_AERIAL:
        REF_ROBO_ID = REF_ROBO_B6_AERIAL;
        REF_CLIENT_ID = REF_CLIENT_B6_AERIAL;
        break;
    case REF_ROBO_B7_SENTRY:
        REF_ROBO_ID = REF_ROBO_B7_SENTRY;
        break;
    }

    switch (temp->robot_id)
    {
    case REF_ROBO_R1_HERO:
    case REF_ROBO_R2_ENGINEER:
    case REF_ROBO_R3_STANDARD:
    case REF_ROBO_R4_STANDARD:
    case REF_ROBO_R5_STANDARD:
    case REF_ROBO_R6_AERIAL:
    case REF_ROBO_R7_SENTRY:
    case REF_ROBO_R8_DART:
    case REF_ROBO_R9_RADAR:
        REF_RADAR_ID = REF_ROBO_R9_RADAR;
        REF_TEAM_COLOR = RED_TEAM;
    case REF_ROBO_B1_HERO:
    case REF_ROBO_B2_ENGINEER:
    case REF_ROBO_B3_STANDARD:
    case REF_ROBO_B4_STANDARD:
    case REF_ROBO_B5_STANDARD:
    case REF_ROBO_B6_AERIAL:
    case REF_ROBO_B7_SENTRY:
    case REF_ROBO_B8_DART:
    case REF_ROBO_B9_RADAR:
        REF_RADAR_ID = REF_ROBO_B9_RADAR;
        REF_TEAM_COLOR = BLUE_TEAM;
    default:
        REF_TEAM_COLOR = REF_ERROR;
    }
}

/**
 * @brief    更新机器人受击状态
 * @param    None
 * @return   None
 */
void Ref_Robot_Set_Hurt(void *data)
{
    uint8_t temp;
    hurt.robot_if_hurt = 1;

    getRefData(bitGet_parm(temp,ext_robot_hurt,ext_robot_hurt_status,4,4));//获取受伤类型
    if (temp == 0x0)//受弹丸攻击掉血
        getRefData(bitGet_parm(hurt.robot_hurt_armor,ext_robot_hurt,ext_robot_hurt_status,0,4));//更新受击装甲板id
}

/**
 * @brief    判断当前建筑(前哨站，哨兵，基地)是否掉血，并刷新掉血数据
 * @param    None
 * @return   None
 */
void Ref_Bulid_If_Hurt(void *data)
{
    ext_game_robot_survivors_t *robot_survivors = (ext_game_robot_survivors_t *)data;
    if (robot_survivors->ally_7_robot_HP < hurt.sentry_hp)
    {
        hurt.build_if_attacked |= 1 << 0;
        hurt.sentry_hp = robot_survivors->ally_7_robot_HP;
    }
    if (robot_survivors->ally_outpost_HP < hurt.outpost_hp)
    {
        hurt.build_if_attacked |= 1 << 1;
        hurt.outpost_hp = robot_survivors->ally_outpost_HP;
    }
    if (robot_survivors->ally_base_HP < hurt.base_hp)
    {
        hurt.build_if_attacked |= 1 << 2;
        hurt.base_hp = robot_survivors->ally_base_HP;
    }

    EnemyInfo.outpost_hp = robot_survivors->ally_outpost_HP;
}

/*裁判系统数据加工*/

/**
 * @brief    获取剩余弹量
 *		  	发送频率： 10Hz 周期发送， 所有机器人发送
 * @param    None
 * @return   剩余弹量
 */
uint16_t Ref_AmmoRemain(void)
{
    uint16_t temp;
#ifdef CORE_USING_HERO
    getRefData(memberGet_parm(temp,ext_bullet_remaining,bullet_remaining_num_42mm));
#else
    getRefData(memberGet_parm(temp,ext_bullet_remaining,bullet_remaining_num_17mm));
#endif
	return temp;
}

/**
 * @brief 获取当前机器人是否飞坡增益
 *        根据规则：同一台机器人需在 10 秒内检测到一方场地两处飞坡增益点的场地交互模块卡，才能触发飞坡增益
 *        飞坡增益持续时间：20秒
 * @return uint8_t
 */
uint8_t RefGetIsFlySlope(void)
{
    uint8_t ret_flag = 0;
    static rt_tick_t flyslope_tick = 0; // 最近一次获取到飞坡增益的时刻
    static rt_tick_t tick1 = 0, tick2 = 0, tick3 = 0, tick4 = 0, ticktmp = 0;
    uint32_t temp;

    ticktmp = GetRefDataRefreshTick(Index_rfid_status);
    if (IsRefDataValid(Index_rfid_status,(10 * 1000))) // 对于判断飞坡，10s内此数据有效
    {
        getRefData(memberGet_parm(temp,ext_rfid_status,rfid_status));
        if ((uint8_t)(temp >> 5 & 0x01))
        { // 己方飞坡增益区飞坡前
            tick1 = ticktmp;
        }
        else if ((uint8_t)(temp >> 6 & 0x01))
        { // 己方飞坡增益区飞坡后
            tick2 = ticktmp;
        }
        else if ((uint8_t)(temp >> 7 & 0x01))
        { // 对方飞坡增益区飞坡前
            tick3 = ticktmp;
        }
        else if ((uint8_t)(temp >> 8 & 0x01))
        { // 对方飞坡增益区飞坡后
            tick4 = ticktmp;
        }
    }
    if (tick1 && tick2) // 确保数据非零有效
    {
        if (tick2 - tick1 <= 10 * 1000)
        { // 己方飞坡增益区触发
            ret_flag = 1;
            flyslope_tick = tick2;
        }
    }
    if (tick3 && tick4) // 确保数据非零有效
    {
        if (tick4 - tick3 <= 10 * 1000)
        { // 对方飞坡增益区触发
            ret_flag = 1;
            flyslope_tick = tick4;
        }
    }
    if (flyslope_tick && rt_tick_get() - flyslope_tick < 20 * 1000) // 20秒增益时间
        ret_flag = 1;
    else // 20秒过后增益消失
        ret_flag = 0;
    return ret_flag;
}

/********************************** RFID ***********************************************/

float get_robot_pos(Ref_Pos_e x_y, Ref_Robot_e robot_kind)
{
    float temp;
    switch (robot_kind)
    {
        case robot_hero:
            if (x_y == pos_x)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,hero_x));
            else if (x_y == pos_y)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,hero_y));
            break;
        case robot_engineer:
            if (x_y == pos_x)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,engineer_x));
            else if (x_y == pos_y)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,engineer_y));
            break;
        case robot_standard_3:
            if (x_y == pos_x)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,standard_3_x));
            else if (x_y == pos_y)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,standard_3_y));
            break;
        case robot_standard_4:
            if (x_y == pos_x)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,standard_4_x));
            else if (x_y == pos_y)
                getRefData(memberGet_parm(temp,ext_ground_robot_position,standard_4_y));
            break;
        default:
            temp = 0;
    }
    return temp;
}

rt_err_t Ref_Init(void)
{
    rt_err_t res;
    
    /*裁判系统线程初始化*/
    res = DJI_Init();

    /*注册裁判系统数据更新后的回调函数*/
    Attach_Ref_rec_callback(Index_game_robot_survivors,Ref_Bulid_If_Hurt);
    Attach_Ref_rec_callback(Index_game_robot_state,Ref_Robot_ID);
    Attach_Ref_rec_callback(Index_robot_hurt,Ref_Robot_Set_Hurt);
    
    return res;
}
