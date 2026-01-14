#ifndef __FUN_VISUAL_COM_H__
#define __FUN_VISUAL_COM_H__
#include <board.h>
#include "pid.h"
#include "mod_motion.h"
#include "mod_RefSystem.h"
#include "sentry_cmd.h"
#include "visual_data.h"
/*视觉目前ROS坐标系为前方为x轴，左边为y轴 底盘坐标系为x轴为右方，y轴为前方*/

// 接收的视觉数据
#ifdef USART_DATA_TYPE_VEL

typedef __packed struct
{
    uint8_t head[4]; // = {0x00, 0x00, 0x80, 0x7f}; // 帧头
    //不算帧头的数据长度
    /******第1个字节，3个字节****** */
    uint8_t game_type : 4;              // 比赛类型
    uint8_t game_progress : 4;          // 当前比赛阶段
    uint16_t stage_remain_time;         // 当前阶段剩余时间

    /******第4个字节，32个字节************* */
    ext_game_robot_survivors_t game_robot_survivors; // 机器人血量数据

    /******第36个字节，4个字节************* */
    ext_event_data_t event_data; // 场地事件数据

    /******第40个字节，12个字节************* */
    uint8_t robot_id;               // 机器人id 可用于进行数据校验
    uint8_t robot_level;            // 机器人等级
    uint16_t remain_HP;             // 剩余血量
    uint16_t max_HP;                // 血量上限
    uint16_t shooter_cooling_rate;  // 机器人枪口每秒冷却值
    uint16_t shooter_cooling_limit; // 机器人枪口热量上限
    uint16_t chassis_power_limit;   // 机器人底盘功率上限

    /******第52个字节，12个字节************* */
    ext_game_robot_pos_t game_robot_pos; // 机器人位置数据

    /******第64个字节，6个字节************* */
    ext_bullet_remaining_t bullet_remaining; // 子弹剩余发射数

    /******第70个字节，32个字节************* */
    float hero_x;  
    float hero_y;  
    float engineer_x;  
    float engineer_y;  
    float standard_3_x;  
    float standard_3_y;  
    float standard_4_x;  
    float standard_4_y;  

    /******第102个字节，6个字节************* */
    ext_sentry_info_t sentry_info; // 哨兵自主决策数据

    uint8_t sum; // 校验和
} VisualComTx_t;        //共66字节

typedef __packed struct
{
    float x;                                           // x线速度
    float y;                                           // y线速度
    float w;                                           // 角速度
    uint8_t is_tyro : 1;                               // 小陀螺模式
    uint8_t revese : 7;                                // 保留位
    uint16_t virt_yaw;                                 // 跟随虚拟角度【0~360】单位：0.1度
    sentry_cmd_info_t sentry_cmd_data;                 // 哨兵自主决策指令
    uint8_t data_num;                                  // 识别车辆数据个数
    visual_data_t radar_data[VISUAL_DATA_NUM_MAX + 1]; // 识别车辆数据-雷达包-不定长(0号位为包数量，每包长度为9字节)
    uint8_t sum;                                       // 校验和
} VisualComRX_t;
#endif
typedef struct
{
    Ctrl_data_info_t ctrlData; // 转化后视觉的控制数据
} VisualCtrl_t;

typedef struct
{
    uint16_t errNum;
    uint8_t rx_f_ms;
    uint32_t rxTick;
    uint32_t txTick;
} VisualComInfo_t;

rt_err_t VisualComInit(void);

// 与视觉通信线程
rt_err_t VisualInit(void);

// 获取哨兵自主决策数据
sentry_cmd_info_t get_sentry_cmd_data(void);

#endif
