#include "fun_getBulletSpeed_AVG.h"
#include "app_RefSystem.h"
#include "fun_protocolGim.h"

/**
 * @brief    获取弹速档位
 * @param    None
 * @return   Ref_bullet_speed_mode_e
 */
static rt_uint8_t Ref_Bullet_Speed_Mode(void)
{ // v1.6.1协议更新后无弹速上限数据
#ifdef CORE_USING_HERO
    switch (HERO_BS_2)
    {
    case HERO_BS_1:
        return LOW_SPEED;
    case HERO_BS_2:
        return MID_SPEED;
    default:
        return LOW_SPEED;
    }
#else
    switch (INFANTRY_BS_3)
    {
    case INFANTRY_BS_1:
        return LOW_SPEED;
    case INFANTRY_BS_2:
        return MID_SPEED;
    case INFANTRY_BS_3:
        return HIGH_SPEED;
    default:
        return LOW_SPEED;
    }
#endif
}

/***
 * @brief 	对裁判系统弹速和云台设定的弹速取平均
 * @param
 * @retval	滤波后弹速,单位：cm/s
 * @author
 *  ***/
rt_uint16_t Get_Bullet_Speed(void)
{
    static SqQueue bullet_speed_buff[3]; // 三组弹速值应有三个循环队列
    static rt_uint8_t bullet_speed_mode = 0;
    static rt_uint8_t is_init = 0;
    static rt_uint16_t bullet_speed_ref_last = 0; ////上一次有裁判系统弹速返回值?
#ifdef CORE_USING_HERO
    static rt_uint16_t bullet_speed_limit[3] = {1000, 1600, 1600};
#else
    static rt_uint16_t bullet_speed_limit[3] = {0, 0, 0};
#endif
    rt_uint16_t len;
    rt_uint16_t bullet_speed_filter;
    rt_uint16_t bullet_speed_ref = 0; // 裁判系统弹速返回值
    float temp;

    if (!is_init)
    {
        InitSqQueue(&bullet_speed_buff[0]);
        InitSqQueue(&bullet_speed_buff[1]);
        InitSqQueue(&bullet_speed_buff[2]);
        is_init = 1;
    }

    bullet_speed_mode = Ref_Bullet_Speed_Mode();
    getRefData(memberGet_parm(temp,ext_shoot_data,bullet_speed));// 裁判系统弹速返回值
    bullet_speed_ref = (rt_uint16_t)(temp * 100); //m/s -> cm/s
    if (bullet_speed_ref != 0)
    {

        if (bullet_speed_ref_last != bullet_speed_ref) // 裁判系统弹速更新
        {
            bullet_speed_ref_last = bullet_speed_ref;
            if (EnSqQueue(&bullet_speed_buff[bullet_speed_mode], bullet_speed_ref))
            {
                len = SqQueueLength(bullet_speed_buff[bullet_speed_mode]);
                bullet_speed_filter = SqQueueSum(bullet_speed_buff[bullet_speed_mode]) / len;
                if (bullet_speed_filter == 0)
                {
                    return bullet_speed_limit[bullet_speed_mode];
                }
                return bullet_speed_filter;
            }
            else
            {
                DeSqQueue(&bullet_speed_buff[bullet_speed_mode], NULL);
                EnSqQueue(&bullet_speed_buff[bullet_speed_mode], bullet_speed_ref);
                len = SqQueueLength(bullet_speed_buff[bullet_speed_mode]);
                return SqQueueSum(bullet_speed_buff[bullet_speed_mode]) / len;
            }
        }
        else // 速度未更新
        {
            len = SqQueueLength(bullet_speed_buff[bullet_speed_mode]);
            bullet_speed_filter = SqQueueSum(bullet_speed_buff[bullet_speed_mode]) / len;
            if (bullet_speed_filter == 0)
            {
                return bullet_speed_limit[bullet_speed_mode];
            }
            return bullet_speed_filter;
        }
    }
    else
    {
        return bullet_speed_limit[bullet_speed_mode];
    }
}



/* 初始化一个空队列Q */
Status InitSqQueue(SqQueue *Q)
{
    Q->front = 0;
    Q->rear = 0;
    return OK;
}

/* 将Q清为空队列 */
Status ClearSqQueue(SqQueue *Q)
{
    Q->front = Q->rear = 0;
    return OK;
}

/* 若队列Q为空队列,则返回TRUE,否则返回FALSE */
Status SqQueueEmpty(SqQueue Q)
{
    if (Q.front == Q.rear) /* 队列空的标志 */
        return TRUE;
    else
        return FALSE;
}

/* 返回Q的元素个数，也就是队列的当前长度 */
int SqQueueLength(SqQueue Q)
{
    return (Q.rear - Q.front + SQQUEUE_MAXSIZE) % SQQUEUE_MAXSIZE;
}

/* 若队列不空,则用e返回Q的队头元素,并返回OK,否则返回ERROR */
Status GetSqHead(SqQueue Q, SqElemType *e)
{
    if (Q.front == Q.rear) /* 队列空 */
        return ERROR;
    *e = Q.data[Q.front];
    return OK;
}

/* 若队列未满，则插入元素e为Q新的队尾元素 */
Status EnSqQueue(SqQueue *Q, SqElemType e)
{
    if ((Q->rear + 1) % SQQUEUE_MAXSIZE == Q->front) /* 队列满的判断 */
        return ERROR;
    Q->data[Q->rear] = e;                      /* 将元素e赋值给队尾 */
    Q->rear = (Q->rear + 1) % SQQUEUE_MAXSIZE; /* rear指针向后移一位置， */
                                               /* 若到最后则转到数组头部 */
    return OK;
}

/* 若队列不空，则删除Q中队头元素，用e返回其值 */
Status DeSqQueue(SqQueue *Q, SqElemType *e)
{
    if (Q->front == Q->rear) /* 队列空的判断 */
        return ERROR;
    *e = Q->data[Q->front];                      /* 将队头元素赋值给e */
    Q->front = (Q->front + 1) % SQQUEUE_MAXSIZE; /* front指针向后移一位置， */
                                                 /* 若到最后则转到数组头部 */
    return OK;
}

/* 从队头到队尾依次对队列Q中每个元素输出 */
int SqQueueSum(SqQueue Q)
{
    int i;
    int sum = 0;
    i = Q.front;
    while (i != Q.rear)
    {
        sum += Q.data[i];
        i = (i + 1) % SQQUEUE_MAXSIZE;
    }
    return sum;
}
