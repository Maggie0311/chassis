#ifndef __FUN_GETBULLETSPEED_AGV_H_
#define __FUN_GETBULLETSPEED_AGV_H_
#include <board.h>

typedef int Status;
typedef int SqElemType;   /* QElemType类型根据实际情况而定，这里假设为int */
#define SQQUEUE_MAXSIZE 6 /* 存储空间初始分配量 */

#ifndef OK
#define OK 1
#endif
#ifndef ERROR
#define ERROR 0
#endif
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/* 循环队列的顺序存储结构 */
typedef struct
{
    SqElemType data[SQQUEUE_MAXSIZE];
    int front; /* 头指针 */
    int rear;  /* 尾指针，若队列不空，指向队列尾元素的下一个位置 */
} SqQueue;

typedef int LqElemType; /* QElemType类型根据实际情况而定，这里假设为int */

typedef struct QNode /* 结点结构 */
{
    void (*func)(void *para);
    LqElemType data;
    struct QNode *next;
} QNode, *QueuePtr;

typedef struct /* 队列的链表结构 */
{
    QueuePtr front, rear; /* 队头、队尾指针 */
} LinkQueue;

Status InitSqQueue(SqQueue *Q);
Status ClearSqQueue(SqQueue *Q);
Status SqQueueEmpty(SqQueue Q);
int SqQueueLength(SqQueue Q);
Status GetSqHead(SqQueue Q, SqElemType *e);
Status EnSqQueue(SqQueue *Q, SqElemType e);
Status DeSqQueue(SqQueue *Q, SqElemType *e);
Status SqQueueSum(SqQueue Q);

/***
 * @brief 	对裁判系统弹速和云台设定的弹速取平均
 * @param
 * @retval	滤波后弹速,单位：cm/s
 * @author
 *  ***/
rt_uint16_t Get_Bullet_Speed(void);
#endif
