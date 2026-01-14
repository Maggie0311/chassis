#ifndef _PROTECT_H_
#define _PROTECT_H_
#include <rtdef.h>

rt_err_t protct_ThreadInit(void);
typedef struct
{
    uint32_t id;
    uint32_t isOffline;
} motorOfflineAlarm_t;
uint8_t GetMotorOffline(void);


#endif
