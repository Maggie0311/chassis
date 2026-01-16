#ifndef __RLS_H
#define __RLS_H

#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f4xx_hal.h"




#define SAFE_MIN 1e-12f // 防止除零的最小值
#define N 2       // 参数向量维度（根据模型调整）
#define LAMBDA 0.99999f // 遗忘因子


typedef struct 
{
    float theta[3];   // 参数估计向量
    float P[N][N];    // 协方差矩阵
    float lambda;     // 动态遗忘因子
    float last_error;
	float error;
	float now_tick;
	int8_t flag;
    float  Phi[3];
    float out_power;
    float now_tor;
    float now_spe[4];
    float last_spe[4];
    float repe_speed;//当匀速运动时防止RLS过拟合
    uint16_t repe_flag;
    float now_power;
    float SetSpeed[4];
}RLS_Estimator;


void RLS_Init(RLS_Estimator *est, float InitTheta[], float delta) ;
float Compute_Condition_Number(RLS_Estimator *est);
void RLS_Update(RLS_Estimator *est, float phi[N], float y_measured);
int8_t Check_Stability(RLS_Estimator *est) ;
void Reconstruct_P(RLS_Estimator *est) ;
extern float InitTheta[3];
#endif