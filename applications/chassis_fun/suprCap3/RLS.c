#include  "RLS.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f4xx_hal.h"



void RLS_Init(RLS_Estimator *est, float InitTheta[], float delta) 
{
    est->lambda = LAMBDA;
    for(int i=0; i<N; i++)
     {
        for(int j=0; j<N; j++)
         {
            est->P[i][j] = (i == j) ? delta : 0.0f; // P(0) = δI
         }
    }
    memcpy(est->theta, InitTheta, (N+1)*sizeof(InitTheta));
		 
}

float K[N];              // 增益向量
float y_pred = 0.0f;      // 模型预测值
float error, denom = 0.0f;


// 递归更新核心（嵌入实时系统调用）
void RLS_Update(RLS_Estimator *est, float phi[N], float y_measured)
{
    y_pred = 0.0f;
    denom = 0.0f;
    // 1. 计算预测值和误差
    for(int i=0; i<N; i++) 
        y_pred += est->theta[i] * phi[i];

    error = y_measured - y_pred;
  
    // 2. 计算增益向量K = P*phi / (lambda + phi'*P*phi)
    for(int i=0; i<N; i++) 
    {
        K[i] = 0.0f;
        for(int j=0; j<N; j++) 
            K[i] += est->P[i][j] * phi[j];
        denom += K[i] * phi[i];
    }
    denom = est->lambda + denom;

    // 3. 更新参数估计：theta = theta + K*error
    for(int i=0; i<N; i++) 
		{
            float  LastTheta=est->theta[i];
            est->theta[i] += K[i] * error / denom;
            // est->theta[i]=UTILS_HP_FAST(est->theta[i],LastTheta,0.2);
			if(est->theta[i]<1e-20f)est->theta[i]=1e-20f;
			//else if(est->theta[i]>1500)est->theta[i]=1500;
		
		}
    // 4. 更新协方差矩阵：P = (P - K*K'/denom) / lambda
    for(int i=0; i<N; i++) 
    {
        for(int j=0; j<N; j++) 
        {
            est->P[i][j] = (est->P[i][j] - K[i]*K[j]/denom);
			est->P[i][j]/= est->lambda;
        }
    }
     //自适应遗忘因子调整（可选）
   float error_change = fabs(error - est->last_error);
   if(error_change > 0.2f * fabs(error)&&error_change) 
   { // 系统突变检测
       est->lambda = 0.96f; // 加速跟踪
   } 
//    else if(error_change >= 0.8f * fabs(error)&&error_change <3*fabs(error))
//    {    
//         est->lambda = 0.9999f; // 跟踪
//    }
   else 
   {
       est->lambda = LAMBDA; // 恢复默认
   }
    est->last_error = error;
}

// 计算条件数（稳定性指标）
float Compute_Condition_Number(RLS_Estimator *est)
{
    float max_d = est->P[0][0];
    float min_d = est->P[0][0];
    
    for(int i = 1; i < N; i++) 
      for(int j=1;j<N;j++)
    {
        if(est->P[i][j] > max_d) max_d = est->P[i][j];
       
    }
    
    return max_d ;
}

// 诊断函数：检查数值稳定性
int8_t Check_Stability(RLS_Estimator *est) 
{
    // 检查D的对角元素是否为正
    for(int i = 0; i < N; i++) 
    {
        if(est->P[i][i] <= SAFE_MIN) 
            return 0; // 不稳定
    }   
    // 检查条件数
    if(Compute_Condition_Number(est) > 1e7f) 
        return 0; // 条件数过大
       
    return 1; // 稳定
}
void Reconstruct_P(RLS_Estimator *est) 
{
 

      if(Compute_Condition_Number(est) > 1e8f) 
      { 
        // 严重膨胀：部分重置
        float reset_ratio = 0.1f; // 只重置10%
        for(int i = 0; i < N; i++) 
        {
            for(int j = 0; j < N; j++) 
            {
                est->P[i][j] *= (1.0f - reset_ratio);
                if(i == j) 
                    est->P[i][j] += reset_ratio * 1.0f; // 添加小单位矩阵
                
            }
        }
    } else if(Compute_Condition_Number(est) > 1e6f) 
    {
        // 轻微膨胀：仅衰减
        for(int i = 0; i < N; i++) 
        {
            for(int j = 0; j < N; j++) 
                est->P[i][j] *= 0.95f;
        }
    }
    

    
}