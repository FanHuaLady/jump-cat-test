#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// 创建单腿腿长调试任务
void BalanceLegDebug_Task_Create(void);

// 使能/停机
void BalanceLegDebug_Enable(void);
void BalanceLegDebug_Disable(void);

// 修改目标腿长（单位：m）
void BalanceLegDebug_SetTargetLength(float target_len);

// 可选：修改控制参数
void BalanceLegDebug_SetGains(float kp, float kd);

#ifdef __cplusplus
}
#endif