#pragma once

#include "balance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// 全局机器人对象
extern BalanceRobot g_balance_robot;

// 全局启动入口
void BalanceApp_Init(void);
void BalanceApp_Task_Create(void);

// 可选：外部手动使能/停机
void BalanceApp_Enable(void);
void BalanceApp_Disable(void);

#ifdef __cplusplus
}
#endif
