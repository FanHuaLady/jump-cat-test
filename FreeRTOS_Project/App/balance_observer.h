#pragma once

#include "balance_types.h"

// 初始化 observer
void BalanceObserver_Init(BalanceRobot* robot);

// 更新机体状态（由 IMU 得到）
void BalanceObserver_UpdateBody(BalanceRobot* robot);

// 更新腿部状态（由电机反馈得到）
void BalanceObserver_UpdateLeg(BalanceRobot* robot);

// 更新速度观测（先做简单版）
void BalanceObserver_UpdateVelocity(BalanceRobot* robot);

// 拼接 LQR 使用的状态量
void BalanceObserver_UpdateLqrState(BalanceRobot* robot);

// 一次性更新全部 observer
void BalanceObserver_UpdateAll(BalanceRobot* robot);
