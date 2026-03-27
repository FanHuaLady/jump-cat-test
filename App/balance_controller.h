#pragma once

#include "balance_types.h"

// 初始化控制器
void BalanceController_Init(BalanceRobot* robot);

// 设置参考值（第一版先写死）
void BalanceController_SetRef(BalanceRobot* robot);

// 腿长控制（第一版只做腿长P/PD）
void BalanceController_LegLength(BalanceRobot* robot);

// 角度控制
void BalanceController_LegAngle(BalanceRobot* robot);

// 输出分配（rod_f + rod_tp -> joint_t -> motor_cmd）
void BalanceController_Output(BalanceRobot* robot);

// 急停/清零
void BalanceController_Stop(BalanceRobot* robot);
