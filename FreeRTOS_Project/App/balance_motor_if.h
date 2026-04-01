#pragma once

#include "balance_types.h"
#include "dvc_motor_dm.h"

// 这里只适配你现在用的 Class_Motor_DM_Normal
using BalanceDmMotor = Class_Motor_DM_Normal;

// 单个槽位的绑定关系
struct BalanceMotorBinding
{
    BalanceDmMotor* motor;
    bool registered;
};

// 初始化适配层
void BalanceMotorIf_Init();

// 注册关节电机
bool BalanceMotorIf_RegisterJoint(uint8_t index, BalanceDmMotor* motor);

// 注册轮电机
bool BalanceMotorIf_RegisterWheel(uint8_t index, BalanceDmMotor* motor);

// 更新反馈：DM电机 -> robot.fdb
void BalanceMotorIf_UpdateFeedback(BalanceRobot* robot);

// 下发命令：robot.cmd -> DM电机
void BalanceMotorIf_SendCommand(const BalanceRobot* robot);

// 关闭所有电机输出（逻辑层）
void BalanceMotorIf_DisableAll();

// 可选：给所有已注册电机发送达妙失能帧
void BalanceMotorIf_SendExitAll();

// 可选：给所有已注册电机发送达妙使能帧
void BalanceMotorIf_SendEnterAll();

// 对所有已注册电机调用周期发送
void BalanceMotorIf_TxAllPeriodic(void);

// 对所有已注册电机调用 100ms 保活
void BalanceMotorIf_AliveAllPeriodic(void);