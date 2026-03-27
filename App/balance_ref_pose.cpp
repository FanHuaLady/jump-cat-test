#include "balance_ref_pose.h"

#include <stddef.h>
#include <math.h>

#include "balance_motor_if.h"
#include "balance_config.h"

namespace
{
    // 关节空间简单 PD 回位参数
    static constexpr float kRefPoseJointKp = 12.0f;
    static constexpr float kRefPoseJointKd = 0.8f;
    static constexpr float kRefPoseJointTorLimit = 2.0f;

    // 到位判定参数
    static constexpr float kRefPosePosEps = 0.03f;
    static constexpr float kRefPoseVelEps = 0.10f;
    static constexpr uint16_t kRefPoseStableNeed = 20;

    static inline float AbsFloat(float x)
    {
        return (x >= 0.0f) ? x : -x;
    }

    static inline float ClampFloat(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    static inline void ClearMotorCmd(BalanceMotorCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->pos = 0.0f;
        cmd->vel = 0.0f;
        cmd->tor = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->enable = false;
    }

    static void SetJointReturnCmdPd(BalanceMotorCmd* cmd,
                                    float err,
                                    float vel)
    {
        if (cmd == nullptr)
        {
            return;
        }

        const float tau =
            ClampFloat(kRefPoseJointKp * err - kRefPoseJointKd * vel,
                       -kRefPoseJointTorLimit,
                        kRefPoseJointTorLimit);

        // 纯扭矩 PD 回位
        cmd->pos = 0.0f;
        cmd->vel = 0.0f;
        cmd->tor = tau;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->enable = true;
    }
}

void BalanceRefPose_Init(BalanceRefPoseState* state)
{
    if (state == nullptr)
    {
        return;
    }

    state->active = false;
    state->finished = false;
    state->stable_count = 0;
    state->target_cont[0] = 0.0f;
    state->target_cont[1] = 0.0f;
}

void BalanceRefPose_Start(BalanceRefPoseState* state)
{
    if (state == nullptr)
    {
        return;
    }

    state->active = true;
    state->finished = false;
    state->stable_count = 0;

    state->target_cont[0] = BALANCE_JOINT_L0_CONT_REF;
    state->target_cont[1] = BALANCE_JOINT_L1_CONT_REF;
}

void BalanceRefPose_Stop(BalanceRefPoseState* state)
{
    if (state == nullptr)
    {
        return;
    }

    state->active = false;
}

bool BalanceRefPose_IsActive(const BalanceRefPoseState* state)
{
    return (state != nullptr) ? state->active : false;
}

bool BalanceRefPose_IsFinished(const BalanceRefPoseState* state)
{
    return (state != nullptr) ? state->finished : false;
}

void BalanceRefPose_Update(BalanceRefPoseState* state, BalanceRobot* robot)
{
    if ((state == nullptr) || (robot == nullptr) || (!state->active))
    {
        return;
    }

    // 当前连续角和速度
    const float q0 = robot->joint_angle_unwrap[BAL_JOINT_L_0].continuous;
    const float q1 = robot->joint_angle_unwrap[BAL_JOINT_L_1].continuous;

    const float dq0 = robot->joint_motor_fdb[BAL_JOINT_L_0].vel;
    const float dq1 = robot->joint_motor_fdb[BAL_JOINT_L_1].vel;

    const float e0 = state->target_cont[0] - q0;
    const float e1 = state->target_cont[1] - q1;

    // 直接写关节 PD 扭矩回位命令
    SetJointReturnCmdPd(&robot->joint_motor_cmd[BAL_JOINT_L_0], e0, dq0);
    SetJointReturnCmdPd(&robot->joint_motor_cmd[BAL_JOINT_L_1], e1, dq1);

    // 其他关节和轮子先关闭
    for (int i = 2; i < BALANCE_JOINT_NUM; ++i)
    {
        ClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        ClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }

    // 到位判定
    const bool pos_ok =
        (AbsFloat(e0) < kRefPosePosEps) &&
        (AbsFloat(e1) < kRefPosePosEps);

    const bool vel_ok =
        (AbsFloat(dq0) < kRefPoseVelEps) &&
        (AbsFloat(dq1) < kRefPoseVelEps);

    if (pos_ok && vel_ok)
    {
        if (state->stable_count < 0xFFFFU)
        {
            state->stable_count++;
        }
    }
    else
    {
        state->stable_count = 0;
    }

    if (state->stable_count >= kRefPoseStableNeed)
    {
        state->finished = true;
        state->active = false;
    }
}