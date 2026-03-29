#include "balance_ref_pose.h"

#include <stddef.h>

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

    static inline bool BalanceRefPose_IsJointSelected(BalanceRefPoseMode mode, int joint_idx)
    {
        switch (mode)
        {
        case BALANCE_REF_POSE_MODE_LEFT_ONLY:
            return (joint_idx == BAL_JOINT_L_0) || (joint_idx == BAL_JOINT_L_1);

        case BALANCE_REF_POSE_MODE_RIGHT_ONLY:
            return (joint_idx == BAL_JOINT_R_0) || (joint_idx == BAL_JOINT_R_1);

        case BALANCE_REF_POSE_MODE_BOTH:
        default:
            return (joint_idx == BAL_JOINT_L_0) ||
                   (joint_idx == BAL_JOINT_L_1) ||
                   (joint_idx == BAL_JOINT_R_0) ||
                   (joint_idx == BAL_JOINT_R_1);
        }
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
    state->mode = BALANCE_REF_POSE_MODE_BOTH;

    for (int i = 0; i < 4; ++i)
    {
        state->target_cont[i] = 0.0f;
    }
}

void BalanceRefPose_Start(BalanceRefPoseState* state)
{
    BalanceRefPose_StartWithMode(state, BALANCE_REF_POSE_MODE_BOTH);
}

void BalanceRefPose_StartWithMode(BalanceRefPoseState* state, BalanceRefPoseMode mode)
{
    if (state == nullptr)
    {
        return;
    }

    state->active = true;
    state->finished = false;
    state->stable_count = 0;
    state->mode = mode;

    state->target_cont[0] = BALANCE_JOINT_L0_CONT_REF;
    state->target_cont[1] = BALANCE_JOINT_L1_CONT_REF;
    state->target_cont[2] = BALANCE_JOINT_R0_CONT_REF;
    state->target_cont[3] = BALANCE_JOINT_R1_CONT_REF;
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

    const int joint_idx[4] =
    {
        BAL_JOINT_L_0,
        BAL_JOINT_L_1,
        BAL_JOINT_R_0,
        BAL_JOINT_R_1
    };

    bool pos_ok = true;
    bool vel_ok = true;
    int selected_joint_count = 0;

    for (int i = 0; i < 4; ++i)
    {
        const int j = joint_idx[i];
        const bool selected = BalanceRefPose_IsJointSelected(state->mode, j);

        if (!selected)
        {
            ClearMotorCmd(&robot->joint_motor_cmd[j]);
            continue;
        }

        selected_joint_count++;

        const float q = robot->joint_angle_unwrap[j].continuous;
        const float dq = robot->joint_motor_fdb[j].vel;
        const float e = state->target_cont[i] - q;

        SetJointReturnCmdPd(&robot->joint_motor_cmd[j], e, dq);

        if (AbsFloat(e) >= kRefPosePosEps)
        {
            pos_ok = false;
        }

        if (AbsFloat(dq) >= kRefPoseVelEps)
        {
            vel_ok = false;
        }
    }

    // 其余关节关闭
    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if ((i == BAL_JOINT_L_0) ||
            (i == BAL_JOINT_L_1) ||
            (i == BAL_JOINT_R_0) ||
            (i == BAL_JOINT_R_1))
        {
            continue;
        }

        ClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    // 轮子关闭
    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        ClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }

    if ((selected_joint_count > 0) && pos_ok && vel_ok)
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