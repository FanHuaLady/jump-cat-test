#include "balance_controller.h"

#include <stddef.h>
#include <string.h>

#include "balance_config.h"
#include "balance_kinematics.h"

namespace
{
    // =========================
    // 第一版腿长控制参数
    // 先用很保守的 P / D
    // 后面再慢慢调
    // =========================
    static constexpr float k_leg_len_kp = 120.0f;
    static constexpr float k_leg_len_kd = 8.0f;

    static inline float BalanceClamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    static inline void BalanceClearMotorCmd(BalanceMotorCmd* cmd)
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

    static inline void BalanceClearLegCmd(BalanceLegCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->rod_f = 0.0f;
        cmd->rod_tp = 0.0f;
        cmd->joint_t[0] = 0.0f;
        cmd->joint_t[1] = 0.0f;
        cmd->wheel_t = 0.0f;
    }
}

void BalanceController_Init(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        BalanceClearLegCmd(&robot->cmd[i]);
    }

    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }
}

// 左右腿目标腿长相同
// 不动，不转
void BalanceController_SetRef(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // =========================
    // 第一版：全部写死
    // 先只做原地站立准备
    // =========================
    robot->ref.target_leg_length[0] = BALANCE_DEFAULT_LEG_LEN_STAND;
    robot->ref.target_leg_length[1] = BALANCE_DEFAULT_LEG_LEN_STAND;

    robot->ref.target_vx = 0.0f;
    robot->ref.target_wz = 0.0f;
    robot->ref.target_roll = 0.0f;
}

void BalanceController_LegLength(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // =========================
    // 第一版只做腿长方向力 rod_f
    // 不做摆角力矩 rod_tp
    // 不做轮子力矩 wheel_t
    // =========================
    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        const float l_ref = robot->ref.target_leg_length[i];
        const float l_now = robot->leg[i].rod.l0;
        const float dl_now = robot->leg[i].rod.dl0;

        const float err_l = l_ref - l_now;

        // 简单 PD
        // 腿短了，就给正的撑腿力
        // 腿长了，就给负的收腿力
        // 腿正在快速伸缩时，用 dl0 做一点阻尼
        
        // 这个rod_f是虚拟腿长方向力，是一个PID的输出值
        // 之后还需要VMC转换为实际的控制力矩
        float rod_f = k_leg_len_kp * err_l - k_leg_len_kd * dl_now;

        // 做一个保守限幅，避免一开始太猛
        rod_f = BalanceClamp(rod_f, -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT, BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);

        robot->cmd[i].rod_f = rod_f;
        robot->cmd[i].rod_tp = 0.0f;
        robot->cmd[i].wheel_t = 0.0f;
    }
}

// 这个函数会利用rod_f, rod_tp和VMC得到真正的控制力矩
// 然后填充joint_motor_cmd
// 会存在一个任务将joint_motor_cmd中的内容发给电机
void BalanceController_Output(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 先清空所有输出
    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }

    // =========================
    // 左腿
    // rod_f + rod_tp -> joint_t[2]
    // =========================
    {
        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(robot->leg[0].joint.phi1, robot->leg[0].joint.phi4, J);

        BalanceCalcVmc(robot->cmd[0].rod_f,
                       robot->cmd[0].rod_tp,
                       J,
                       robot->cmd[0].joint_t);

        robot->joint_motor_cmd[BAL_JOINT_L_0].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_0].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_0].tor =
            BalanceClamp(robot->cmd[0].joint_t[0],
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                         BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_L_0].kp = BALANCE_DEFAULT_JOINT_MIT_KP;
        robot->joint_motor_cmd[BAL_JOINT_L_0].kd = BALANCE_DEFAULT_JOINT_MIT_KD;
        robot->joint_motor_cmd[BAL_JOINT_L_0].enable = true;

        robot->joint_motor_cmd[BAL_JOINT_L_1].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_1].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_L_1].tor =
            BalanceClamp(robot->cmd[0].joint_t[1],
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                         BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_L_1].kp = BALANCE_DEFAULT_JOINT_MIT_KP;
        robot->joint_motor_cmd[BAL_JOINT_L_1].kd = BALANCE_DEFAULT_JOINT_MIT_KD;
        robot->joint_motor_cmd[BAL_JOINT_L_1].enable = true;
    }

    // =========================
    // 右腿
    // =========================
    {
        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(robot->leg[1].joint.phi1, robot->leg[1].joint.phi4, J);

        BalanceCalcVmc(robot->cmd[1].rod_f,
                       robot->cmd[1].rod_tp,
                       J,
                       robot->cmd[1].joint_t);

        robot->joint_motor_cmd[BAL_JOINT_R_0].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_0].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_0].tor =
            BalanceClamp(robot->cmd[1].joint_t[0],
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                         BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_R_0].kp = BALANCE_DEFAULT_JOINT_MIT_KP;
        robot->joint_motor_cmd[BAL_JOINT_R_0].kd = BALANCE_DEFAULT_JOINT_MIT_KD;
        robot->joint_motor_cmd[BAL_JOINT_R_0].enable = true;

        robot->joint_motor_cmd[BAL_JOINT_R_1].pos = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_1].vel = 0.0f;
        robot->joint_motor_cmd[BAL_JOINT_R_1].tor =
            BalanceClamp(robot->cmd[1].joint_t[1],
                         -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                         BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
        robot->joint_motor_cmd[BAL_JOINT_R_1].kp = BALANCE_DEFAULT_JOINT_MIT_KP;
        robot->joint_motor_cmd[BAL_JOINT_R_1].kd = BALANCE_DEFAULT_JOINT_MIT_KD;
        robot->joint_motor_cmd[BAL_JOINT_R_1].enable = true;
    }

    // =========================
    // 第一版：轮子不输出
    // =========================
    robot->wheel_motor_cmd[BAL_WHEEL_L].pos = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].vel = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].tor = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_L].kp = BALANCE_DEFAULT_WHEEL_MIT_KP;
    robot->wheel_motor_cmd[BAL_WHEEL_L].kd = BALANCE_DEFAULT_WHEEL_MIT_KD;
    robot->wheel_motor_cmd[BAL_WHEEL_L].enable = true;

    robot->wheel_motor_cmd[BAL_WHEEL_R].pos = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].vel = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].tor = 0.0f;
    robot->wheel_motor_cmd[BAL_WHEEL_R].kp = BALANCE_DEFAULT_WHEEL_MIT_KP;
    robot->wheel_motor_cmd[BAL_WHEEL_R].kd = BALANCE_DEFAULT_WHEEL_MIT_KD;
    robot->wheel_motor_cmd[BAL_WHEEL_R].enable = true;
}

void BalanceController_Stop(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        BalanceClearLegCmd(&robot->cmd[i]);
    }

    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        BalanceClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }
}
