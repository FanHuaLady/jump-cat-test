#include "balance_controller.h"

#include <stddef.h>
#include <string.h>

#include "balance_config.h"
#include "balance_kinematics.h"

namespace
{
    // =========================
    // 腿长控制参数
    // =========================
    static constexpr float k_leg_len_kp = 60.0f;
    static constexpr float k_leg_len_kd = 4.0f;

    // =========================
    // 虚拟杆角度控制参数
    // 先保守一点，后面慢慢调
    // =========================
    static constexpr float k_leg_ang_kp = 35.0f;
    static constexpr float k_leg_ang_kd = 2.5f;

    // =========================
    // 当前第一版目标虚拟杆角度
    // 先固定为 0
    // 后面你可以改成：
    // phi0_ref = k_pitch * (pitch_ref - pitch_now)
    // =========================
    static constexpr float k_leg_ang_ref = -1.6580f;

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
    // 腿长方向控制：输出 rod_f
    // 这里只改 rod_f，不动 rod_tp
    // =========================
    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        const float l_ref  = robot->ref.target_leg_length[i];  // 目标腿长
        const float l_now  = robot->leg[i].rod.l0;             // 当前虚拟腿长
        const float dl_now = robot->leg[i].rod.dl0;            // 当前虚拟腿长变化率

        const float err_l = l_ref - l_now;

        // 腿长 PD
        float rod_f = k_leg_len_kp * err_l - k_leg_len_kd * dl_now;

        // 保守限幅
        rod_f = BalanceClamp(rod_f,
                             -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                              BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);

        robot->cmd[i].rod_f = rod_f;
    }
}

void BalanceController_LegAngle(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // =========================
    // 虚拟杆摆角控制：输出 rod_tp
    // 当前先固定目标角为 0
    // =========================
    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        const float phi0_now  = robot->leg[i].rod.phi0;    // 当前虚拟杆角度
        const float dphi0_now = robot->leg[i].rod.dphi0;   // 当前虚拟杆角速度

        const float phi0_ref = k_leg_ang_ref;
        const float err_phi0 = phi0_ref - phi0_now;

        // 虚拟杆角度 PD
        float rod_tp = k_leg_ang_kp * err_phi0 - k_leg_ang_kd * dphi0_now;

        // 保守限幅
        rod_tp = BalanceClamp(rod_tp,
                              -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                               BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);

        robot->cmd[i].rod_tp = rod_tp;
        robot->cmd[i].wheel_t = 0.0f;
    }
}

// 这个函数会利用 rod_f, rod_tp 和 VMC 得到真正的控制力矩
// 然后填充 joint_motor_cmd
// 会存在一个任务将 joint_motor_cmd 中的内容发给电机
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
