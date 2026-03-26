#include "balance_observer.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "balance_config.h"
#include "balance_kinematics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

namespace
{
    // 参考姿态标定后的关节角映射：
    // phi = sign * (continuous - cont_ref) + phi_ref
    static inline float BalanceApplyJointMount(float continuous,
                                               float sign,
                                               float cont_ref,
                                               float phi_ref)
    {
        return sign * (continuous - cont_ref) + phi_ref;
    }

    static inline float BalanceWrapPi(float angle)
    {
        while (angle > M_PI)  angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    static inline float BalanceClamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    static inline void BalanceClearLeg(BalanceLegState* leg)
    {
        if (leg == nullptr)
        {
            return;
        }

        memset(leg, 0, sizeof(BalanceLegState));
    }

    static inline void BalanceAngleUnwrapReset(BalanceAngleUnwrap* unwrap)
    {
        if (unwrap == nullptr)
        {
            return;
        }

        unwrap->initialized = false;
        unwrap->raw_last = 0.0f;
        unwrap->continuous = 0.0f;
    }

    static inline float BalanceAngleUnwrapUpdate(BalanceAngleUnwrap* unwrap, float raw_now)
    {
        if (unwrap == nullptr)
        {
            return raw_now;
        }

        // 达妙当前位置反馈范围近似 [-12.5, 12.5]
        constexpr float kHalfRange = 12.5f;
        constexpr float kFullRange = 25.0f;

        if (!unwrap->initialized)
        {
            unwrap->initialized = true;
            unwrap->raw_last = raw_now;
            unwrap->continuous = raw_now;
            return unwrap->continuous;
        }

        float delta = raw_now - unwrap->raw_last;

        if (delta > kHalfRange)
        {
            delta -= kFullRange;
        }
        else if (delta < -kHalfRange)
        {
            delta += kFullRange;
        }

        unwrap->continuous += delta;
        unwrap->raw_last = raw_now;

        return unwrap->continuous;
    }
}

void BalanceObserver_Init(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    robot->dt = BALANCE_CTRL_DT;
    robot->enable = false;
    robot->safe = true;

    robot->body.x = 0.0f;
    robot->body.x_dot = 0.0f;
    robot->body.x_acc = 0.0f;
    robot->body.x_dot_obv = 0.0f;
    robot->body.x_acc_obv = 0.0f;

    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        BalanceAngleUnwrapReset(&robot->joint_angle_unwrap[i]);
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        BalanceClearLeg(&robot->leg[i]);
        memset(&robot->leg_state[i], 0, sizeof(BalanceLegLqrState));
        robot->ref.target_leg_length[i] = BALANCE_DEFAULT_LEG_LEN_STAND;
    }

    robot->ref.target_vx = 0.0f;
    robot->ref.target_wz = 0.0f;
    robot->ref.target_roll = 0.0f;
}

// 从 IMU 读取机身姿态和角速度，构造 body 状态
void BalanceObserver_UpdateBody(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 直接从统一 IMU 结构取数据
    robot->body.roll = robot->imu.roll;
    robot->body.pitch = robot->imu.pitch;
    robot->body.yaw = robot->imu.yaw;

    robot->body.roll_dot = robot->imu.roll_dot;
    robot->body.pitch_dot = robot->imu.pitch_dot;
    robot->body.yaw_dot = robot->imu.yaw_dot;

    // 平衡主平面暂用 pitch
    robot->body.phi = BalanceWrapPi(robot->body.pitch);
    robot->body.phi_dot = robot->body.pitch_dot;

    // 线加速度简单取 IMU x 方向
    robot->body.x_acc = robot->imu.ax;
}

void BalanceObserver_UpdateLeg(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // ----- 左腿 -----
    {
        BalanceLegState& leg = robot->leg[0];

        const float joint0_raw = robot->joint_motor_fdb[BAL_JOINT_L_0].pos;
        const float joint1_raw = robot->joint_motor_fdb[BAL_JOINT_L_1].pos;

        const float joint0_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_L_0], joint0_raw);
        const float joint1_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_L_1], joint1_raw);

        // 参考姿态映射
        leg.joint.phi1 = BalanceApplyJointMount(joint0_cont,
                                                BALANCE_JOINT_L0_SIGN,
                                                BALANCE_JOINT_L0_CONT_REF,
                                                BALANCE_JOINT_L0_PHI_REF);
        leg.joint.phi4 = BalanceApplyJointMount(joint1_cont,
                                                BALANCE_JOINT_L1_SIGN,
                                                BALANCE_JOINT_L1_CONT_REF,
                                                BALANCE_JOINT_L1_PHI_REF);

        // 速度、力矩也统一到同一个关节坐标系
        leg.joint.dphi1 = BALANCE_JOINT_L0_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_0].vel;
        leg.joint.dphi4 = BALANCE_JOINT_L1_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_1].vel;
        leg.joint.t1    = BALANCE_JOINT_L0_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_0].tor;
        leg.joint.t2    = BALANCE_JOINT_L1_SIGN * robot->joint_motor_fdb[BAL_JOINT_L_1].tor;

        leg.wheel_vel = robot->wheel_motor_fdb[BAL_WHEEL_L].vel;

        float l0_phi0[2] = {0.0f, 0.0f};
        BalanceCalcL0Phi0(leg.joint.phi1, leg.joint.phi4, l0_phi0);

        leg.rod.l0 = l0_phi0[0];
        leg.rod.phi0 = l0_phi0[1];
        leg.rod.theta = M_PI_2 - leg.rod.phi0 - robot->body.phi;

        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(leg.joint.phi1, leg.joint.phi4, J);

        float d_l0_d_phi0[2] = {0.0f, 0.0f};
        BalanceCalcdL0dPhi0(J, leg.joint.dphi1, leg.joint.dphi4, d_l0_d_phi0);

        leg.rod.dl0 = d_l0_d_phi0[0];
        leg.rod.dphi0 = d_l0_d_phi0[1];
        leg.rod.dtheta = -leg.rod.dphi0 - robot->body.phi_dot;

        // 第一版先固定不做离地判定
        leg.is_take_off = false;

        // 腿长限制
        leg.rod.l0 = BalanceClamp(leg.rod.l0,
                                  BALANCE_DEFAULT_LEG_LEN_MIN,
                                  BALANCE_DEFAULT_LEG_LEN_MAX);
    }

    // ----- 右腿 -----
    {
        BalanceLegState& leg = robot->leg[1];

        const float joint0_raw = robot->joint_motor_fdb[BAL_JOINT_R_0].pos;
        const float joint1_raw = robot->joint_motor_fdb[BAL_JOINT_R_1].pos;

        const float joint0_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_R_0], joint0_raw);
        const float joint1_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_R_1], joint1_raw);

        // 参考姿态映射
        leg.joint.phi1 = BalanceApplyJointMount(joint0_cont,
                                                BALANCE_JOINT_R0_SIGN,
                                                BALANCE_JOINT_R0_CONT_REF,
                                                BALANCE_JOINT_R0_PHI_REF);
        leg.joint.phi4 = BalanceApplyJointMount(joint1_cont,
                                                BALANCE_JOINT_R1_SIGN,
                                                BALANCE_JOINT_R1_CONT_REF,
                                                BALANCE_JOINT_R1_PHI_REF);

        // 速度、力矩也统一到同一个关节坐标系
        leg.joint.dphi1 = BALANCE_JOINT_R0_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_0].vel;
        leg.joint.dphi4 = BALANCE_JOINT_R1_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_1].vel;
        leg.joint.t1    = BALANCE_JOINT_R0_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_0].tor;
        leg.joint.t2    = BALANCE_JOINT_R1_SIGN * robot->joint_motor_fdb[BAL_JOINT_R_1].tor;

        leg.wheel_vel = robot->wheel_motor_fdb[BAL_WHEEL_R].vel;

        float l0_phi0[2] = {0.0f, 0.0f};
        BalanceCalcL0Phi0(leg.joint.phi1, leg.joint.phi4, l0_phi0);

        leg.rod.l0 = l0_phi0[0];
        leg.rod.phi0 = l0_phi0[1];
        leg.rod.theta = M_PI_2 - leg.rod.phi0 - robot->body.phi;

        float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
        BalanceCalcJacobian(leg.joint.phi1, leg.joint.phi4, J);

        float d_l0_d_phi0[2] = {0.0f, 0.0f};
        BalanceCalcdL0dPhi0(J, leg.joint.dphi1, leg.joint.dphi4, d_l0_d_phi0);

        leg.rod.dl0 = d_l0_d_phi0[0];
        leg.rod.dphi0 = d_l0_d_phi0[1];
        leg.rod.dtheta = -leg.rod.dphi0 - robot->body.phi_dot;

        // 第一版先固定不做离地判定
        leg.is_take_off = false;

        // 腿长限制
        leg.rod.l0 = BalanceClamp(leg.rod.l0,
                                  BALANCE_DEFAULT_LEG_LEN_MIN,
                                  BALANCE_DEFAULT_LEG_LEN_MAX);
    }
}

// 简化版速度观测：
// 左右轮角速度平均 * 轮半径 = 前向线速度
void BalanceObserver_UpdateVelocity(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    const float speed =
        BALANCE_DEFAULT_WHEEL_RADIUS *
        (robot->leg[0].wheel_vel + robot->leg[1].wheel_vel) * 0.5f;

    robot->body.x_dot_obv = speed;
    robot->body.x_acc_obv = robot->body.x_acc;
    robot->body.x_dot = robot->body.x_dot_obv;

    // 第一版直接积分出 x
    robot->body.x += robot->body.x_dot_obv * robot->dt;
}

void BalanceObserver_UpdateLqrState(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        robot->leg_state[i].theta     = robot->leg[i].rod.theta;
        robot->leg_state[i].theta_dot = robot->leg[i].rod.dtheta;
        robot->leg_state[i].x         = robot->body.x;
        robot->leg_state[i].x_dot     = robot->body.x_dot_obv;
        robot->leg_state[i].phi       = robot->body.phi;
        robot->leg_state[i].phi_dot   = robot->body.phi_dot;
    }
}

void BalanceObserver_UpdateAll(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    BalanceObserver_UpdateBody(robot);
    BalanceObserver_UpdateLeg(robot);
    BalanceObserver_UpdateVelocity(robot);
    BalanceObserver_UpdateLqrState(robot);
}