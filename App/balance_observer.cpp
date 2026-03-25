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
    static inline float BalanceApplyJointMount(float continuous, float sign, float offset)
    {
        return sign * continuous + offset;
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

// 这里是补全LQR的一部分 QQQ!!!
// 从IMU 读取机身姿态和角速度，构造 body 状态
void BalanceObserver_UpdateBody(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }
    
    // 这里的陀螺仪数据来源是balance_imu_if.cpp
    // 还有一个陀螺仪任务，所以可以在这里直接用
    
    // 直接从统一IMU结构取数据
    robot->body.roll = robot->imu.roll;                         // 身体的 roll 就等于IMU的 roll
    robot->body.pitch = robot->imu.pitch;                       // 身体的 pitch 就等于IMU的 pitch
    robot->body.yaw = robot->imu.yaw;                           // 身体的 yaw 就等于IMU的 yaw

    robot->body.roll_dot = robot->imu.roll_dot;                 // 身体的 roll_dot 就等于IMU的 roll_dot
    robot->body.pitch_dot = robot->imu.pitch_dot;               // 身体的 pitch_dot 就等于IMU的 pitch_dot
    robot->body.yaw_dot = robot->imu.yaw_dot;                   // 身体的 yaw_dot 就等于IMU的 yaw_dot

    // 按原平衡步兵语义：
    // phi = -pitch
    // phi_dot = -pitch_dot
    // 为什么取负，是原模型的坐标定义问题【思考】
    // 经过这个函数
    // LQR算法就补上了phi和pitch
    robot->body.phi = -robot->body.pitch;                       // LQR 最终使用的俯仰角变量叫 phi
    robot->body.phi_dot = -robot->body.pitch_dot;               // LQR 最终使用的俯仰角速度变量叫 pitch_dot

    // 先简单把 body x_acc 直接取 IMU 前向加速度
    // 后面如果需要，可以补去重力和坐标变换
    robot->body.x_acc = robot->imu.ax;
}

// 这里是补全LQR的一部分 QQQ!!!
// 它把两个关节角变成虚拟腿状态
void BalanceObserver_UpdateLeg(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // =========================
    // 第一版约定：
    // 左腿关节: 0,1
    // 右腿关节: 2,3
    // 左轮: 0
    // 右轮: 1
    //
    // 这里先不做：
    // - 零位偏置
    // - 方向翻转
    // - 特殊安装修正
    // 后面再加
    // =========================

    // ----- 左腿 -----
    {
        BalanceLegState& leg = robot->leg[0];

        const float joint0_raw = robot->joint_motor_fdb[BAL_JOINT_L_0].pos;
        const float joint1_raw = robot->joint_motor_fdb[BAL_JOINT_L_1].pos;

        const float joint0_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_L_0], joint0_raw);
        const float joint1_cont =
            BalanceAngleUnwrapUpdate(&robot->joint_angle_unwrap[BAL_JOINT_L_1], joint1_raw);

        // 第一版：先不加零位偏置和方向修正
        leg.joint.phi1  = BalanceApplyJointMount(joint0_cont,
                                                 BALANCE_JOINT_L0_SIGN,
                                                 BALANCE_JOINT_L0_OFFSET);
        leg.joint.phi4  = BalanceApplyJointMount(joint1_cont,
                                                 BALANCE_JOINT_L1_SIGN,
                                                 BALANCE_JOINT_L1_OFFSET);

        leg.joint.dphi1 = robot->joint_motor_fdb[BAL_JOINT_L_0].vel;
        leg.joint.dphi4 = robot->joint_motor_fdb[BAL_JOINT_L_1].vel;
        leg.joint.t1    = robot->joint_motor_fdb[BAL_JOINT_L_0].tor;
        leg.joint.t2    = robot->joint_motor_fdb[BAL_JOINT_L_1].tor;

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

        leg.is_take_off = false;
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

        // 第一版：先不加零位偏置和方向修正
        leg.joint.phi1  = BalanceApplyJointMount(joint0_cont,
                                                 BALANCE_JOINT_R0_SIGN,
                                                 BALANCE_JOINT_R0_OFFSET);
        leg.joint.phi4  = BalanceApplyJointMount(joint1_cont,
                                                 BALANCE_JOINT_R1_SIGN,
                                                 BALANCE_JOINT_R1_OFFSET);

        leg.joint.dphi1 = robot->joint_motor_fdb[BAL_JOINT_R_0].vel;
        leg.joint.dphi4 = robot->joint_motor_fdb[BAL_JOINT_R_1].vel;
        leg.joint.t1    = robot->joint_motor_fdb[BAL_JOINT_R_0].tor;
        leg.joint.t2    = robot->joint_motor_fdb[BAL_JOINT_R_1].tor;

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

        leg.is_take_off = false;
    }
}

// 这里是补全LQR的一部分 QQQ!!!
void BalanceObserver_UpdateVelocity(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    // 简化版速度观测：
    // 左右轮角速度平均 * 轮半径 = 前向线速度
    const float speed =
        BALANCE_DEFAULT_WHEEL_RADIUS *
        (robot->leg[0].wheel_vel + robot->leg[1].wheel_vel) * 0.5f;

    robot->body.x_dot_obv = speed;
    robot->body.x_acc_obv = robot->body.x_acc;
    robot->body.x_dot = robot->body.x_dot_obv;                      // 补齐LQR

    // 第一版直接积分出 x
    robot->body.x += robot->body.x_dot_obv * robot->dt;             // 补齐LQR
}

// LQR最终的整合
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
