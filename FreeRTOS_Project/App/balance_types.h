#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "balance_config.h"

// 单个电机反馈
typedef struct BalanceMotorFdb
{
    float pos;                                  // rad
    float vel;                                  // rad/s
    float tor;                                  // N*m
    bool online;                                // 是否在线
} BalanceMotorFdb;

// 单个电机命令
typedef struct BalanceMotorCmd
{
    float pos;                                  // rad
    float vel;                                  // rad/s
    float tor;                                  // N*m
    float kp;                                   // MIT kp
    float kd;                                   // MIT kd
    bool enable;                                // 本周期是否使能发送
} BalanceMotorCmd;

// IMU统一数据
typedef struct BalanceImuData
{
    float roll;
    float pitch;
    float yaw;

    float roll_dot;
    float pitch_dot;
    float yaw_dot;

    float ax;
    float ay;
    float az;

    bool online;
} BalanceImuData;

// 关节空间状态
typedef struct BalanceJointState
{
    float phi1;                                 // 第一个主动关节角转的度数
    float phi4;                                 // 第二个主动关节角转的度数
    float dphi1;                                // 第一个主动关节角速度，单位 rad/s
    float dphi4;                                // 第二个主动关节角速度，单位 rad/s
    float t1;                                   // 表示第一个主动关节当前力矩，单位 N·m
    float t2;                                   // 表示第二个主动关节当前力矩，单位 N·m
} BalanceJointState;

// 虚拟腿状态
typedef struct BalanceRodState
{
    float l0;                                   // 虚拟腿长度
    float phi0;                                 // 虚拟腿的绝对角 / 几何角
    float theta;                                // 平衡模型使用的腿摆角

    float dl0;                                  // 虚拟腿长度变化率
    float dphi0;                                // 虚拟腿角速度
    float dtheta;                               // 腿摆角速度
} BalanceRodState;

// 单腿完整状态
// 关节、虚拟腿、轮子放到一起
typedef struct BalanceLegState
{
    BalanceJointState joint;                    // 关节空间状态，保留原始驱动变量
    BalanceRodState rod;                        // 虚拟腿

    float wheel_vel;                            // 这一侧轮子的角速度
    bool is_take_off;                           // 这一条腿是否离地 / 腾空
} BalanceLegState;

// 机体状态
typedef struct BalanceBodyState
{
    float roll;
    float pitch;
    float yaw;

    float roll_dot;
    float pitch_dot;
    float yaw_dot;                              // IMU 姿态角速度

    float phi;                                  // ?
    float phi_dot;                              // 平衡模型的机体俯仰角速度

    float x;                                    // 车体前向位置
    float x_dot;
    float x_acc;

    float x_dot_obv;
    float x_acc_obv;
} BalanceBodyState;

// LQR状态向量
// 极其重要！！！！！
typedef struct BalanceLegLqrState
{
    float theta;                                // 轮腿系统里那根等效摆杆的角度
    float theta_dot;                            // 组合得到摆杆角速度
    float x;                                    // 机器人沿前进方向累计走了多远
    float x_dot;                                // 目前是 左右轮平均线速度               
    float phi;                                  // 机体俯仰角
    float phi_dot;                              // 机体俯仰角速度
} BalanceLegLqrState;

// 参考值
typedef struct BalanceRefState
{
    float target_leg_length[2];
    float target_vx;
    float target_wz;
    float target_roll;
} BalanceRefState;

// 单腿控制输出
typedef struct BalanceLegCmd
{
    float rod_f;                                                    // 
    float rod_tp;
    float joint_t[2];
    float wheel_t;
} BalanceLegCmd;

typedef struct BalanceAngleUnwrap
{
    bool initialized;
    float raw_last;
    float continuous;
} BalanceAngleUnwrap;

// 整机对象
typedef struct BalanceRobot
{
    BalanceMotorFdb joint_motor_fdb[BALANCE_JOINT_NUM];             // 关节电机的接收数组
    BalanceMotorFdb wheel_motor_fdb[BALANCE_WHEEL_NUM];             // 

    BalanceMotorCmd joint_motor_cmd[BALANCE_JOINT_NUM];             //  关节电机的命令数组
    BalanceMotorCmd wheel_motor_cmd[BALANCE_WHEEL_NUM];             // 
    
    BalanceAngleUnwrap joint_angle_unwrap[BALANCE_JOINT_NUM];
    
    BalanceImuData imu;                                             // 陀螺仪的数据
    BalanceBodyState body;                                          // 表示整个机体
    BalanceLegState leg[BALANCE_LEG_NUM];                           // 里面装了左腿和右腿
    BalanceLegLqrState leg_state[BALANCE_LEG_NUM];
    BalanceRefState ref;
    BalanceLegCmd cmd[BALANCE_LEG_NUM];

    float dt;
    bool enable;
    bool safe;
} BalanceRobot;
