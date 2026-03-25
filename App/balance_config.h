#pragma once

// =========================
// 基础规模配置
// =========================
#define BALANCE_LEG_NUM               2
#define BALANCE_JOINT_PER_LEG         2
#define BALANCE_JOINT_NUM             4
#define BALANCE_WHEEL_NUM             2

// 控制周期，先给 2ms
#define BALANCE_CTRL_DT               0.002f

// =========================
// 默认物理参数
// =========================
#define BALANCE_DEFAULT_BODY_MASS     12.0f
#define BALANCE_DEFAULT_WHEEL_RADIUS  0.060f
#define BALANCE_DEFAULT_GRAVITY       9.81f

#define BALANCE_DEFAULT_LEG_L1        0.105f
#define BALANCE_DEFAULT_LEG_L2        0.125f
#define BALANCE_DEFAULT_LEG_L3        0.125f
#define BALANCE_DEFAULT_LEG_L4        0.105f
#define BALANCE_DEFAULT_LEG_L5        0.000f

#define BALANCE_DEFAULT_LEG_LEN_MIN   0.100f
#define BALANCE_DEFAULT_LEG_LEN_MAX   0.320f
#define BALANCE_DEFAULT_LEG_LEN_STAND 0.100f

#define BALANCE_DEFAULT_ON_LENGTH     0.230f
#define BALANCE_DEFAULT_CN_LENGTH     0.125f
#define BALANCE_DEFAULT_WN_LENGTH     0.274f

#define BALANCE_DEFAULT_ON_SIGN       1.0f
#define BALANCE_DEFAULT_WN_SIGN       1.0f

#define BALANCE_DEFAULT_MIN_OP_LEN    1e-5f

// =========================
// 关节安装修正
// 连续角只负责解圈，真正用于几何计算的关节角 = sign * continuous + offset
// offset 单位是 rad
// =========================
#define BALANCE_JOINT_L0_SIGN         1.0f
#define BALANCE_JOINT_L1_SIGN         1.0f
#define BALANCE_JOINT_R0_SIGN         1.0f
#define BALANCE_JOINT_R1_SIGN         1.0f

#define BALANCE_JOINT_L0_OFFSET       0.2f
#define BALANCE_JOINT_L1_OFFSET       0.2f
#define BALANCE_JOINT_R0_OFFSET       0.0f
#define BALANCE_JOINT_R1_OFFSET       0.0f

// =========================
// 默认安全限制
// =========================
#define BALANCE_DEFAULT_JOINT_TORQUE_LIMIT  1.0f
#define BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT  1.0f

#define BALANCE_DEFAULT_JOINT_KP_LIMIT      5.0f
#define BALANCE_DEFAULT_JOINT_KD_LIMIT      2.0f

#define BALANCE_DEFAULT_BODY_PITCH_LIMIT    0.60f
#define BALANCE_DEFAULT_BODY_ROLL_LIMIT     0.60f

// =========================
// 电机默认 MIT 参数
// =========================
#define BALANCE_DEFAULT_JOINT_MIT_KP        0.0f
#define BALANCE_DEFAULT_JOINT_MIT_KD        0.2f

#define BALANCE_DEFAULT_WHEEL_MIT_KP        0.0f
#define BALANCE_DEFAULT_WHEEL_MIT_KD        0.1f

// =========================
// 逻辑编号约定
// =========================
enum BalanceJointIndex
{
    BAL_JOINT_L_0 = 0,
    BAL_JOINT_L_1 = 1,
    BAL_JOINT_R_0 = 2,
    BAL_JOINT_R_1 = 3,
};

enum BalanceWheelIndex
{
    BAL_WHEEL_L = 0,
    BAL_WHEEL_R = 1,
};
