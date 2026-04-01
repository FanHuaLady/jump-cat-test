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
// 默认物理参数（先占位）
// 后面你再自己改
// =========================
#define BALANCE_DEFAULT_BODY_MASS     12.0f
#define BALANCE_DEFAULT_WHEEL_RADIUS  0.060f
#define BALANCE_DEFAULT_GRAVITY       9.81f

#define BALANCE_DEFAULT_LEG_L1        0.110f
#define BALANCE_DEFAULT_LEG_L2        0.250f
#define BALANCE_DEFAULT_LEG_L3        0.250f
#define BALANCE_DEFAULT_LEG_L4        0.110f
#define BALANCE_DEFAULT_LEG_L5        0.000f

#define BALANCE_DEFAULT_LEG_LEN_MIN   0.100f
#define BALANCE_DEFAULT_LEG_LEN_MAX   0.320f
#define BALANCE_DEFAULT_LEG_LEN_STAND 0.200f

// =========================
// 默认安全限制
// =========================
#define BALANCE_DEFAULT_JOINT_TORQUE_LIMIT  10.0f
#define BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT  10.0f

#define BALANCE_DEFAULT_JOINT_KP_LIMIT      500.0f
#define BALANCE_DEFAULT_JOINT_KD_LIMIT      5.0f

#define BALANCE_DEFAULT_BODY_PITCH_LIMIT    0.60f
#define BALANCE_DEFAULT_BODY_ROLL_LIMIT     0.60f

// =========================
// 电机缺省 MIT 参数
// 这里只是“安全起步值”
// 后续你可以在控制器里覆盖
// =========================
#define BALANCE_DEFAULT_JOINT_MIT_KP        0.0f
#define BALANCE_DEFAULT_JOINT_MIT_KD        0.2f

#define BALANCE_DEFAULT_WHEEL_MIT_KP        0.0f
#define BALANCE_DEFAULT_WHEEL_MIT_KD        0.1f

// =========================
// 逻辑编号约定（非常重要）
// 以后只改注册，不改控制代码
// =========================
enum BalanceJointIndex
{
    BAL_JOINT_L_0 = 0,   // 左腿关节0
    BAL_JOINT_L_1 = 1,   // 左腿关节1
    BAL_JOINT_R_0 = 2,   // 右腿关节0
    BAL_JOINT_R_1 = 3,   // 右腿关节1
};

enum BalanceWheelIndex
{
    BAL_WHEEL_L = 0,     // 左轮
    BAL_WHEEL_R = 1,     // 右轮
};