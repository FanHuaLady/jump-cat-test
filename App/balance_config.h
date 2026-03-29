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
#define BALANCE_DEFAULT_LEG_LEN_MAX   0.400f
#define BALANCE_DEFAULT_LEG_LEN_STAND 0.200f

#define BALANCE_DEFAULT_ON_LENGTH     0.230f
#define BALANCE_DEFAULT_CN_LENGTH     0.125f
#define BALANCE_DEFAULT_WN_LENGTH     0.274f

#define BALANCE_DEFAULT_ON_SIGN       1.0f
#define BALANCE_DEFAULT_WN_SIGN       1.0f

#define BALANCE_DEFAULT_MIN_OP_LEN    1e-5f

// =========================
// 关节安装修正 + 参考姿态标定
//
// continuous 只负责解圈。
// 真正用于几何计算的关节角：
// phi = sign * (continuous - cont_ref) + phi_ref
//
// 其中：
// cont_ref : 参考姿态下，该电机的连续角读数（单位 rad）
// phi_ref  : 同一参考姿态下，模型里的关节角（单位 rad）
// =========================

// ---- 方向修正 ----
#define BALANCE_JOINT_L0_SIGN         1.0f
#define BALANCE_JOINT_L1_SIGN         1.0f
#define BALANCE_JOINT_R0_SIGN         1.0f
#define BALANCE_JOINT_R1_SIGN         1.0f

// ---- 参考姿态下的电机连续角读数 a ----
// 这里先给 0，占位，等你实测后填进去
#define BALANCE_JOINT_L0_CONT_REF     0.2117f
#define BALANCE_JOINT_L1_CONT_REF     1.7568f
#define BALANCE_JOINT_R0_CONT_REF     0.0f
#define BALANCE_JOINT_R1_CONT_REF     0.0f

// ---- 参考姿态下的模型角 phi_ref ----
// 按 O 为原点、x 右、y 上、逆时针为正 的坐标系填写
// 这里先给 0，占位，等你确定后填进去
#define BALANCE_JOINT_L0_PHI_REF      -2.35619f
#define BALANCE_JOINT_L1_PHI_REF      -0.7854f
#define BALANCE_JOINT_R0_PHI_REF      0.0f
#define BALANCE_JOINT_R1_PHI_REF      0.0f

// =========================
// 默认安全限制
// =========================
#define BALANCE_DEFAULT_JOINT_TORQUE_LIMIT  2.0f
#define BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT  1.0f

#define BALANCE_DEFAULT_JOINT_KP_LIMIT      20.0f
#define BALANCE_DEFAULT_JOINT_KD_LIMIT      3.0f

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