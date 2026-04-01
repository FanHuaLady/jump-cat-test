#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

// ========== 遥控器旋钮配置宏 ==========
// 旋钮原始值范围
#define RC_KNOB_MAX_VALUE       783     // 最大值（顺时针最大）
#define RC_KNOB_MIN_VALUE      -784     // 最小值（逆时针最大）
#define RC_KNOB_DEADZONE        20      // 死区范围，±20以内视为0

// 目标角度映射范围
#define TARGET_ANGLE_MAX        180.0f  // 旋钮打满对应 +180°
#define TARGET_ANGLE_MIN       -180.0f  // 旋钮打满对应 -180°



// ========== 遥控器摇杆配置宏（角速度控制）==========
// 摇杆原始值范围（根据实际测试，ch[0]和ch[1]的范围）
#define RC_STICK_MAX_VALUE      783     // 最大值（摇杆打满）
#define RC_STICK_MIN_VALUE     -784     // 最小值（摇杆打满反方向）
#define RC_STICK_DEADZONE       20      // 死区范围，±20以内视为0


// 也可以单独设置Yaw和Roll的最大速度
#define TARGET_YAW_SPEED_MAX    180.0f   // Yaw最大角速度（°/s）
#define TARGET_YAW_SPEED_MIN   -180.0f
#define TARGET_ROLL_SPEED_MAX   180.0f   // Roll最大角速度（°/s）
#define TARGET_ROLL_SPEED_MIN  -180.0f



void Control_Task_Create(void);

#ifdef __cplusplus
}
#endif

#endif

