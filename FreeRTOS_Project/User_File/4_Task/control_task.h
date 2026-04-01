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



void Control_Task_Create(void);

#ifdef __cplusplus
}
#endif

#endif

