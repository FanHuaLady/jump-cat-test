#ifndef __MOTOR_DM_TASK_H
#define __MOTOR_DM_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

// 电机ID定义
#define MOTOR_X      0          // 水平电机
#define MOTOR_Y      1          // 竖直电机
#define MOTOR_NONE   0xFF       // 无效电机ID，表示不控制电机

#define MOTOR_DM_X_MasterID     0x11        // CAN_Rx_ID
#define MOTOR_DM_X_CANID        0x10        // CAN_Tx_ID

#define MOTOR_DM_Y_MasterID     0x00        // CAN_Rx_ID
#define MOTOR_DM_Y_CANID        0x01        // CAN_Tx_ID


// =========================
// MIT 模式参数限幅宏定义
// =========================

// 水平电机限幅值
#define MOTOR_X_KP_MIN     0.0f
#define MOTOR_X_KP_MAX     10.0f
#define MOTOR_X_KD_MIN     0.0f
#define MOTOR_X_KD_MAX     10.0f
#define MOTOR_X_POS_MIN    0.0f
#define MOTOR_X_POS_MAX    3.14f      // π rad，约180°
#define MOTOR_X_VEL_MIN   -5.0f
#define MOTOR_X_VEL_MAX    5.0f       // 约300°/s
#define MOTOR_X_TOR_MIN    0.0f
#define MOTOR_X_TOR_MAX    10.0f

// 竖直电机限幅值
#define MOTOR_Y_KP_MIN     0.0f
#define MOTOR_Y_KP_MAX     10.0f
#define MOTOR_Y_KD_MIN     0.0f
#define MOTOR_Y_KD_MAX     10.0f
#define MOTOR_Y_POS_MIN    0.0f
#define MOTOR_Y_POS_MAX    3.14f      // π rad，约180°
#define MOTOR_Y_VEL_MIN   -5.0f
#define MOTOR_Y_VEL_MAX    5.0f       // 约300°/s
#define MOTOR_Y_TOR_MIN    0.0f
#define MOTOR_Y_TOR_MAX    10.0f

void MOTOR_DM_Task_Create(void);

#ifdef __cplusplus
}
#endif

#endif

