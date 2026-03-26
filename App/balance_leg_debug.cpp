#include "balance_leg_debug.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "balance_config.h"
#include "balance_types.h"
#include "balance_motor_if.h"
#include "balance_observer.h"
#include "balance_kinematics.h"

#include "usart.h"
#include "drv_can.h"

#include "dvc_motor_dm.h"

// 如果你的电机初始化需要 CAN 句柄，在这里 include
// #include "can.h"

// ==============================
// 全局调试机器人对象
// 这里只用左腿两个关节电机
// ==============================
static BalanceRobot g_leg_debug_robot;

// ==============================
// 两个测试电机对象
// 你后面把 Init(...) 补进去
// ==============================
static Class_Motor_DM_Normal g_leg_debug_joint_0;
static Class_Motor_DM_Normal g_leg_debug_joint_1;

// ==============================
// 任务句柄
// ==============================
static TaskHandle_t xBalanceLegDebugCtrlTaskHandle = NULL;
static TaskHandle_t xBalanceLegDebugSendTaskHandle = NULL;
static TaskHandle_t xBalanceLegDebugAliveTaskHandle = NULL;
static TaskHandle_t xBalanceLegDebugPrintTaskHandle = NULL;

// ==============================
// 调试参数
// ==============================
static bool  g_leg_debug_inited = false;
static bool  g_leg_debug_enable = false;
static float g_leg_target_length = BALANCE_DEFAULT_LEG_LEN_STAND;
static float g_leg_kp = 100.0f;
static float g_leg_kd = 6.0f;

// 力方向控制量限幅（先保守）
static float g_leg_force_limit = 40.0f;

// ==============================
// 自动回参考姿态逻辑
// ==============================

// ==============================
// 工具函数
// ==============================
static inline float Clamp(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static void ClearMotorCmd(BalanceMotorCmd* cmd)
{
    if (cmd == nullptr)
    {
        return;
    }

    cmd->pos = 0.0f;
    cmd->vel = 0.0f;
    cmd->tor = 0.0f;
    cmd->kp  = 0.0f;
    cmd->kd  = 0.0f;
    cmd->enable = false;
}

static void ClearAllMotorCmd(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        ClearMotorCmd(&robot->joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        ClearMotorCmd(&robot->wheel_motor_cmd[i]);
    }
}

static void BalanceLegDebug_PrintUart7(const char* text)
{
    if (text == nullptr)
    {
        return;
    }

    HAL_UART_Transmit(&huart7,
                      reinterpret_cast<uint8_t*>(const_cast<char*>(text)),
                      static_cast<uint16_t>(strlen(text)),
                      20);
}

static int BalanceLegDebug_AppendFloat4(char* buf, size_t buf_size, const char* name, float value)
{
    if ((buf == nullptr) || (buf_size == 0U) || (name == nullptr))
    {
        return 0;
    }

    const char sign = (value < 0.0f) ? '-' : '\0';
    const float abs_value = (value < 0.0f) ? -value : value;
    const long scaled = static_cast<long>(abs_value * 10000.0f + 0.5f);
    const long integer_part = scaled / 10000L;
    const long fraction_part = scaled % 10000L;

    if (sign != '\0')
    {
        return snprintf(buf,
                        buf_size,
                        "%s=%c%ld.%04ld",
                        name,
                        sign,
                        integer_part,
                        fraction_part);
    }

    return snprintf(buf,
                    buf_size,
                    "%s=%ld.%04ld",
                    name,
                    integer_part,
                    fraction_part);
}

// 达妙单圈范围近似 [-12.5, 12.5]

// 已知当前 unwrap 状态，把 continuous 目标映射到“最近的 raw 目标”

// ==============================
// 初始化部分
// ==============================
static void BalanceLegDebug_InitRobot(void)
{
    memset(&g_leg_debug_robot, 0, sizeof(g_leg_debug_robot));

    g_leg_debug_robot.dt = BALANCE_CTRL_DT;
    g_leg_debug_robot.enable = false;
    g_leg_debug_robot.safe = true;

    BalanceObserver_Init(&g_leg_debug_robot);
    ClearAllMotorCmd(&g_leg_debug_robot);
}

static void BalanceLegDebug_CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    switch (Header.Identifier)
    {
    case (0x02): // MasterID
        g_leg_debug_joint_0.CAN_RxCpltCallback();
        break;
    case (0x03): // MasterID
        g_leg_debug_joint_1.CAN_RxCpltCallback();
        break;
    }
}

static void BalanceLegDebug_InitMotors(void)
{
    // ==========================================
    // 这里只接左腿两个关节电机
    // 不接轮子
    // ==========================================

    CAN_Init(&hfdcan1, BalanceLegDebug_CAN1_Callback);
    g_leg_debug_joint_0.Init(&hfdcan1, 0x02, 0x02,
                             Motor_DM_Control_Method_NORMAL_MIT,
                             12.5f, 25.0f, 10.0f, 10.261194f);
    g_leg_debug_joint_1.Init(&hfdcan1, 0x03, 0x03,
                             Motor_DM_Control_Method_NORMAL_MIT,
                             12.5f, 25.0f, 10.0f, 10.261194f);

    BalanceMotorIf_Init();

    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_leg_debug_joint_0);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_1, &g_leg_debug_joint_1);
}

static void BalanceLegDebug_Init(void)
{
    if (g_leg_debug_inited)
    {
        return;
    }

    BalanceLegDebug_InitRobot();
    BalanceLegDebug_InitMotors();

    g_leg_debug_inited = true;
}

// ==============================
// 对外接口
// ==============================
void BalanceLegDebug_Enable(void)
{
    if (!g_leg_debug_inited)
    {
        BalanceLegDebug_Init();
    }

    g_leg_debug_enable = true;                                      // 开启使能

    g_leg_target_length = Clamp(BALANCE_DEFAULT_LEG_LEN_STAND,
                                BALANCE_DEFAULT_LEG_LEN_MIN,
                                BALANCE_DEFAULT_LEG_LEN_MAX);

    BalanceMotorIf_SendEnterAll();
}

void BalanceLegDebug_Disable(void)
{
    g_leg_debug_enable = false;

    g_leg_target_length = Clamp(BALANCE_DEFAULT_LEG_LEN_STAND,
                                BALANCE_DEFAULT_LEG_LEN_MIN,
                                BALANCE_DEFAULT_LEG_LEN_MAX);

    ClearAllMotorCmd(&g_leg_debug_robot);
    BalanceMotorIf_SendCommand(&g_leg_debug_robot);
    BalanceMotorIf_SendExitAll();
}

void BalanceLegDebug_SetTargetLength(float target_len)
{
    g_leg_target_length = Clamp(target_len,
                                BALANCE_DEFAULT_LEG_LEN_MIN,
                                BALANCE_DEFAULT_LEG_LEN_MAX);
}

void BalanceLegDebug_SetGains(float kp, float kd)
{
    g_leg_kp = kp;
    g_leg_kd = kd;
}

// ==============================
// 回参考姿态控制
// 让 continuous -> CONT_REF
// 到位后，observer 里的 phi 就会等于 PHI_REF
// ==============================

// ==============================
// 单腿腿长控制核心
// 只控左腿：joint0 + joint1
// ==============================
static void BalanceLegDebug_RunController(void)
{
    // 只关心左腿
    BalanceLegState& leg = g_leg_debug_robot.leg[0];

    // 腿长误差
    const float err_l = g_leg_target_length - leg.rod.l0;

    // 简单 PD
    float rod_f = g_leg_kp * err_l - g_leg_kd * leg.rod.dl0;
    rod_f = Clamp(rod_f, -g_leg_force_limit, g_leg_force_limit);

    // 第一阶段不做摆角控制
    const float rod_tp = 0.0f;

    // 重新计算 Jacobian（只针对左腿）
    float J[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    BalanceCalcJacobian(leg.joint.phi1, leg.joint.phi4, J);

    // 虚拟力 -> 两个关节力矩
    float joint_t[2] = {0.0f, 0.0f};
    BalanceCalcVmc(rod_f, rod_tp, J, joint_t);

    // 写入调试输出，方便你看变量
    g_leg_debug_robot.cmd[0].rod_f = rod_f;
    g_leg_debug_robot.cmd[0].rod_tp = rod_tp;
    g_leg_debug_robot.cmd[0].joint_t[0] = joint_t[0];
    g_leg_debug_robot.cmd[0].joint_t[1] = joint_t[1];

    // 只给左腿两个关节写命令
    ClearAllMotorCmd(&g_leg_debug_robot);

    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_0].pos = 0.0f;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_0].vel = 0.0f;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_0].tor =
        Clamp(joint_t[0], -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT, BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_0].kp = BALANCE_DEFAULT_JOINT_MIT_KP;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_0].kd = BALANCE_DEFAULT_JOINT_MIT_KD;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_0].enable = true;

    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_1].pos = 0.0f;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_1].vel = 0.0f;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_1].tor =
        Clamp(joint_t[1], -BALANCE_DEFAULT_JOINT_TORQUE_LIMIT, BALANCE_DEFAULT_JOINT_TORQUE_LIMIT);
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_1].kp = BALANCE_DEFAULT_JOINT_MIT_KP;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_1].kd = BALANCE_DEFAULT_JOINT_MIT_KD;
    g_leg_debug_robot.joint_motor_cmd[BAL_JOINT_L_1].enable = true;
}

// ==============================
// 任务 1：控制任务
// 读反馈 -> observer -> 先回参考姿态 -> 单腿腿长控制
// ==============================
static void vBalanceLegDebugCtrlTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!g_leg_debug_inited)
    {
        BalanceLegDebug_Init();
    }

    // 默认不自动开，你手动调用 Enable 更安全
    BalanceLegDebug_Enable();

    for (;;)
    {
        // 1. 更新电机反馈
        BalanceMotorIf_UpdateFeedback(&g_leg_debug_robot);

        // 2. 单腿调试时不需要 IMU，机体姿态先固定为 0
        g_leg_debug_robot.body.phi = 0.0f;
        g_leg_debug_robot.body.phi_dot = 0.0f;

        // 3. 更新腿状态
        BalanceObserver_UpdateLeg(&g_leg_debug_robot);

        if (g_leg_debug_enable)
        {
            BalanceLegDebug_RunController();
        }
        else
        {
            ClearAllMotorCmd(&g_leg_debug_robot);
        }

        // 这些量你可以在调试器里重点看
        volatile float phi1   = g_leg_debug_robot.leg[0].joint.phi1;
        volatile float phi4   = g_leg_debug_robot.leg[0].joint.phi4;
        volatile float l0     = g_leg_debug_robot.leg[0].rod.l0;
        volatile float phi0   = g_leg_debug_robot.leg[0].rod.phi0;
        volatile float dl0    = g_leg_debug_robot.leg[0].rod.dl0;
        volatile float dphi0  = g_leg_debug_robot.leg[0].rod.dphi0;
        volatile float rod_f  = g_leg_debug_robot.cmd[0].rod_f;
        volatile float jt0    = g_leg_debug_robot.cmd[0].joint_t[0];
        volatile float jt1    = g_leg_debug_robot.cmd[0].joint_t[1];
        volatile float c0     = g_leg_debug_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous;
        volatile float c1     = g_leg_debug_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous;
        volatile float e0     = c0 - BALANCE_JOINT_L0_CONT_REF;
        volatile float e1     = c1 - BALANCE_JOINT_L1_CONT_REF;

        (void)phi1;
        (void)phi4;
        (void)l0;
        (void)phi0;
        (void)dl0;
        (void)dphi0;
        (void)rod_f;
        (void)jt0;
        (void)jt1;
        (void)c0;
        (void)c1;
        (void)e0;
        (void)e1;

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ==============================
// 任务 2：发送任务
// 把 cmd 写入电机对象，并真正周期发送
// ==============================
static void vBalanceLegDebugSendTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(600));

    for (;;)
    {
        BalanceMotorIf_SendCommand(&g_leg_debug_robot);
        BalanceMotorIf_TxAllPeriodic();

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ==============================
// 任务 3：保活任务
// ==============================
static void vBalanceLegDebugAliveTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;)
    {
        BalanceMotorIf_AliveAllPeriodic();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==============================
// 任务 4：串口7打印调试信息
// ==============================
static void vBalanceLegDebugPrintTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1200));

    char tx_buf[180] = {0};

    for (;;)
    {
        const BalanceLegState& leg = g_leg_debug_robot.leg[0];
        int written = 0;

        written = snprintf(tx_buf, sizeof(tx_buf), "leg_dbg\r\n");
        if (written > 0)
        {
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        written = snprintf(tx_buf, sizeof(tx_buf), "");
        size_t used = (written >= 0) ? static_cast<size_t>(written) : 0U;
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "phi1", leg.joint.phi1);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "phi4", leg.joint.phi4);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        written = snprintf(tx_buf, sizeof(tx_buf), "");
        used = (written >= 0) ? static_cast<size_t>(written) : 0U;
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used,
                                                   "c0", g_leg_debug_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used,
                                                   "c1", g_leg_debug_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        written = snprintf(tx_buf, sizeof(tx_buf), "");
        used = (written >= 0) ? static_cast<size_t>(written) : 0U;
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used,
                                                   "e0",
                                                   g_leg_debug_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous -
                                                   BALANCE_JOINT_L0_CONT_REF);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used,
                                                   "e1",
                                                   g_leg_debug_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous -
                                                   BALANCE_JOINT_L1_CONT_REF);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        written = snprintf(tx_buf, sizeof(tx_buf), "");
        used = (written >= 0) ? static_cast<size_t>(written) : 0U;
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "l_ref", g_leg_target_length);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "l0", leg.rod.l0);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "phi0", leg.rod.phi0);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        written = snprintf(tx_buf, sizeof(tx_buf), "");
        used = (written >= 0) ? static_cast<size_t>(written) : 0U;
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "err_l", g_leg_target_length - leg.rod.l0);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "dl0", leg.rod.dl0);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "dphi0", leg.rod.dphi0);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        written = snprintf(tx_buf, sizeof(tx_buf), "");
        used = (written >= 0) ? static_cast<size_t>(written) : 0U;
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "rod_f", g_leg_debug_robot.cmd[0].rod_f);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "jt0", g_leg_debug_robot.cmd[0].joint_t[0]);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            written = BalanceLegDebug_AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "jt1", g_leg_debug_robot.cmd[0].joint_t[1]);
            if (written > 0) used += static_cast<size_t>(written);
        }
        if (used < sizeof(tx_buf))
        {
            snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
            BalanceLegDebug_PrintUart7(tx_buf);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==============================
// 创建任务
// ==============================
void BalanceLegDebug_Task_Create(void)
{
    if (!g_leg_debug_inited)
    {
        BalanceLegDebug_Init();
    }

    BaseType_t ret;

    ret = xTaskCreate(vBalanceLegDebugCtrlTask,
                      "vLegDbgCtrlTask",
                      512,
                      NULL,
                      3,
                      &xBalanceLegDebugCtrlTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceLegDebugSendTask,
                      "vLegDbgSendTask",
                      384,
                      NULL,
                      4,
                      &xBalanceLegDebugSendTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceLegDebugAliveTask,
                      "vLegDbgAliveTask",
                      256,
                      NULL,
                      2,
                      &xBalanceLegDebugAliveTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceLegDebugPrintTask,
                      "vLegDbgPrintTask",
                      384,
                      NULL,
                      1,
                      &xBalanceLegDebugPrintTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }
}
