#include "balance_app.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "balance_config.h"
#include "balance_motor_if.h"
#include "balance_imu_if.h"
#include "balance_observer.h"
#include "balance_controller.h"
#include "balance_tool.h"

#include "fdcan.h"
#include "dvc_motor_dm.h"
#include "drv_can.h"
#include "bsp_jy61p.h"

// =====================================================
// 全局机器人对象
// =====================================================
BalanceRobot g_balance_robot;

// =====================================================
// 电机对象
// =====================================================
static Class_Motor_DM_Normal g_motor_joint_0;
static Class_Motor_DM_Normal g_motor_joint_1;
static Class_Motor_DM_Normal g_motor_joint_2;
static Class_Motor_DM_Normal g_motor_joint_3;

static Class_Motor_DM_Normal g_motor_wheel_0;
static Class_Motor_DM_Normal g_motor_wheel_1;

// =====================================================
// 任务句柄
// =====================================================
static TaskHandle_t xBalanceControlTaskHandle = NULL;
static TaskHandle_t xBalanceMotorSendTaskHandle = NULL;
static TaskHandle_t xBalanceAliveTaskHandle = NULL;
static TaskHandle_t xBalancePrintTaskHandle = NULL;

// =====================================================
// 运行标志
// =====================================================
static bool g_balance_app_inited = false;
static bool g_balance_app_enabled = false;

// =====================================================
// 模式定义
// 0: 回参考姿态模式
// 1: 正常控制模式
// =====================================================
enum
{
    BAL_APP_MODE_RETURN_REF = 0,
    BAL_APP_MODE_NORMAL = 1,
};

static uint8_t g_balance_mode = BAL_APP_MODE_RETURN_REF;

// =====================================================
// 回位状态
// =====================================================
static bool g_balance_return_ref_done = false;
static uint16_t g_balance_return_ref_stable_count = 0;

// =====================================================
// 内部参数
// =====================================================
namespace
{
    // 回位控制参数
    static constexpr float kReturnRefKp = 20.0f;
    static constexpr float kReturnRefKd = 3.0f;

    // 回位完成判定阈值
    static constexpr float kReturnRefPosEps = 0.05f;   // rad
    static constexpr float kReturnRefVelEps = 0.10f;   // rad/s
    static constexpr uint16_t kReturnRefStableNeed = 20;

    // 模式切换缓冲时间
    static constexpr uint32_t kSwitchDelayMs = 300U;

    static inline float AbsFloat(float x)
    {
        return (x >= 0.0f) ? x : -x;
    }

    static inline void ClearMotorCmd(BalanceMotorCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->pos = 0.0f;
        cmd->vel = 0.0f;
        cmd->tor = 0.0f;
        cmd->kp = 0.0f;
        cmd->kd = 0.0f;
        cmd->enable = false;
    }

    static inline void ClearLegCmd(BalanceLegCmd* cmd)
    {
        if (cmd == nullptr)
        {
            return;
        }

        cmd->rod_f = 0.0f;
        cmd->rod_tp = 0.0f;
        cmd->joint_t[0] = 0.0f;
        cmd->joint_t[1] = 0.0f;
        cmd->wheel_t = 0.0f;
    }
}

// =====================================================
// 内部函数声明
// =====================================================
static void BalanceApp_InitRobot(void);
static void BalanceApp_InitImu(void);
static void BalanceApp_InitMotors(void);

static void BalanceApp_StartReturnToRef(void);
static bool BalanceApp_ReturnToRef_Update(void);
static void BalanceApp_SwitchToNormalMode(void);
static void BalanceApp_SwitchToReturnRefMode(void);

static void vBalanceControlTask(void *pvParameters);
static void vBalanceMotorSendTask(void *pvParameters);
static void vBalanceAliveTask(void *pvParameters);
static void vBalancePrintTask(void *pvParameters);

// =====================================================
// CAN 回调
// =====================================================
void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (0x02):
        g_motor_joint_0.CAN_RxCpltCallback();
        break;
    case (0x03):
        g_motor_joint_1.CAN_RxCpltCallback();
        break;
    default:
        break;
    }
}

// =====================================================
// 初始化
// =====================================================
static void BalanceApp_InitRobot(void)
{
    memset(&g_balance_robot, 0, sizeof(g_balance_robot));
    BalanceObserver_Init(&g_balance_robot);
    BalanceController_Init(&g_balance_robot);
}

static void BalanceApp_InitImu(void)
{
    BalanceImuIf_Init();
}

static void BalanceApp_InitMotors(void)
{
    CAN_Init(&hfdcan1, CAN1_Callback);

    g_motor_joint_0.Init(&hfdcan1, 0x02, 0x02,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_1.Init(&hfdcan1, 0x03, 0x03,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);

    BalanceMotorIf_Init();

    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_motor_joint_0);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_1, &g_motor_joint_1);
}

void BalanceApp_Init(void)
{
    if (g_balance_app_inited)
    {
        return;
    }

    BalanceApp_InitRobot();
    BalanceApp_InitImu();
    BalanceApp_InitMotors();

    g_balance_app_inited = true;
}

// =====================================================
// 模式切换
// =====================================================
static void BalanceApp_StartReturnToRef(void)
{
    g_balance_return_ref_done = false;
    g_balance_return_ref_stable_count = 0;
    g_balance_mode = BAL_APP_MODE_RETURN_REF;
}

static void BalanceApp_SwitchToNormalMode(void)
{
    // 先停一下，避免两个模式切换瞬间抢命令
    BalanceController_Stop(&g_balance_robot);
    BalanceMotorIf_SendCommand(&g_balance_robot);
    vTaskDelay(pdMS_TO_TICKS(kSwitchDelayMs));

    g_balance_mode = BAL_APP_MODE_NORMAL;
}

static void BalanceApp_SwitchToReturnRefMode(void)
{
    // 从正常模式切回回位模式时也先清一次
    BalanceController_Stop(&g_balance_robot);
    BalanceMotorIf_SendCommand(&g_balance_robot);
    vTaskDelay(pdMS_TO_TICKS(kSwitchDelayMs));

    BalanceApp_StartReturnToRef();
}

// =====================================================
// 使能 / 失能
// =====================================================
void BalanceApp_Enable(void)
{
    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    g_balance_app_enabled = true;
    g_balance_robot.enable = true;
    g_balance_robot.safe = false;

    BalanceMotorIf_SendEnterAll();

    // 默认使能后先回参考姿态
    BalanceApp_StartReturnToRef();
}

void BalanceApp_Disable(void)
{
    g_balance_app_enabled = false;
    g_balance_robot.enable = false;
    g_balance_robot.safe = true;

    BalanceController_Stop(&g_balance_robot);
    BalanceMotorIf_SendCommand(&g_balance_robot);
    BalanceMotorIf_SendExitAll();
}

// =====================================================
// 回参考姿态更新
//
// 说明：
// 这里假设 joint_angle_unwrap[] 结构里有 continuous_angle 成员。
// 如果你工程里这个成员名不一样，把下面 q0/q1 的读取改成你真实的成员名。
// =====================================================
static bool BalanceApp_ReturnToRef_Update(void)
{
    const float q0 = g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous;
    const float q1 = g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous;

    const float dq0 = g_balance_robot.joint_motor_fdb[BAL_JOINT_L_0].vel;
    const float dq1 = g_balance_robot.joint_motor_fdb[BAL_JOINT_L_1].vel;

    const float q0_ref = BALANCE_JOINT_L0_CONT_REF;
    const float q1_ref = BALANCE_JOINT_L1_CONT_REF;

    const float e0 = q0_ref - q0;
    const float e1 = q1_ref - q1;

    // 清掉腿部虚拟控制量，避免概念混淆
    for (int i = 0; i < BALANCE_LEG_NUM; ++i)
    {
        ClearLegCmd(&g_balance_robot.cmd[i]);
    }

    // 左腿两个关节直接发 MIT 位置回位命令
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].pos = q0_ref;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].vel = 0.0f;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].tor = 0.0f;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].kp  = kReturnRefKp;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].kd  = kReturnRefKd;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].enable = true;

    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_1].pos = q1_ref;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_1].vel = 0.0f;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_1].tor = 0.0f;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_1].kp  = kReturnRefKp;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_1].kd  = kReturnRefKd;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_1].enable = true;

    // 其余关节与轮子先关闭
    for (int i = 2; i < BALANCE_JOINT_NUM; ++i)
    {
        ClearMotorCmd(&g_balance_robot.joint_motor_cmd[i]);
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        ClearMotorCmd(&g_balance_robot.wheel_motor_cmd[i]);
    }

    // 到位判定
    const bool pos_ok =
        (AbsFloat(e0) < kReturnRefPosEps) &&
        (AbsFloat(e1) < kReturnRefPosEps);

    const bool vel_ok =
        (AbsFloat(dq0) < kReturnRefVelEps) &&
        (AbsFloat(dq1) < kReturnRefVelEps);

    if (pos_ok && vel_ok)
    {
        if (g_balance_return_ref_stable_count < 0xFFFFU)
        {
            g_balance_return_ref_stable_count++;
        }
    }
    else
    {
        g_balance_return_ref_stable_count = 0;
    }

    if (g_balance_return_ref_stable_count >= kReturnRefStableNeed)
    {
        g_balance_return_ref_done = true;
        return true;
    }

    return false;
}

// =====================================================
// 控制任务
// 2ms：反馈 + IMU + observer + 模式控制
// =====================================================
static void vBalanceControlTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    BalanceApp_Enable();

    for (;;)
    {
        BalanceMotorIf_UpdateFeedback(&g_balance_robot);
        BalanceImuIf_Update(&g_balance_robot.imu);
        BalanceObserver_UpdateAll(&g_balance_robot);

        if (g_balance_robot.enable && !g_balance_robot.safe)
        {
            if (g_balance_mode == BAL_APP_MODE_RETURN_REF)          // 如果模式是返回模式
            {
                const bool done = BalanceApp_ReturnToRef_Update();  // 开始返回
                if (done)
                {
                    BalanceApp_SwitchToNormalMode();                // 返回结束
                }
            }
            else
            {
                BalanceController_SetRef(&g_balance_robot);
                BalanceController_LegLength(&g_balance_robot);
                BalanceController_LegAngle(&g_balance_robot);
                BalanceController_Output(&g_balance_robot);
            }
        }
        else
        {
            BalanceController_Stop(&g_balance_robot);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =====================================================
// 电机发送任务
// =====================================================
static void vBalanceMotorSendTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(600));

    for (;;)
    {
        BalanceMotorIf_SendCommand(&g_balance_robot);
        BalanceMotorIf_TxAllPeriodic();

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =====================================================
// 保活任务
// =====================================================
static void vBalanceAliveTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;)
    {
        BalanceMotorIf_AliveAllPeriodic();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =====================================================
// 串口打印任务
// =====================================================
static void vBalancePrintTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1200));

    for (;;)
    {
        BalanceTool_PrintRaw("\r\n[balance]\r\n");

        BalanceTool_PrintFloat4Line("L0",
                                    g_balance_robot.leg[0].rod.l0,
                                    "dL0",
                                    g_balance_robot.leg[0].rod.dl0);

        BalanceTool_PrintFloat4Line("phi0_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].rod.phi0),
                                    "dphi0_dps",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].rod.dphi0));

        BalanceTool_PrintFloat4Line("rod_f",
                                    g_balance_robot.cmd[0].rod_f,
                                    "rod_tp",
                                    g_balance_robot.cmd[0].rod_tp);

        BalanceTool_PrintFloat4Line("pitch_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.imu.pitch),
                                    "pitch_dps",
                                    BalanceTool_RadToDeg(g_balance_robot.imu.pitch_dot));

        BalanceTool_PrintFloat4Line("phi1_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].joint.phi1),
                                    "phi4_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].joint.phi4));

        // 回位相关打印
        BalanceTool_PrintFloat4Line("q0_cont",
                                    g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous,       // 连续角
                                    "q1_cont",
                                    g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous);      // 连续角

        BalanceTool_PrintFloat4Line("q0_err",
                                    BALANCE_JOINT_L0_CONT_REF - g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous,
                                    "q1_err",
                                    BALANCE_JOINT_L1_CONT_REF - g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous);

        BalanceTool_PrintFloat4Line("mode",
                                    (g_balance_mode == BAL_APP_MODE_RETURN_REF) ? 0.0f : 1.0f,
                                    "ret_done",
                                    g_balance_return_ref_done ? 1.0f : 0.0f);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =====================================================
// 创建任务
// =====================================================
void BalanceApp_Task_Create(void)
{
    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    BaseType_t ret;

    ret = xTaskCreate(vBalanceControlTask,
                      "vBalanceControlTask",
                      512,
                      NULL,
                      3,
                      &xBalanceControlTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceMotorSendTask,
                      "vBalanceMotorSendTask",
                      384,
                      NULL,
                      4,
                      &xBalanceMotorSendTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceAliveTask,
                      "vBalanceAliveTask",
                      256,
                      NULL,
                      2,
                      &xBalanceAliveTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalancePrintTask,
                      "vBalancePrintTask",
                      512,
                      NULL,
                      1,
                      &xBalancePrintTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }
}