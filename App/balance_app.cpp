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
#include "balance_ref_pose.h"

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
// 参考姿态模块状态
// =====================================================
static BalanceRefPoseState g_balance_ref_pose;

// =====================================================
// 内部参数
// =====================================================
namespace
{
    // 模式切换缓冲时间
    static constexpr uint32_t kSwitchDelayMs = 1500U;

    static inline float ClampFloat(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
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
    case (0x05):
        g_motor_joint_0.CAN_RxCpltCallback();
        break;
    case (0x04):
        g_motor_joint_1.CAN_RxCpltCallback();
        break;
    case (0x02):
        g_motor_joint_2.CAN_RxCpltCallback();
        break;
    case (0x03):
        g_motor_joint_3.CAN_RxCpltCallback();
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
    BalanceRefPose_Init(&g_balance_ref_pose);
}

static void BalanceApp_InitImu(void)
{
    BalanceImuIf_Init();
}

static void BalanceApp_InitMotors(void)
{
    CAN_Init(&hfdcan1, CAN1_Callback);

    g_motor_joint_0.Init(&hfdcan1, 0x05, 0x05,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_1.Init(&hfdcan1, 0x04, 0x04,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_2.Init(&hfdcan1, 0x02, 0x02,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_3.Init(&hfdcan1, 0x03, 0x03,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 25.0f, 10.0f, 10.261194f);

    BalanceMotorIf_Init();

    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_motor_joint_1);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_1, &g_motor_joint_0);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_R_0, &g_motor_joint_2);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_R_1, &g_motor_joint_3);
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
    BalanceRefPose_Start(&g_balance_ref_pose);
    g_balance_mode = BALANCE_REF_POSE_MODE_RIGHT_ONLY;
}

static void BalanceApp_SwitchToNormalMode(void)
{
    BalanceRefPose_Stop(&g_balance_ref_pose);
    BalanceController_Stop(&g_balance_robot);
    BalanceMotorIf_SendCommand(&g_balance_robot);
    vTaskDelay(pdMS_TO_TICKS(kSwitchDelayMs));

    g_balance_mode = BAL_APP_MODE_NORMAL;
}

static void BalanceApp_SwitchToReturnRefMode(void)
{
    BalanceRefPose_Stop(&g_balance_ref_pose);
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

    BalanceRefPose_Stop(&g_balance_ref_pose);
    BalanceController_Stop(&g_balance_robot);
    BalanceMotorIf_SendCommand(&g_balance_robot);
    BalanceMotorIf_SendExitAll();
}

// =====================================================
// 控制任务
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
        BalanceMotorIf_UpdateFeedback(&g_balance_robot);   // 获取电机数据
        BalanceImuIf_Update(&g_balance_robot.imu);
        BalanceObserver_UpdateAll(&g_balance_robot);

        if (g_balance_robot.enable && !g_balance_robot.safe)
        {
            if (g_balance_mode == BAL_APP_MODE_RETURN_REF)
            {
                for (int i = 0; i < BALANCE_LEG_NUM; ++i)
                {
                    ClearLegCmd(&g_balance_robot.cmd[i]);
                }

                BalanceRefPose_Update(&g_balance_ref_pose, &g_balance_robot);

                if (BalanceRefPose_IsFinished(&g_balance_ref_pose))
                {
                    BalanceApp_SwitchToNormalMode();
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
            BalanceRefPose_Stop(&g_balance_ref_pose);
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
        const float q0 = g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous;
        const float q1 = g_balance_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous;
        const float dq0 = g_balance_robot.joint_motor_fdb[BAL_JOINT_L_0].vel;
        const float dq1 = g_balance_robot.joint_motor_fdb[BAL_JOINT_L_1].vel;

        BalanceTool_PrintRaw("\r\n[balance]\r\n");

        BalanceTool_PrintFloat4Line("L0",
                                    g_balance_robot.leg[0].rod.l0,
                                    "dL0",
                                    g_balance_robot.leg[0].rod.dl0);

        BalanceTool_PrintFloat4Line("phi0_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].rod.phi0),
                                    "dphi0_dps",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].rod.dphi0));

        BalanceTool_PrintFloat4Line("phi1_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].joint.phi1),
                                    "phi4_deg",
                                    BalanceTool_RadToDeg(g_balance_robot.leg[0].joint.phi4));

        BalanceTool_PrintFloat4Line("q0_cont",
                                    q0,
                                    "q1_cont",
                                    q1);

        BalanceTool_PrintFloat4Line("q0_err",
                                    g_balance_ref_pose.target_cont[0] - q0,
                                    "q1_err",
                                    g_balance_ref_pose.target_cont[1] - q1);

        BalanceTool_PrintFloat4Line("dq0",
                                    dq0,
                                    "dq1",
                                    dq1);

        BalanceTool_PrintFloat4Line("mode",
                                    (g_balance_mode == BAL_APP_MODE_RETURN_REF) ? 0.0f : 1.0f,
                                    "ref_done",
                                    BalanceRefPose_IsFinished(&g_balance_ref_pose) ? 1.0f : 0.0f);

        BalanceTool_PrintFloat4Line("ref_active",
                                    BalanceRefPose_IsActive(&g_balance_ref_pose) ? 1.0f : 0.0f,
                                    "stable",
                                    (float)g_balance_ref_pose.stable_count);

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