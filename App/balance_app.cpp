#include "balance_app.h"

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
// 内部函数声明
// =====================================================
static void BalanceApp_InitRobot(void);
static void BalanceApp_InitImu(void);
static void BalanceApp_InitMotors(void);

static void vBalanceControlTask(void *pvParameters);
static void vBalanceMotorSendTask(void *pvParameters);
static void vBalanceAliveTask(void *pvParameters);
static void vBalancePrintTask(void *pvParameters);

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
            BalanceController_SetRef(&g_balance_robot);
            BalanceController_LegLength(&g_balance_robot);
            BalanceController_LegAngle(&g_balance_robot);
            BalanceController_Output(&g_balance_robot);
        }
        else
        {
            BalanceController_Stop(&g_balance_robot);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

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

static void vBalancePrintTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1200));

    for (;;)
    {
        BalanceTool_PrintRaw("\r\n[balance]\r\n");

        BalanceTool_PrintFloat4Line("L0",        g_balance_robot.leg[0].rod.l0,         // 虚拟腿长度
                                    "dL0",       g_balance_robot.leg[0].rod.dl0);       // 虚拟腿长度变化率

        BalanceTool_PrintFloat4Line("phi0",      g_balance_robot.leg[0].rod.phi0,       // 虚拟腿角度
                                    "dphi0",     g_balance_robot.leg[0].rod.dphi0);     // 虚拟角速度

        BalanceTool_PrintFloat4Line("rod_f",     g_balance_robot.cmd[0].rod_f,          // 虚拟腿轴向力
                                    "rod_tp",    g_balance_robot.cmd[0].rod_tp);        // 虚拟腿转动力矩

        BalanceTool_PrintFloat4Line("pitch",     g_balance_robot.imu.pitch,             // 陀螺仪pitch角
                                    "pitch_dot", g_balance_robot.imu.pitch_dot);        // 陀螺仪pitch角速度

        BalanceTool_PrintFloat4Line("phi1",      BalanceTool_RadToDeg(g_balance_robot.leg[0].joint.phi1),
                                    "phi4",      BalanceTool_RadToDeg(g_balance_robot.leg[0].joint.phi4));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

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