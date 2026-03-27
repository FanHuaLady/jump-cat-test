#include "balance_app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "balance_config.h"
#include "balance_motor_if.h"
#include "balance_imu_if.h"
#include "balance_observer.h"
#include "balance_controller.h"

#include "fdcan.h"
#include "dvc_motor_dm.h"
#include "drv_can.h"
#include "bsp_jy61p.h"

// 如果你的 IMU 初始化需要 UART 句柄，在这里包含
// 例如：
// #include "usart.h"

// 如果你的 DM 电机初始化需要 CAN 句柄，在这里包含
// 例如：
// #include "can.h"

// =====================================================
// 全局机器人对象
// =====================================================
BalanceRobot g_balance_robot;

// =====================================================
// 这里直接创建 6 个电机对象
// 逻辑上：4 个关节 + 2 个轮
// 具体哪个对应哪条腿，后面由 Register 绑定
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

void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (0x02): // MasterID
        g_motor_joint_0.CAN_RxCpltCallback();
        break;
    case (0x03): // MasterID
        g_motor_joint_1.CAN_RxCpltCallback();
        break;
    }
}

// =====================================================
// 初始化机器人对象和各模块
// =====================================================
static void BalanceApp_InitRobot(void)
{
    // 整个对象先清零
    memset(&g_balance_robot, 0, sizeof(g_balance_robot));
    BalanceObserver_Init(&g_balance_robot);                         // 初始化数据收集加整合层
    BalanceController_Init(&g_balance_robot);                       // 初始化指令控制层
}

static void BalanceApp_InitImu(void)
{
    // =========================
    // 这里按你的工程实际补 IMU 初始化
    // 例如：
    // BSP_JY61P.Init(&huart10);
    // =========================

    BalanceImuIf_Init();
}

static void BalanceApp_InitMotors(void)
{
    CAN_Init(&hfdcan1, CAN1_Callback);                              // 电机的CAN
    g_motor_joint_0.Init(&hfdcan1, 0x02, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);
    g_motor_joint_1.Init(&hfdcan1, 0x03, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);

    BalanceMotorIf_Init();
    
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_motor_joint_0);
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_1, &g_motor_joint_1);
}

// =====================================================
// 对外初始化入口
// =====================================================
void BalanceApp_Init(void)
{
    if (g_balance_app_inited)
    {
        return;
    }

    BalanceApp_InitRobot();                             // 初始化机体
    BalanceApp_InitImu();                               // 初始化陀螺仪
    BalanceApp_InitMotors();                            // 初始化电机

    g_balance_app_inited = true;                        // 初始化完毕
}

// =====================================================
// 对外使能/停机
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

    BalanceMotorIf_SendEnterAll();                      // 使能所有注册的电机
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
// 控制主任务
// 2ms：读反馈 + IMU + observer + controller
// =====================================================
static void vBalanceControlTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    // 默认先不自动使能
    // 调试方便，你可以手动调用 BalanceApp_Enable()
    // 如果想自动开，取消下面注释：
    BalanceApp_Enable();

    for (;;)
    {
        // 1) 更新电机反馈
        BalanceMotorIf_UpdateFeedback(&g_balance_robot);
        
        // 2) 更新 IMU 统一数据
        BalanceImuIf_Update(&g_balance_robot.imu);

        // 3) observer：状态估计和状态拼装
        BalanceObserver_UpdateAll(&g_balance_robot);

        // 4) controller
        if (g_balance_robot.enable && !g_balance_robot.safe)
        {
            BalanceController_SetRef(&g_balance_robot);
            BalanceController_LegLength(&g_balance_robot);
            BalanceController_Output(&g_balance_robot);
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
// 2ms：把 cmd 写入对象 + 触发达妙周期发送
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
// 100ms：调用达妙在线检测/重连逻辑
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
// 创建任务
// =====================================================
void BalanceApp_Task_Create(void)
{
    if (!g_balance_app_inited)
    {
        BalanceApp_Init();                                      // 初始化
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
}
