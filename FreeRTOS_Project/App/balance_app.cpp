#include "balance_app.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "balance_config.h"
#include "balance_motor_if.h"
#include "balance_observer.h"
#include "balance_controller.h"

#include "drv_can.h"
#include "usart.h"

// =====================================================
// 全局机器人对象
// 即使当前只是单电机演示，也继续沿用这套统一数据结构，
// 这样后面切回整机控制时不需要推倒重来。
// =====================================================
BalanceRobot g_balance_robot;

// =====================================================
// 电机对象
// 当前版本只启用 g_motor_joint_0。
// 其他对象先保留，后续扩展多电机时可以直接继续用。
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
static TaskHandle_t xBalanceDebugTaskHandle = NULL;

// =====================================================
// 运行标志
// =====================================================
static bool g_balance_app_inited = false;                           // 此代码初始化
static bool g_balance_app_enabled = false;                          // 此代码使能

namespace
{
// =====================================================
// 单电机演示参数
// 你当前的需求是：先在 balance_app 中只让一个电机转起来。
// 这里选用 BAL_JOINT_L_0 对应的 g_motor_joint_0。
// =====================================================
static constexpr uint8_t k_demo_motor_rx_id = 0x01;
static constexpr uint8_t k_demo_motor_tx_id = 0x01;
static constexpr float k_demo_motor_target_vel = 1.5f;              // rad/s
static constexpr float k_demo_motor_target_kp = 0.0f;
static constexpr float k_demo_motor_target_kd = 0.7f;
static constexpr float k_demo_motor_target_tor = 0.0f;
static constexpr uint32_t k_demo_forward_ms = 10000U;
static constexpr uint32_t k_demo_reverse_ms = 10000U;
static constexpr uint32_t k_demo_pause_ms = 250U;

static int32_t BalanceApp_FloatToMilli(const float value)
{
    if (value >= 0.0f)
    {
        return static_cast<int32_t>(value * 1000.0f + 0.5f);
    }

    return static_cast<int32_t>(value * 1000.0f - 0.5f);
}

static float BalanceApp_GetDemoTargetVel(void)
{
    const uint32_t cycle_ms = k_demo_forward_ms + k_demo_pause_ms +
                              k_demo_reverse_ms + k_demo_pause_ms;
    const uint32_t now_ms = static_cast<uint32_t>(xTaskGetTickCount()) *
                            static_cast<uint32_t>(portTICK_PERIOD_MS);
    const uint32_t phase_ms = now_ms % cycle_ms;

    if (phase_ms < k_demo_forward_ms)
    {
        return k_demo_motor_target_vel;
    }

    if (phase_ms < (k_demo_forward_ms + k_demo_pause_ms))
    {
        return 0.0f;
    }

    if (phase_ms < (k_demo_forward_ms + k_demo_pause_ms + k_demo_reverse_ms))
    {
        return -k_demo_motor_target_vel;
    }

    return 0.0f;
}

static void BalanceApp_ClearAllMotorCmd(void)
{
    for (int i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        g_balance_robot.joint_motor_cmd[i].pos = 0.0f;
        g_balance_robot.joint_motor_cmd[i].vel = 0.0f;
        g_balance_robot.joint_motor_cmd[i].tor = 0.0f;
        g_balance_robot.joint_motor_cmd[i].kp = 0.0f;
        g_balance_robot.joint_motor_cmd[i].kd = 0.0f;
        g_balance_robot.joint_motor_cmd[i].enable = false;
    }

    for (int i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        g_balance_robot.wheel_motor_cmd[i].pos = 0.0f;
        g_balance_robot.wheel_motor_cmd[i].vel = 0.0f;
        g_balance_robot.wheel_motor_cmd[i].tor = 0.0f;
        g_balance_robot.wheel_motor_cmd[i].kp = 0.0f;
        g_balance_robot.wheel_motor_cmd[i].kd = 0.0f;
        g_balance_robot.wheel_motor_cmd[i].enable = false;
    }
}

static void BalanceApp_SetSingleMotorDemoCmd(void)
{
    // 每个周期都先清空全部命令，保证当前只有 1 个电机会被驱动。
    BalanceApp_ClearAllMotorCmd();

    // 用最简单的 MIT 命令驱动单电机转动：
    // pos = 0
    // vel = 固定目标速度
    // tor = 0
    // kp = 0
    // kd = 小阻尼
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].pos = 0.0f;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].vel = BalanceApp_GetDemoTargetVel();
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].tor = k_demo_motor_target_tor;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].kp = k_demo_motor_target_kp;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].kd = k_demo_motor_target_kd;
    g_balance_robot.joint_motor_cmd[BAL_JOINT_L_0].enable = true;
}

static void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    // 当前演示版只有 1 个电机，所以只转发给这 1 个对象。
    // 以后要扩展多电机时，在这里按 Header.Identifier 分发即可。
    if (Header.Identifier == k_demo_motor_rx_id)
    {
        g_motor_joint_0.CAN_RxCpltCallback();
    }
}
}

// =====================================================
// 内部函数声明
// =====================================================
static void BalanceApp_InitRobot(void);
static void BalanceApp_InitMotors(void);
static void BalanceApp_RegisterMotors(void);

static void vBalanceControlTask(void *pvParameters);
static void vBalanceMotorSendTask(void *pvParameters);
static void vBalanceAliveTask(void *pvParameters);
static void vBalanceDebugTask(void *pvParameters);

// =====================================================
// 初始化机器人对象
// 虽然当前不跑 observer / controller，
// 但把基础状态初始化完整，后面切回整机控制会更顺。
// =====================================================
static void BalanceApp_InitRobot(void)
{
    memset(&g_balance_robot, 0, sizeof(g_balance_robot));

    g_balance_robot.dt = BALANCE_CTRL_DT;
    g_balance_robot.enable = false;
    g_balance_robot.safe = true;

    BalanceObserver_Init(&g_balance_robot);                             // 初始化LQR的状态空间向量
    BalanceController_Init(&g_balance_robot);                           // 初始化电机还有腿
    BalanceApp_ClearAllMotorCmd();                                      // 初始化机身的
}

// =====================================================
// 初始化电机
// 当前只初始化 1 个演示电机，完全参照你 balance_test_task 中的写法。
// =====================================================
static void BalanceApp_InitMotors(void)
{
    BalanceMotorIf_Init();                                              // 将所有电机指针置为nullptr和取消注册

    CAN_Init(&hfdcan1, CAN1_Callback);

    g_motor_joint_0.Init(&hfdcan1,
                         k_demo_motor_rx_id,
                         k_demo_motor_tx_id,
                         Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f,
                         25.0f,
                         10.0f,
                         10.261194f);

    // 上电后先清零控制量，避免对象里残留旧命令。
    g_motor_joint_0.Set_Control_Angle(0.0f);
    g_motor_joint_0.Set_Control_Omega(0.0f);
    g_motor_joint_0.Set_Control_Torque(0.0f);
    g_motor_joint_0.Set_K_P(0.0f);
    g_motor_joint_0.Set_K_D(0.0f);
}

// =====================================================
// 注册电机
// 这里只注册已经真实 Init 过的 g_motor_joint_0。
// 不能把剩余 5 个未初始化对象一起注册，否则后续会误发命令。
// =====================================================
static void BalanceApp_RegisterMotors(void)
{
    BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_motor_joint_0);
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

    BalanceApp_InitRobot();                                 // 初始化机身
    BalanceApp_InitMotors();                                // 初始化电机
    BalanceApp_RegisterMotors();                            // 注册电机

    g_balance_app_inited = true;                            // 初始化完毕
}

// =====================================================
// 对外使能
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
}

// =====================================================
// 对外停机
// =====================================================
void BalanceApp_Disable(void)
{
    g_balance_app_enabled = false;
    g_balance_robot.enable = false;
    g_balance_robot.safe = true;

    BalanceApp_ClearAllMotorCmd();
    BalanceMotorIf_SendCommand(&g_balance_robot);
    BalanceMotorIf_SendExitAll();
}

// =====================================================
// 控制主任务
// 当前版本不做 IMU / observer / controller，
// 只在这里周期性生成“单电机匀速转动”的命令。
// =====================================================
static void vBalanceControlTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));

    if (!g_balance_app_inited)
    {
        BalanceApp_Init();
    }

    // 为了方便联调，这里直接自动使能。
    BalanceApp_Enable();

    for (;;)
    {
        BalanceMotorIf_UpdateFeedback(&g_balance_robot);                    // 将电机返回的数据保存
        BalanceObserver_UpdateLeg(&g_balance_robot);

        if (g_balance_robot.enable && !g_balance_robot.safe)
        {
            BalanceApp_SetSingleMotorDemoCmd();                             // 给电机设置控制指令
        }
        else
        {
            BalanceApp_ClearAllMotorCmd();
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =====================================================
// 电机发送任务
// 2ms 周期：
// 1. 把 g_balance_robot 中的命令写入电机对象
// 2. 触发达妙电机周期发送
// =====================================================
static void vBalanceMotorSendTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(600));

    for (;;)
    {
        BalanceMotorIf_SendCommand(&g_balance_robot);                       // 将机器人控制指令下发给电机
        BalanceMotorIf_TxAllPeriodic();                                     // 正式向电机发指令

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =====================================================
// 保活任务
// 100ms 周期检查在线状态，如果掉线则由驱动内部尝试重新使能。
// =====================================================
static void vBalanceAliveTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;)
    {
        BalanceMotorIf_AliveAllPeriodic();                                  // 检测电机是否掉线
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =====================================================
// 串口调试任务
// 通过 UART7 周期打印电机反馈，方便观察回传数据
// =====================================================
static void vBalanceDebugTask(void *pvParameters)
{
    (void)pvParameters;

    static char uart7_tx_buf[160];

    vTaskDelay(pdMS_TO_TICKS(1200));

    for (;;)
    {
        const BalanceMotorFdb &fdb = g_balance_robot.joint_motor_fdb[BAL_JOINT_L_0];
        const int32_t raw_angle_milli = BalanceApp_FloatToMilli(fdb.pos);
        const int32_t cont_angle_milli = BalanceApp_FloatToMilli(g_balance_robot.leg[0].joint.phi1);
        const int32_t vel_milli = BalanceApp_FloatToMilli(fdb.vel);
        const int32_t tor_milli = BalanceApp_FloatToMilli(fdb.tor);
        const int len = snprintf(uart7_tx_buf,
                                 sizeof(uart7_tx_buf),
                                 "dm_fdb online=%d status=%d raw=%ld.%03ld cont=%ld.%03ld vel=%ld.%03ld tor=%ld.%03ld\r\n",
                                 static_cast<int>(fdb.online),
                                 static_cast<int>(g_motor_joint_0.Get_Status()),
                                 static_cast<long>(raw_angle_milli / 1000),
                                 static_cast<long>(labs(raw_angle_milli % 1000)),
                                 static_cast<long>(cont_angle_milli / 1000),
                                 static_cast<long>(labs(cont_angle_milli % 1000)),
                                 static_cast<long>(vel_milli / 1000),
                                 static_cast<long>(labs(vel_milli % 1000)),
                                 static_cast<long>(tor_milli / 1000),
                                 static_cast<long>(labs(tor_milli % 1000)));

        if (len > 0)
        {
            const uint16_t tx_len = (len < static_cast<int>(sizeof(uart7_tx_buf))) ?
                                    static_cast<uint16_t>(len) :
                                    static_cast<uint16_t>(sizeof(uart7_tx_buf) - 1U);
            HAL_UART_Transmit(&huart7, reinterpret_cast<uint8_t *>(uart7_tx_buf), tx_len, 20);
        }

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
    
    // 此任务用于
    // 获取电机返回的信息
    // 设置电机的cmd数组
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
    
    // 此任务用于
    // 将cmd数组的内容给电机设定
    // 给电机发送控制指令
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
    
    // 此任务用于
    // 检查电机是否离线
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

    ret = xTaskCreate(vBalanceDebugTask,
                      "vBalanceDebugTask",
                      512,
                      NULL,
                      1,
                      &xBalanceDebugTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }
}
