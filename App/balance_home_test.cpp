#include "balance_home_test.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "balance_config.h"
#include "balance_motor_if.h"
#include "balance_observer.h"
#include "balance_types.h"

#include "drv_can.h"
#include "usart.h"

#include "dvc_motor_dm.h"

namespace
{
    constexpr uint16_t kHomeDoneStableCountTh = 20U;

    BalanceRobot g_home_test_robot;

    Class_Motor_DM_Normal g_home_test_joint_0;
    Class_Motor_DM_Normal g_home_test_joint_1;

    TaskHandle_t xBalanceHomeTestCtrlTaskHandle = NULL;
    TaskHandle_t xBalanceHomeTestSendTaskHandle = NULL;
    TaskHandle_t xBalanceHomeTestAliveTaskHandle = NULL;
    TaskHandle_t xBalanceHomeTestPrintTaskHandle = NULL;

    bool g_home_test_inited = false;
    bool g_home_test_enable = false;
    bool g_home_test_done = false;
    bool g_home_feedback_ready = false;

    float g_home_kp = 5.0f;
    float g_home_kd = 2.0f;
    float g_home_pos_tol = 0.05f;
    float g_home_vel_tol = 0.20f;
    float g_home_ff_tor = 0.12f;

    float g_home_err0 = 0.0f;
    float g_home_err1 = 0.0f;
    uint16_t g_home_done_stable_count = 0U;

    inline float Clamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }

    void ClearMotorCmd(BalanceMotorCmd* cmd)
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

    void ClearAllMotorCmd(BalanceRobot* robot)
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

    void ResetHomeState(void)
    {
        g_home_test_done = false;
        g_home_feedback_ready = false;
        g_home_err0 = 0.0f;
        g_home_err1 = 0.0f;
        g_home_done_stable_count = 0U;
    }

    void PrintUart7(const char* text)
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

    int AppendFloat4(char* buf, size_t buf_size, const char* name, float value)
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
            return snprintf(buf, buf_size, "%s=%c%ld.%04ld", name, sign, integer_part, fraction_part);
        }

        return snprintf(buf, buf_size, "%s=%ld.%04ld", name, integer_part, fraction_part);
    }

    inline float WrapToMotorRange(float x)
    {
        while (x > 12.5f) x -= 25.0f;
        while (x < -12.5f) x += 25.0f;
        return x;
    }

    float ContTargetToRawTarget(const BalanceAngleUnwrap* unwrap, float target_cont)
    {
        if ((unwrap == nullptr) || (!unwrap->initialized))
        {
            return WrapToMotorRange(target_cont);
        }

        float delta_raw = target_cont - unwrap->continuous;
        while (delta_raw > 12.5f) delta_raw -= 25.0f;
        while (delta_raw < -12.5f) delta_raw += 25.0f;

        return WrapToMotorRange(unwrap->raw_last + delta_raw);
    }

    float CalcHomeAssistTorque(float err)
    {
        if (fabsf(err) < g_home_pos_tol)
        {
            return 0.0f;
        }

        return (err > 0.0f) ? g_home_ff_tor : -g_home_ff_tor;
    }

    void UpdateHomeState(void)
    {
        g_home_feedback_ready =
            g_home_test_robot.joint_angle_unwrap[BAL_JOINT_L_0].initialized &&
            g_home_test_robot.joint_angle_unwrap[BAL_JOINT_L_1].initialized &&
            g_home_test_robot.joint_motor_fdb[BAL_JOINT_L_0].online &&
            g_home_test_robot.joint_motor_fdb[BAL_JOINT_L_1].online;

        g_home_err0 =
            g_home_test_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous - BALANCE_JOINT_L0_CONT_REF;
        g_home_err1 =
            g_home_test_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous - BALANCE_JOINT_L1_CONT_REF;

        const bool joint0_ok =
            g_home_feedback_ready &&
            (fabsf(g_home_err0) < g_home_pos_tol) &&
            (fabsf(g_home_test_robot.joint_motor_fdb[BAL_JOINT_L_0].vel) < g_home_vel_tol);
        const bool joint1_ok =
            g_home_feedback_ready &&
            (fabsf(g_home_err1) < g_home_pos_tol) &&
            (fabsf(g_home_test_robot.joint_motor_fdb[BAL_JOINT_L_1].vel) < g_home_vel_tol);

        if (joint0_ok && joint1_ok)
        {
            if (g_home_done_stable_count < kHomeDoneStableCountTh)
            {
                ++g_home_done_stable_count;
            }
        }
        else
        {
            g_home_done_stable_count = 0U;
        }

        g_home_test_done = (g_home_done_stable_count >= kHomeDoneStableCountTh);
    }

    void ApplyHomeCmd(uint8_t joint_idx, float target_cont, float err)
    {
        BalanceMotorCmd& cmd = g_home_test_robot.joint_motor_cmd[joint_idx];
        cmd.pos = ContTargetToRawTarget(&g_home_test_robot.joint_angle_unwrap[joint_idx], target_cont);
        cmd.vel = 0.0f;
        cmd.tor = CalcHomeAssistTorque(-err);
        cmd.kp = Clamp(g_home_kp, 0.0f, BALANCE_DEFAULT_JOINT_KP_LIMIT);
        cmd.kd = Clamp(g_home_kd, 0.0f, BALANCE_DEFAULT_JOINT_KD_LIMIT);
        cmd.enable = true;
    }

    void BalanceHomeTest_InitRobot(void)
    {
        memset(&g_home_test_robot, 0, sizeof(g_home_test_robot));

        g_home_test_robot.dt = BALANCE_CTRL_DT;
        g_home_test_robot.enable = false;
        g_home_test_robot.safe = true;

        BalanceObserver_Init(&g_home_test_robot);
        ClearAllMotorCmd(&g_home_test_robot);
        ResetHomeState();
    }

    void BalanceHomeTest_CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
    {
        (void)Buffer;

        switch (Header.Identifier)
        {
        case (0x02):
            g_home_test_joint_0.CAN_RxCpltCallback();
            break;
        case (0x03):
            g_home_test_joint_1.CAN_RxCpltCallback();
            break;
        default:
            break;
        }
    }

    void BalanceHomeTest_InitMotors(void)
    {
        CAN_Init(&hfdcan1, BalanceHomeTest_CAN1_Callback);
        g_home_test_joint_0.Init(&hfdcan1, 0x02, 0x02,
                                 Motor_DM_Control_Method_NORMAL_MIT,
                                 12.5f, 25.0f, 10.0f, 10.261194f);
        g_home_test_joint_1.Init(&hfdcan1, 0x03, 0x03,
                                 Motor_DM_Control_Method_NORMAL_MIT,
                                 12.5f, 25.0f, 10.0f, 10.261194f);

        BalanceMotorIf_Init();
        BalanceMotorIf_RegisterJoint(BAL_JOINT_L_0, &g_home_test_joint_0);
        BalanceMotorIf_RegisterJoint(BAL_JOINT_L_1, &g_home_test_joint_1);
    }

    void BalanceHomeTest_Init(void)
    {
        if (g_home_test_inited)
        {
            return;
        }

        BalanceHomeTest_InitRobot();
        BalanceHomeTest_InitMotors();
        g_home_test_inited = true;
    }

    void BalanceHomeTest_RunController(void)
    {
        ClearAllMotorCmd(&g_home_test_robot);

        ApplyHomeCmd(BAL_JOINT_L_0, BALANCE_JOINT_L0_CONT_REF, g_home_err0);
        ApplyHomeCmd(BAL_JOINT_L_1, BALANCE_JOINT_L1_CONT_REF, g_home_err1);
    }

    void vBalanceHomeTestCtrlTask(void *pvParameters)
    {
        (void)pvParameters;

        vTaskDelay(pdMS_TO_TICKS(500));

        if (!g_home_test_inited)
        {
            BalanceHomeTest_Init();
        }

        BalanceHomeTest_Enable();

        for (;;)
        {
            BalanceMotorIf_UpdateFeedback(&g_home_test_robot);

            g_home_test_robot.body.phi = 0.0f;
            g_home_test_robot.body.phi_dot = 0.0f;
            BalanceObserver_UpdateLeg(&g_home_test_robot);
            UpdateHomeState();

            if (g_home_test_enable && !g_home_test_done)
            {
                BalanceHomeTest_RunController();
            }
            else
            {
                ClearAllMotorCmd(&g_home_test_robot);
            }

            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    void vBalanceHomeTestSendTask(void *pvParameters)
    {
        (void)pvParameters;

        vTaskDelay(pdMS_TO_TICKS(600));

        for (;;)
        {
            BalanceMotorIf_SendCommand(&g_home_test_robot);
            BalanceMotorIf_TxAllPeriodic();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    void vBalanceHomeTestAliveTask(void *pvParameters)
    {
        (void)pvParameters;

        vTaskDelay(pdMS_TO_TICKS(1000));

        for (;;)
        {
            BalanceMotorIf_AliveAllPeriodic();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    void vBalanceHomeTestPrintTask(void *pvParameters)
    {
        (void)pvParameters;

        vTaskDelay(pdMS_TO_TICKS(1200));

        char tx_buf[128] = {0};

        for (;;)
        {
            int written = snprintf(tx_buf,
                                   sizeof(tx_buf),
                                   "home done=%d ready=%d stable=%u/%u\r\n",
                                   g_home_test_done ? 1 : 0,
                                   g_home_feedback_ready ? 1 : 0,
                                   static_cast<unsigned int>(g_home_done_stable_count),
                                   static_cast<unsigned int>(kHomeDoneStableCountTh));
            if (written > 0)
            {
                PrintUart7(tx_buf);
            }

            size_t used = 0U;
            written = snprintf(tx_buf, sizeof(tx_buf), "");
            used = (written >= 0) ? static_cast<size_t>(written) : 0U;
            if (used < sizeof(tx_buf))
            {
                written = AppendFloat4(tx_buf + used, sizeof(tx_buf) - used,
                                       "c0",
                                       g_home_test_robot.joint_angle_unwrap[BAL_JOINT_L_0].continuous);
                if (written > 0) used += static_cast<size_t>(written);
            }
            if (used < sizeof(tx_buf))
            {
                written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
                if (written > 0) used += static_cast<size_t>(written);
            }
            if (used < sizeof(tx_buf))
            {
                written = AppendFloat4(tx_buf + used, sizeof(tx_buf) - used,
                                       "c1",
                                       g_home_test_robot.joint_angle_unwrap[BAL_JOINT_L_1].continuous);
                if (written > 0) used += static_cast<size_t>(written);
            }
            if (used < sizeof(tx_buf))
            {
                snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
                PrintUart7(tx_buf);
            }

            written = snprintf(tx_buf, sizeof(tx_buf), "");
            used = (written >= 0) ? static_cast<size_t>(written) : 0U;
            if (used < sizeof(tx_buf))
            {
                written = AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "e0", g_home_err0);
                if (written > 0) used += static_cast<size_t>(written);
            }
            if (used < sizeof(tx_buf))
            {
                written = snprintf(tx_buf + used, sizeof(tx_buf) - used, ", ");
                if (written > 0) used += static_cast<size_t>(written);
            }
            if (used < sizeof(tx_buf))
            {
                written = AppendFloat4(tx_buf + used, sizeof(tx_buf) - used, "e1", g_home_err1);
                if (written > 0) used += static_cast<size_t>(written);
            }
            if (used < sizeof(tx_buf))
            {
                snprintf(tx_buf + used, sizeof(tx_buf) - used, "\r\n");
                PrintUart7(tx_buf);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void BalanceHomeTest_Enable(void)
{
    if (!g_home_test_inited)
    {
        BalanceHomeTest_Init();
    }

    g_home_test_enable = true;
    ResetHomeState();
    BalanceMotorIf_SendEnterAll();
}

void BalanceHomeTest_Disable(void)
{
    g_home_test_enable = false;
    ResetHomeState();

    ClearAllMotorCmd(&g_home_test_robot);
    BalanceMotorIf_SendCommand(&g_home_test_robot);
    BalanceMotorIf_SendExitAll();
}

bool BalanceHomeTest_IsHomeDone(void)
{
    return g_home_test_done;
}

void BalanceHomeTest_Task_Create(void)
{
    if (!g_home_test_inited)
    {
        BalanceHomeTest_Init();
    }

    BaseType_t ret;

    ret = xTaskCreate(vBalanceHomeTestCtrlTask,
                      "vHomeTestCtrlTask",
                      512,
                      NULL,
                      3,
                      &xBalanceHomeTestCtrlTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceHomeTestSendTask,
                      "vHomeTestSendTask",
                      384,
                      NULL,
                      4,
                      &xBalanceHomeTestSendTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceHomeTestAliveTask,
                      "vHomeTestAliveTask",
                      256,
                      NULL,
                      2,
                      &xBalanceHomeTestAliveTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }

    ret = xTaskCreate(vBalanceHomeTestPrintTask,
                      "vHomeTestPrintTask",
                      384,
                      NULL,
                      1,
                      &xBalanceHomeTestPrintTaskHandle);
    if (ret != pdPASS)
    {
        while (1) {}
    }
}
