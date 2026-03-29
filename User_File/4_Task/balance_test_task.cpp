// #include "balance_test_task.h"

// #include "FreeRTOS.h"
// #include "task.h"
// #include "dvc_motor_dm.h"

// // 如果你的工程里还需要CAN句柄头文件、定时器头文件、BSP头文件，
// // 在这里继续加 include。
// // 例如：
// // #include "can.h"
// // #include "tim.h"

// // =========================
// // 单电机测试对象
// // 现在直接在本文件内创建
// // =========================
// Class_Motor_DM_Normal motor_test_dm;
// Class_Motor_DM_Normal motor_test_dm1;

// // 任务句柄
// static TaskHandle_t xBalanceTestTaskHandle = NULL;

// // 测试状态
// static bool g_balance_test_enable = false;
// static bool g_balance_test_inited = false;

// // 目标 MIT 参数
// static float g_target_pos = 0.0f;
// static float g_target_vel = 1.0f;   // 第一版先小一点
// static float g_target_tor = 0.0f;
// static float g_target_kp  = 0.0f;
// static float g_target_kd  = 0.5f;

// volatile uint32_t fdcan_err_code = 0;
// volatile uint32_t fdcan_ir = 0;
// volatile uint32_t fdcan_psr = 0;
// volatile uint32_t fdcan_ecr = 0;

// void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
// {
//     fdcan_err_code = hfdcan->ErrorCode;
//     fdcan_ir = hfdcan->Instance->IR;
//     fdcan_psr = hfdcan->Instance->PSR;
//     fdcan_ecr = hfdcan->Instance->ECR;
// }

// // 内部限幅函数
// static inline float Balance_Test_Clamp(float x, float min_v, float max_v)
// {
//     if (x < min_v) return min_v;
//     if (x > max_v) return max_v;
//     return x;
// }

// void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
// {
//     switch (Header.Identifier)
//     {
//     case (0x00): // MasterID
//         motor_test_dm.CAN_RxCpltCallback();
//         break;
//     case (0x11): // MasterID
//         motor_test_dm1.CAN_RxCpltCallback();
//         break;
//     }
// }

// /**
//  * @brief 电机初始化
//  * @note 这里需要你根据自己 Class_Motor_DM_Normal 的真实初始化接口补全
//  */
// void Balance_Test_Motor_Init(void)
// {
//     if (g_balance_test_inited)
//     {
//         return;
//     }

//     // ==========================================================
//     // 你需要在这里填入你自己的电机初始化代码
//     //
//     // 下面这段是“示意写法”，不是最终一定能编译通过的写法
//     // 因为我还没看到你这个类的完整 Init 参数表
//     //
//     // 你需要做的事本质上是：
//     // 1. 绑定 CAN
//     // 2. 设置电机 ID
//     // 3. 指定达妙型号（8009）
//     // 4. 选择 MIT 模式
//     // 5. 完成对象初始化
//     //
//     // 比如可能类似这种风格（示意）：
//     //
//     // motor_test_dm.Init(&hcan1,
//     //                    0x01,
//     //                    DM_Motor_J8009_2EC,
//     //                    Motor_DM_Control_Status_MIT);
//     //
//     // 或者如果你们的类是配置结构体初始化，也在这里填进去。
//     // ==========================================================
    
//     CAN_Init(&hfdcan1, CAN1_Callback);                              // 电机的CAN
    
//     motor_test_dm.Init(&hfdcan1, 0x00, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);
    
//     // 初始化后，先清零控制量
//     motor_test_dm.Set_Control_Angle(0.0f);
//     motor_test_dm.Set_Control_Omega(0.0f);
//     motor_test_dm.Set_Control_Torque(0.0f);
//     motor_test_dm.Set_K_P(0.0f);
//     motor_test_dm.Set_K_D(0.0f);

//     motor_test_dm1.Init(&hfdcan1, 0x11, 0x10, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);
    
//     // 初始化后，先清零控制量
//     motor_test_dm1.Set_Control_Angle(0.0f);
//     motor_test_dm1.Set_Control_Omega(0.0f);
//     motor_test_dm1.Set_Control_Torque(0.0f);
//     motor_test_dm1.Set_K_P(0.0f);
//     motor_test_dm1.Set_K_D(0.0f);

//     g_balance_test_inited = true;
// }

// /**
//  * @brief 内部函数：下发测试控制量
//  */
// static void Balance_Test_Motor_Apply(void)
// {
//     if (!g_balance_test_enable)
//     {
//         motor_test_dm.Set_Control_Angle(0.0f);
//         motor_test_dm.Set_Control_Omega(0.0f);
//         motor_test_dm.Set_Control_Torque(0.0f);
//         motor_test_dm.Set_K_P(0.0f);
//         motor_test_dm.Set_K_D(0.0f);

//         motor_test_dm1.Set_Control_Angle(0.0f);
//         motor_test_dm1.Set_Control_Omega(0.0f);
//         motor_test_dm1.Set_Control_Torque(0.0f);
//         motor_test_dm1.Set_K_P(0.0f);
//         motor_test_dm1.Set_K_D(0.0f);
//         return;
//     }

//     // 这里先给保守一点的限幅
//     const float cmd_vel = Balance_Test_Clamp(g_target_vel, -3.0f, 3.0f);
//     const float cmd_kp  = Balance_Test_Clamp(g_target_kp,  0.0f, 100.0f);
//     const float cmd_kd  = Balance_Test_Clamp(g_target_kd,  0.0f, 5.0f);
//     const float cmd_tor = Balance_Test_Clamp(g_target_tor, -5.0f, 5.0f);

//     motor_test_dm.Set_Control_Angle(g_target_pos);
//     motor_test_dm.Set_Control_Omega(cmd_vel);
//     motor_test_dm.Set_Control_Torque(cmd_tor);
//     motor_test_dm.Set_K_P(cmd_kp);
//     motor_test_dm.Set_K_D(cmd_kd);

//     motor_test_dm1.Set_Control_Angle(g_target_pos);
//     motor_test_dm1.Set_Control_Omega(cmd_vel);
//     motor_test_dm1.Set_Control_Torque(cmd_tor);
//     motor_test_dm1.Set_K_P(cmd_kp);
//     motor_test_dm1.Set_K_D(cmd_kd);
    

//     motor_test_dm1.TIM_Send_PeriodElapsedCallback();
//     motor_test_dm.TIM_Send_PeriodElapsedCallback();
// }

// /**
//  * @brief 启动单电机匀速测试
//  * @param target_vel 目标角速度(rad/s)
//  */
// void Balance_Test_Motor_Start(float target_kp,float target_kd,float target_pose,float target_vel)
// {
//     if (!g_balance_test_inited)
//     {
//         Balance_Test_Motor_Init();
//     }

//     g_target_vel = target_vel;
//     g_target_pos = target_pose;
//     g_target_kp = target_kp;
//     g_target_kd = target_kd;


//     g_balance_test_enable = true;

//     motor_test_dm.CAN_Send_Enter();
//     motor_test_dm1.CAN_Send_Enter();
// }

// /**
//  * @brief 停止单电机测试
//  */
// void Balance_Test_Motor_Stop(void)
// {
//     g_balance_test_enable = false;

//     motor_test_dm.Set_Control_Angle(0.0f);
//     motor_test_dm.Set_Control_Omega(0.0f);
//     motor_test_dm.Set_Control_Torque(0.0f);
//     motor_test_dm.Set_K_P(0.0f);
//     motor_test_dm.Set_K_D(0.0f);

//     motor_test_dm.CAN_Send_Exit();
// }

// /**
//  * @brief 修改目标速度
//  * @param target_vel 目标角速度(rad/s)
//  */
// void Balance_Test_Motor_SetVelocity(float target_vel)
// {
//     g_target_vel = target_vel;
// }

// /**
//  * @brief 单电机匀速测试任务
//  * @param pvParameters 未使用
//  */
// static void vBalanceTestTask(void *pvParameters)
// {
//     (void)pvParameters;

//     // 稍等系统启动稳定
//     vTaskDelay(pdMS_TO_TICKS(500));

//     // 初始化测试电机
//     Balance_Test_Motor_Init();

//     // 自动启动，先给小速度
//     Balance_Test_Motor_Start(1.0f,0.5f,0.0f,0.0f);
    
//     uint32_t offline_retry_cnt = 0;
    
//     while (1)
//     {
//         // 下发控制指令
//         Balance_Test_Motor_Apply();

//         // 获取电机状态（需要确保 Class_Motor_DM_Normal 提供了 Get_Status() 方法）
//         auto status = motor_test_dm.Get_Status();
//         auto status1 = motor_test_dm1.Get_Status();
//         // 如果电机离线或错误，尝试重新使能

//             // 避免过于频繁发送使能命令（每 20ms 尝试一次）
//             if (offline_retry_cnt++ >= 100)   // 2ms * 10 = 20ms
//             {
//                 offline_retry_cnt = 0;

//                 // 重新发送使能命令
//                 motor_test_dm.TIM_100ms_Alive_PeriodElapsedCallback();
//                 motor_test_dm1.TIM_100ms_Alive_PeriodElapsedCallback();
//             }

//         // 调试用变量（可保留或删除）
//         volatile float now_angle  = motor_test_dm.Get_Now_Angle();
//         volatile float now_omega  = motor_test_dm.Get_Now_Omega();
//         volatile float now_torque = motor_test_dm.Get_Now_Torque();
//         volatile auto  now_status = motor_test_dm.Get_Status();
//         (void)now_angle;
//         (void)now_omega;
//         (void)now_torque;
//         (void)now_status;

//         vTaskDelay(pdMS_TO_TICKS(2));
//     }
// }



// /**
//  * @brief 创建单电机匀速测试任务
//  */
// void Balance_Test_Task_Create(void)
// {
//     SYS_Timestamp.Init(&htim5);                                     // 初始化时间戳
//     HAL_TIM_Base_Start_IT(&htim5);
    
//     BaseType_t ret = xTaskCreate(vBalanceTestTask,
//                                  "vBalanceTestTask",
//                                  256,
//                                  NULL,
//                                  1,
//                                  &xBalanceTestTaskHandle);

//     if (ret != pdPASS)
//     {
//         while (1)
//         {
//         }
//     }
// }
