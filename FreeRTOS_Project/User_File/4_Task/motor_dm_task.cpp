#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "motor_dm_task.h"
#include "dvc_motor_dm.h"

#include "alg_basic.h"

Class_Motor_DM_Normal motor_dm_x;   // 水平的电机
Class_Motor_DM_Normal motor_dm_y;   // 竖直的电机

// 任务句柄
static TaskHandle_t xMotorDMTaskHandle = NULL;


// 速度队列句柄
static QueueHandle_t xSpeedQueueYaw = NULL;
static QueueHandle_t xSpeedQueuePitch = NULL;

// 初始化队列
void Motor_DM_Init_Queue(void)
{
    // 队列长度为1，只保留最新速度
    xSpeedQueueYaw = xQueueCreate(1, sizeof(float));
    xSpeedQueuePitch = xQueueCreate(1, sizeof(float));
}

// 设置电机速度（由 set_angle_task 调用）
void Motor_DM_Set_Speed(uint8_t motor_id, float speed_rad_s)
{
    if (motor_id == MOTOR_X && xSpeedQueueYaw != NULL)
    {
        // 覆盖写入，只保留最新值
        xQueueOverwrite(xSpeedQueueYaw, &speed_rad_s);
    }
    else if (motor_id == MOTOR_Y && xSpeedQueuePitch != NULL)
    {
        xQueueOverwrite(xSpeedQueuePitch, &speed_rad_s);
    }
}

// 读取最新速度（内部使用，在 Motor_DM_Test_Apply 中调用）
static float Motor_DM_Get_Speed(uint8_t motor_id)
{
    float speed = 0.0f;
    
    if (motor_id == MOTOR_X && xSpeedQueueYaw != NULL)
    {
        // 非阻塞读取
        xQueuePeek(xSpeedQueueYaw, &speed, 0);
    }
    else if (motor_id == MOTOR_Y && xSpeedQueuePitch != NULL)
    {
        xQueuePeek(xSpeedQueuePitch, &speed, 0);
    }
    
    return speed;
}

void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    case (MOTOR_DM_X_MasterID): // 水平电机的MasterID
        motor_dm_x.CAN_RxCpltCallback();
        break;
    case (MOTOR_DM_Y_MasterID): // 竖直电机的MasterID
        motor_dm_y.CAN_RxCpltCallback();
        break;
    }
}

volatile uint32_t fdcan_err_code = 0;
volatile uint32_t fdcan_ir = 0;
volatile uint32_t fdcan_psr = 0;
volatile uint32_t fdcan_ecr = 0;

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    fdcan_err_code = hfdcan->ErrorCode;
    fdcan_ir = hfdcan->Instance->IR;
    fdcan_psr = hfdcan->Instance->PSR;
    fdcan_ecr = hfdcan->Instance->ECR;
}



// 初始化电机
void Motor_DM_Test_Init(void)
{
    CAN_Init(&hfdcan1, CAN1_Callback); 

    motor_dm_x.Init(&hfdcan1, MOTOR_DM_X_MasterID, MOTOR_DM_X_CANID, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);
    motor_dm_y.Init(&hfdcan1, MOTOR_DM_Y_MasterID, MOTOR_DM_Y_CANID, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f, 10.261194f);

    motor_dm_x.Set_Control_Angle(0.0f);
    motor_dm_x.Set_Control_Omega(0.0f);
    motor_dm_x.Set_Control_Torque(0.0f);
    motor_dm_x.Set_K_P(0.0f);
    motor_dm_x.Set_K_D(0.5f);

    motor_dm_y.Set_Control_Angle(0.0f);
    motor_dm_y.Set_Control_Omega(0.0f);
    motor_dm_y.Set_Control_Torque(0.0f);
    motor_dm_y.Set_K_P(0.0f);
    motor_dm_y.Set_K_D(0.5f);

    motor_dm_x.CAN_Send_Enter();
    motor_dm_y.CAN_Send_Enter();
   
}

void Motor_DM_Test_Apply(void)
{

        // 从队列读取最新速度
    static float g_target_vel_x = 0.0f;
    static float g_target_vel_y = 0.0f;
    
    static float g_target_kp_x  = 0.0f;     // kp值
    static float g_target_kd_x  = 0.5f;     // kd值
    static float g_target_pos_x = 0.0f;     // 设定的目标位置
    static float g_target_tor_x = 0.0f;     // 设定的前馈力矩

    static float g_target_kp_y  = 0.0f;
    static float g_target_kd_y  = 0.5f;
    static float g_target_pos_y = 0.0f;
    static float g_target_tor_y = 0.0f;

    // 从队列中读取最新目标速度
    g_target_vel_x = Motor_DM_Get_Speed(MOTOR_X);
    g_target_vel_y = Motor_DM_Get_Speed(MOTOR_Y);

    // 水平电机参数限幅
    const float cmd_kp_x  = Basic_Math_Constrain(g_target_kp_x,  MOTOR_X_KP_MIN,  MOTOR_X_KP_MAX);
    const float cmd_kd_x  = Basic_Math_Constrain(g_target_kd_x,  MOTOR_X_KD_MIN,  MOTOR_X_KD_MAX);
    const float cmd_pos_x = Basic_Math_Constrain(g_target_pos_x, MOTOR_X_POS_MIN, MOTOR_X_POS_MAX);
    const float cmd_vel_x = Basic_Math_Constrain(g_target_vel_x, MOTOR_X_VEL_MIN, MOTOR_X_VEL_MAX);
    const float cmd_tor_x = Basic_Math_Constrain(g_target_tor_x, MOTOR_X_TOR_MIN, MOTOR_X_TOR_MAX);

    // 竖直电机参数限幅
    const float cmd_kp_y  = Basic_Math_Constrain(g_target_kp_y,  MOTOR_Y_KP_MIN,  MOTOR_Y_KP_MAX);
    const float cmd_kd_y  = Basic_Math_Constrain(g_target_kd_y,  MOTOR_Y_KD_MIN,  MOTOR_Y_KD_MAX);
    const float cmd_pos_y = Basic_Math_Constrain(g_target_pos_y, MOTOR_Y_POS_MIN, MOTOR_Y_POS_MAX);
    const float cmd_vel_y = Basic_Math_Constrain(g_target_vel_y, MOTOR_Y_VEL_MIN, MOTOR_Y_VEL_MAX);
    const float cmd_tor_y = Basic_Math_Constrain(g_target_tor_y, MOTOR_Y_TOR_MIN, MOTOR_Y_TOR_MAX);

    motor_dm_x.Set_K_P(cmd_kp_x);
    motor_dm_x.Set_K_D(cmd_kd_x);
    motor_dm_x.Set_Control_Angle(cmd_pos_x);
    motor_dm_x.Set_Control_Omega(cmd_vel_x);
    motor_dm_x.Set_Control_Torque(cmd_tor_x);

    motor_dm_y.Set_K_P(cmd_kp_y);
    motor_dm_y.Set_K_D(cmd_kd_y);
    motor_dm_y.Set_Control_Angle(cmd_pos_y);
    motor_dm_y.Set_Control_Omega(cmd_vel_y);
    motor_dm_y.Set_Control_Torque(cmd_tor_y);

    motor_dm_x.TIM_Send_PeriodElapsedCallback();
    motor_dm_y.TIM_Send_PeriodElapsedCallback();





}




static void vMotorDMTask(void *pvParameters)
{
    (void)pvParameters;  // 消除未使用参数警告

    // 稍等系统启动稳定
    vTaskDelay(pdMS_TO_TICKS(500));
    Motor_DM_Test_Init();


    uint8_t offline_retry_cnt = 0;
    
    while (1)
    {
        // 下发控制指令
        Motor_DM_Test_Apply();

        // 获取电机状态
        auto status_x = motor_dm_x.Get_Status();
        auto status_y = motor_dm_y.Get_Status();

        // 判断电机是否在线（根据你的电机类实际的状态枚举值调整）
        bool is_online_x = (status_x == Motor_DM_Status_ENABLE);
        bool is_online_y = (status_y == Motor_DM_Status_ENABLE);
        
        // 如果任一电机离线，计数重试
        if (!is_online_x || !is_online_y)
        {
            offline_retry_cnt++;
            
            // 每 100 次循环（约 200ms）重试一次
            if (offline_retry_cnt >= 100)
            {
                offline_retry_cnt = 0;
                
                // 重新发送使能命令
                if (!is_online_x)
                {
                    motor_dm_x.TIM_100ms_Alive_PeriodElapsedCallback();
                }
                if (!is_online_y)
                {
                    motor_dm_y.TIM_100ms_Alive_PeriodElapsedCallback();
                }
            }
        }
        else
        {
            // 两个电机都在线，重置计数器
            offline_retry_cnt = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(2));  // 500Hz 控制频率
    }
}



void MOTOR_DM_Task_Create(void)
{
    Motor_DM_Init_Queue();
    BaseType_t ret = xTaskCreate(vMotorDMTask,          // 任务函数
                                 "vMotorDMTask",        // 任务名称
                                 512,                  // 栈大小（字）
                                 NULL,                 // 参数
                                 2,                    // 优先级（与Control相同）
                                 &xMotorDMTaskHandle);
    if (ret != pdPASS)
    {
        while (1)
        {
            // 任务创建失败，停止系统
        }
    }

}