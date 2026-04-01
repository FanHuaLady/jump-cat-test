// vofa_task.cpp

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "dvc_vofa.h"
#include "bsp_jy61p.h"

#include "vofa_task.h"
#include "jy61p_task.h"
#include "set_angle_test_task.h"
#include "motor_dm_task.h"

#include <string.h>
#include <stdio.h>

// ========== 外部变量声明 ==========
extern Class_Vofa_UART Vofa_UART;
extern QueueHandle_t xIMUDataQueue;
extern UART_HandleTypeDef huart7;

// ========== 外部函数声明（从 set_angle_test_task 引入）==========

// 使用VOFA调参的开始
extern float Get_Control_Enable(void);
extern void Set_Control_Enable(float enable);


extern void Set_Target_Yaw(float yaw_deg);
extern void Set_Target_Pitch(float pitch_deg);
extern void Set_Target_Roll(float roll_deg);

// 设置 PID 参数(角度误差转换为目标速度的参数)
extern void Set_Angle_PID_Kp_X(float kp);
extern void Set_Angle_PID_Ki_X(float ki);
extern void Set_Angle_PID_Kd_X(float kd);

extern void Set_Angle_PID_Kp_Y(float kp);
extern void Set_Angle_PID_Ki_Y(float ki);
extern void Set_Angle_PID_Kd_Y(float kd);

// vofa查看PID参数(角度误差转换为目标速度的参数)
extern float Test_Get_PID_Kp_X(void);
extern float Test_Get_PID_Ki_X(void);
extern float Test_Get_PID_Kd_X(void);

extern float Test_Get_PID_Kp_Y(void);
extern float Test_Get_PID_Ki_Y(void);
extern float Test_Get_PID_Kd_Y(void);

// VOFA显示接口（目标角度）
extern float Test_Get_Target_Yaw(void);
extern float Test_Get_Target_Pitch(void);
extern float Test_Get_Target_Roll(void);

// // VOFA显示接口（当前角度）
// extern float Test_Get_Current_Yaw(void);
// extern float Test_Get_Current_Pitch(void);
// extern float Test_Get_Current_Roll(void);

// VOFA显示接口（角度误差）
extern float Test_Get_Error_Yaw(void);
extern float Test_Get_Error_Pitch(void);
extern float Test_Get_Error_Roll(void);

// ========== 静态变量 ==========
static QueueHandle_t xVofaCommandQueue = NULL;

// 接收变量名列表
static char Vofa_Variable_List[][VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = 
{
    
    {"yaw"},
    {"pitch"},
    {"roll"},
    {"kp_x"},
    {"ki_x"},
    {"kd_x"},
    {"kp_y"},
    {"ki_y"},
    {"kd_y"},
    {"en"},

};

// ========== 静态函数声明 ==========
static void vVofaSendTask(void *pvParameters);
static void vVofaCmdTask(void *pvParameters);

// ========== VOFA发送任务 ==========
static void vVofaSendTask(void *pvParameters)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        
        AttitudeData_t imu_data;
        
        if (xQueuePeek(xIMUDataQueue, &imu_data, 0) == pdTRUE)
        {
            // JY61P原始数据
            float yaw = imu_data.yaw;
            float pitch = imu_data.pitch;
            float roll = imu_data.roll;
            
            // 从 set_angle_test_task 获取数据
            float target_yaw = Test_Get_Target_Yaw();
            float target_pitch = Test_Get_Target_Pitch();
            float target_roll = Test_Get_Target_Roll();
            
            // 从 set_angle_test_task 获取角度误差
            float error_yaw = Test_Get_Error_Yaw();
            float error_pitch = Test_Get_Error_Pitch();
            float error_roll = Test_Get_Error_Roll();
            
            // PID 参数
            float kp_x = Test_Get_PID_Kp_X();
            float ki_x = Test_Get_PID_Ki_X();
            float kd_x = Test_Get_PID_Kd_X();

            float kp_y = Test_Get_PID_Kp_Y();
            float ki_y = Test_Get_PID_Ki_Y();
            float kd_y = Test_Get_PID_Kd_Y();
            
            float control_enable = Get_Control_Enable();  // 用 float 接收
            
            // 发送JustFloat模式数据
            Vofa_UART.Set_Data(16, 
                               &yaw, &pitch, &roll,
                               &target_yaw, &target_pitch, &target_roll,
                               &error_yaw, &error_pitch, &error_roll,
                               &kp_x, &ki_x, &kd_x,
                               &kp_y, &ki_y, &kd_y,
                               &control_enable);
            
            Vofa_UART.TIM_1ms_Write_PeriodElapsedCallback();
        }
    }
}

// ========== VOFA命令处理任务 ==========
static void vVofaCmdTask(void *pvParameters)
{
    VofaCommand_t cmd;
    
    for(;;)
    {
        if (xQueueReceive(xVofaCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            switch (cmd.index)
            {
            case 0:     // 目标yaw角
                Set_Target_Yaw(cmd.value);
                break;
            case 1:     // 目标pitch角
                Set_Target_Pitch(cmd.value);
                break;
            case 2:     // 目标roll角
                Set_Target_Roll(cmd.value);
                break;
            case 3:     // 水平电机 误差转为目标速度的kp
                Set_Angle_PID_Kp_X(cmd.value);
                break;
            case 4:     // 水平电机 误差转为目标速度的ki
                Set_Angle_PID_Ki_X(cmd.value);
                break;
            case 5:     // 水平电机 误差转为目标速度的kd
                Set_Angle_PID_Kd_X(cmd.value);
                break;
            case 6:     // 竖直电机 误差转为目标速度的kp
                Set_Angle_PID_Kp_Y(cmd.value);
                break;
            case 7:     // 竖直电机 误差转为目标速度的ki
                Set_Angle_PID_Ki_Y(cmd.value);
                break;
            case 8:     // 竖直电机 误差转为目标速度的kd
                Set_Angle_PID_Kd_Y(cmd.value);
                break;

            case 9:  
            {   // 使能标志
                Set_Control_Enable(cmd.value);
                float value = cmd.value;
                break; 
            }
            default:
                break;
            }
        }
    }
}

// ========== UART接收回调函数 ==========
void Serial_UART_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (xVofaCommandQueue != NULL)
    {
        Vofa_UART.UART_RxCpltCallback((const uint8_t*)Buffer, Length);
        
        int32_t index = Vofa_UART.Get_Variable_Index();
        float value = Vofa_UART.Get_Variable_Value();
        
        if (index >= 0)
        {
            VofaCommand_t cmd = {index, value};
            xQueueSendFromISR(xVofaCommandQueue, &cmd, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// ========== VOFA任务初始化 ==========
void Vofa_Task_Create(void)
{
    // 等待稳定VOFA发送接收
    vTaskDelay(pdMS_TO_TICKS(500));
    Vofa_UART.Init(&huart7, 
                   VOFA_RX_VARIABLE_NUM, 
                   (const char **)Vofa_Variable_List, 
                   0x7F800000);
    
    UART_Init(&huart7, Serial_UART_Call_Back);
    
    xVofaCommandQueue = xQueueCreate(VOFA_CMD_QUEUE_LENGTH, sizeof(VofaCommand_t));
    if (xVofaCommandQueue == NULL)
    {
        while (1);
    }
    
    BaseType_t ret1 = xTaskCreate(vVofaSendTask,
                                  "VofaSend",
                                  VOFA_TASK_STACK_SIZE,
                                  NULL,
                                  VOFA_TASK_PRIORITY_SEND,
                                  NULL);
    
    BaseType_t ret2 = xTaskCreate(vVofaCmdTask,
                                  "VofaCmd",
                                  VOFA_TASK_STACK_SIZE,
                                  NULL,
                                  VOFA_TASK_PRIORITY_CMD,
                                  NULL);
    
    if (ret1 != pdPASS || ret2 != pdPASS)
    {
        while (1);
    }
}