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
extern void Set_Target_Yaw(float yaw_deg);
extern void Set_Target_Pitch(float pitch_deg);
extern void Set_Target_Roll(float roll_deg);

// VOFA显示接口（目标角度）
extern float Test_Get_Target_Yaw(void);
extern float Test_Get_Target_Pitch(void);
extern float Test_Get_Target_Roll(void);

// VOFA显示接口（当前角度）
extern float Test_Get_Current_Yaw(void);
extern float Test_Get_Current_Pitch(void);
extern float Test_Get_Current_Roll(void);

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
    {"roll"}
};

// ========== 静态函数声明 ==========
static void vVofaSendTask(void *pvParameters);
static void vVofaCmdTask(void *pvParameters);

// ========== VOFA发送任务 ==========
static void vVofaSendTask(void *pvParameters)
{
    AttitudeData_t imu_data;
    float yaw, pitch, roll;
    float target_yaw, target_pitch, target_roll;
    float current_yaw, current_pitch, current_roll;
    float error_yaw, error_pitch, error_roll;
    
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        
        if (xQueuePeek(xIMUDataQueue, &imu_data, 0) == pdTRUE)
        {
            // JY61P原始数据
            yaw = imu_data.yaw;
            pitch = imu_data.pitch;
            roll = imu_data.roll;
            
            // 从 set_angle_test_task 获取数据
            target_yaw = Test_Get_Target_Yaw();
            target_pitch = Test_Get_Target_Pitch();
            target_roll = Test_Get_Target_Roll();
            
            current_yaw = Test_Get_Current_Yaw();
            current_pitch = Test_Get_Current_Pitch();
            current_roll = Test_Get_Current_Roll();
            
            error_yaw = Test_Get_Error_Yaw();
            error_pitch = Test_Get_Error_Pitch();
            error_roll = Test_Get_Error_Roll();
            
            // 发送12个数据
            Vofa_UART.Set_Data(12, 
                               &yaw, &pitch, &roll,
                               &target_yaw, &target_pitch, &target_roll,
                               &current_yaw, &current_pitch, &current_roll,
                               &error_yaw, &error_pitch, &error_roll);
            
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
            case 0:
                Set_Target_Yaw(cmd.value);
                break;
            case 1:
                Set_Target_Pitch(cmd.value);
                break;
            case 2:
                Set_Target_Roll(cmd.value);
                break;
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