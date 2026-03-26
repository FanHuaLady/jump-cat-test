// vofa_task.cpp
// 功能：VOFA串口调试任务，实现IMU数据通过VOFA JustFloat协议发送，并支持通过串口命令调节参数

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "vofa_task.h"
#include "dvc_vofa.h"
#include "bsp_jy61p.h"
#include "jy61p_task.h"

#include <string.h>
#include <stdio.h>




/*
 * 任务说明：
 * 该任务实现了以下功能：
 * 1. 通过VOFA JustFloat协议发送IMU数据（yaw, pitch, roll）和可调参数
 * 2. 接收VOFA发送的命令（格式：变量名=数值#），实现对参数的增加、减少、设置和重置
 * 3. 使用FreeRTOS的队列和互斥锁实现任务间通信和数据保护
 * 
 * 命令格式说明：
 * - q00=数值#   : 参数增加指定数值
 * - q11=数值#   : 参数减少指定数值  
 * - r00=数值#   : 直接设置参数为指定数值
 * - r11#        : 重置参数为默认值25.0
 */

// ========== 外部变量声明 ==========
extern Class_Vofa_UART Vofa_UART;          // VOFA UART对象，处理数据收发
extern QueueHandle_t xIMUDataQueue;        // IMU数据队列，存储姿态数据
extern UART_HandleTypeDef huart7;          // UART7硬件句柄

// ========== 静态变量 ==========
static QueueHandle_t xVofaCommandQueue = NULL;  // VOFA命令队列，接收来自串口的命令
static SemaphoreHandle_t xVofaMutex = NULL;     // 互斥锁，保护共享变量g_custom_value
static float g_custom_value = 25.0f;            // 本任务测试用的可调节参数，默认值25.0，范围-100~100

// 接收变量名列表 - 定义VOFA可以识别的命令名称
// 注意：变量名必须与VOFA软件中发送的命令名称完全匹配
// 对应测试参数，该任务中是：static float g_custom_value = 25.0f;    
static char Vofa_Variable_List[][VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = 
{
    {"q00"},    // 索引0: 增加命令 - 发送 "q00=数值#" 增加参数
    {"q11"},    // 索引1: 减少命令 - 发送 "q11=数值#" 减少参数
    {"r00"},    // 索引2: 设置命令 - 发送 "r00=数值#" 直接设置参数
    {"r11"}     // 索引3: 重置命令 - 发送 "r11#" 重置参数为默认值
};

// ========== 静态函数声明 ==========
static void vVofaSendTask(void *pvParameters);   // VOFA发送任务，定期发送IMU数据
static void vVofaCmdTask(void *pvParameters);    // VOFA命令处理任务，处理接收到的命令

// ========== VOFA发送任务 ==========
// 功能：定期（50Hz）从IMU队列获取姿态数据，通过VOFA发送到上位机
// 发送内容：yaw(偏航角), pitch(俯仰角), roll(横滚角), custom_value(可调参数)
static void vVofaSendTask(void *pvParameters)
{
    AttitudeData_t imu_data;      // IMU数据结构体，存储yaw, pitch, roll
    float yaw, pitch, roll;       // 姿态角
    float current_custom_value;   // 当前参数值（局部副本）
    
    for(;;)
    {
        // 延时20ms，实现50Hz的发送频率
        // 50Hz = 1000ms / 20ms = 50次/秒，适合实时数据显示
        vTaskDelay(pdMS_TO_TICKS(20));
        
        // 从IMU队列获取最新数据（peek方式，不移除队列项）
        // xQueuePeek只读取不移除，适合多个任务同时读取最新数据
        if (xQueuePeek(xIMUDataQueue, &imu_data, 0) == pdTRUE)
        {
            // 提取姿态数据
            yaw = imu_data.yaw;
            pitch = imu_data.pitch;
            // 注意：roll角直接从JY61P模块获取，不从队列读取，测试用，看是否可以
            roll = BSP_JY61P.GetRoll();
            
            // 获取互斥锁，保护共享变量g_custom_value
            // 等待最多10ms，超时则跳过本次发送
            if (xSemaphoreTake(xVofaMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                // 复制当前参数值到局部变量，避免在发送过程中被修改
                current_custom_value = g_custom_value;
                
                // 设置要发送的数据：4个浮点数
                // 参数顺序：yaw, pitch, roll, custom_value
                Vofa_UART.Set_Data(4, &yaw, &pitch, &roll, &current_custom_value);
                
                // 触发VOFA发送，将数据通过DMA发送到串口
                // 内部会调用Output()准备数据，然后通过HAL_UART_Transmit_DMA发送
                Vofa_UART.TIM_1ms_Write_PeriodElapsedCallback();
                
                // 释放互斥锁
                xSemaphoreGive(xVofaMutex);
            }   
        }
    }
}

// ========== VOFA命令处理任务 ==========
// 功能：从命令队列中接收命令，解析并修改参数值
// 支持：增加、减少、直接设置、重置四种操作
static void vVofaCmdTask(void *pvParameters)
{
    VofaCommand_t cmd;        // 命令结构体，包含命令索引和数值
    float old_value;          // 修改前的参数值
    float new_value;          // 修改后的参数值
    
    for(;;)
    {
        // 等待命令队列中的命令（阻塞等待，直到有命令到达）
        // portMAX_DELAY表示无限等待，任务会阻塞直到收到命令
        if (xQueueReceive(xVofaCommandQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            // 获取互斥锁，保护共享变量g_custom_value
            // 等待最多100ms，确保能够获取到锁
            if (xSemaphoreTake(xVofaMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                // 保存原值
                old_value = g_custom_value;
                
                // 根据命令索引执行相应操作
                switch (cmd.index)
                {
                case 0:  // q00命令 - 增加
                    new_value = old_value + cmd.value;
                    g_custom_value = new_value;
                    // 限制范围在 -100 到 100 之间
                    if (g_custom_value > 100.0f) g_custom_value = 100.0f;
                    if (g_custom_value < -100.0f) g_custom_value = -100.0f;
                    break;
                    
                case 1:  // q11命令 - 减少
                    new_value = old_value - cmd.value;
                    g_custom_value = new_value;
                    // 限制范围在 -100 到 100 之间
                    if (g_custom_value > 100.0f) g_custom_value = 100.0f;
                    if (g_custom_value < -100.0f) g_custom_value = -100.0f;
                    break;
                    
                case 2:  // r00命令 - 直接设置
                    g_custom_value = cmd.value;
                    break;
                    
                case 3:  // r11命令 - 重置为默认值
                    g_custom_value = 25.0f;
                    break;
                    
                default:
                    // 未知命令，不做任何处理
                    break;
                }
                
                // 释放互斥锁
                xSemaphoreGive(xVofaMutex);
            }
        }
    }
}

// ========== UART接收回调函数 ==========
// 功能：在UART中断中被调用，处理接收到的VOFA命令
// 参数：Buffer - 接收到的数据缓冲区
//       Length - 数据长度
// 注意：此函数在中断上下文中执行，必须快速处理，不能有阻塞操作
void Serial_UART_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // 用于任务切换标志
    
    // 检查命令队列是否已创建
    if (xVofaCommandQueue != NULL)
    {
        // 调用VOFA的接收回调，解析变量名和数值
        // 内部会识别命令格式"变量名=数值#"，并解析出命令索引和数值
        Vofa_UART.UART_RxCpltCallback((const uint8_t*)Buffer, Length);
        
        // 获取解析结果
        int32_t index = Vofa_UART.Get_Variable_Index();  // 命令索引（0-3）
        float value = Vofa_UART.Get_Variable_Value();    // 命令数值
        
        // 如果解析到有效的变量名（index >= 0）
        if (index >= 0)
        {
            // 构造命令结构体
            VofaCommand_t cmd = {index, value};
            
            // 从中断中发送命令到队列
            // xQueueSendFromISR是中断安全版本的队列发送函数
            xQueueSendFromISR(xVofaCommandQueue, &cmd, &xHigherPriorityTaskWoken);
            
            // 如果发送导致更高优先级任务就绪，则进行任务切换
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// ========== 辅助函数 ==========
/**
 * @brief 获取当前参数值（供其他任务调用）
 * @return float 当前参数值
 * @note 线程安全，通过互斥锁保护
 */
float Vofa_Get_Custom_Value(void)
{
    float value = 0;
    
    // 获取互斥锁，保护共享变量
    if (xSemaphoreTake(xVofaMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        value = g_custom_value;      // 读取当前值
        xSemaphoreGive(xVofaMutex);   // 释放互斥锁
    }
    
    return value;
}

/**
 * @brief 设置参数值（供其他任务调用）
 * @param value 要设置的新值
 * @note 线程安全，通过互斥锁保护
 */
void Vofa_Set_Custom_Value(float value)
{
    // 获取互斥锁，保护共享变量
    if (xSemaphoreTake(xVofaMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_custom_value = value;      // 设置新值
        xSemaphoreGive(xVofaMutex);   // 释放互斥锁
    }
}

// ========== VOFA任务初始化 ==========
/**
 * @brief 创建VOFA任务，初始化所有相关资源
 * @note 在系统启动时调用一次
 */
void Vofa_Task_Create(void)
{
    // 步骤1：初始化VOFA对象
    // 参数说明：
    //   &huart7          - 使用的UART硬件
    //   VOFA_RX_VARIABLE_NUM - 接收命令数量（4个）
    //   Vofa_Variable_List   - 命令名称列表
    //   0x7F800000       - JustFloat协议的帧尾标志
    Vofa_UART.Init(&huart7, 
                   VOFA_RX_VARIABLE_NUM, 
                   (const char **)Vofa_Variable_List, 
                   0x7F800000);
    
    // 步骤2：注册UART接收回调函数
    // 这个函数会配置UART中断和DMA，使能接收功能
    UART_Init(&huart7, Serial_UART_Call_Back);
    
    // 步骤3：创建互斥锁，用于保护共享变量g_custom_value
    xVofaMutex = xSemaphoreCreateMutex();
    if (xVofaMutex == NULL)
    {
        // 互斥锁创建失败，系统无法正常运行，死循环
        while (1);
    }
    
    // 步骤4：创建命令队列，用于接收VOFA命令
    // 队列长度：VOFA_CMD_QUEUE_LENGTH（默认10）
    // 队列项大小：sizeof(VofaCommand_t)
    xVofaCommandQueue = xQueueCreate(VOFA_CMD_QUEUE_LENGTH, sizeof(VofaCommand_t));
    if (xVofaCommandQueue == NULL)
    {
        // 队列创建失败，系统无法正常运行，死循环
        while (1);
    }
    
    // 步骤5：创建VOFA发送任务
    // 任务名称：VofaSend
    // 任务栈大小：VOFA_TASK_STACK_SIZE（512字）
    // 任务优先级：VOFA_TASK_PRIORITY_SEND（2）
    BaseType_t ret1 = xTaskCreate(vVofaSendTask,
                                  "VofaSend",
                                  VOFA_TASK_STACK_SIZE,
                                  NULL,
                                  VOFA_TASK_PRIORITY_SEND,
                                  NULL);
    
    // 步骤6：创建VOFA命令处理任务
    // 任务名称：VofaCmd  
    // 任务栈大小：VOFA_TASK_STACK_SIZE（512字）
    // 任务优先级：VOFA_TASK_PRIORITY_CMD（3，略高于发送任务）
    BaseType_t ret2 = xTaskCreate(vVofaCmdTask,
                                  "VofaCmd",
                                  VOFA_TASK_STACK_SIZE,
                                  NULL,
                                  VOFA_TASK_PRIORITY_CMD,
                                  NULL);
    
    // 检查任务创建是否成功
    if (ret1 != pdPASS || ret2 != pdPASS)
    {
        // 任务创建失败，系统无法正常运行，死循环
        while (1);
    }
}