#include "drv_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_jy61p.h"
#include "jy61p_task.h"
#include "bsp_power.h"
#include <stdbool.h>
#include <stdio.h>

#include "queue.h"  // 传递打印任务使用


/*
 * ============================================================================
 * JY61P 任务修改说明 - 从串口打印改为队列传输，用VOFA的JUSTFLOAT来输出打印
 * ============================================================================
 * 
 * 【修改目的】
 *   将 JY61P 姿态传感器数据通过 FreeRTOS 队列传递给 VOFA 任务，
 *   实现通过 VOFA 上位机实时显示 IMU 数据，并支持参数调节。
 *
 * 【修改前 - 串口打印方式】
 *   直接在 JY61P 任务中通过 HAL_UART_Transmit 打印数据到串口，
 *   这种方式虽然简单，但数据无法被其他任务使用，且占用串口资源。
 *
 * 【修改后 - 队列传输方式】
 *   1. 创建全局队列 xIMUDataQueue，用于传递 IMU 数据
 *   2. JY61P 任务获取数据后，通过队列发送给 VOFA 任务
 *   3. VOFA 任务从队列读取数据，通过 VOFA 协议发送到上位机
 *
 * ============================================================================
 */

/*
 * ============================================================================
 * 步骤1：在 jy61p_task.h 或 jy61p_task.cpp 中声明全局队列
 * ============================================================================
 * 
 * // 在文件顶部（其他 include 之后）添加：
 * #include "queue.h"
 * 
 * // 声明全局队列句柄，供 VOFA 任务访问
 * // 注意：需要声明为 extern，在 .c/.cpp 文件中定义
 * QueueHandle_t xIMUDataQueue = NULL;
 * 
 * ============================================================================
 */

/*
 * ============================================================================
 * 步骤2：在 JY61P 任务创建函数中创建队列
 * ============================================================================
 * 
 * void JY61P_Task_Create(void)
 * {
 *     // 初始化电源（如果需要）
 *     BSP_Power.Init(false, false, true);
 *     
 *     // 【关键修改1】创建数据队列
 *     // 参数说明：
 *     //   1 - 队列长度，设置为 1 表示只保存最新数据（覆盖模式）
 *     //   sizeof(AttitudeData_t) - 每个队列项的大小，即姿态数据结构体的大小
 *     // 为什么长度设为1？
 *     //   - IMU 数据更新频率高（100Hz），我们只关心最新值
 *     //   - 使用覆盖模式可以避免队列满导致任务阻塞
 *     //   - 减少内存占用，提高效率
 *     xIMUDataQueue = xQueueCreate(1, sizeof(AttitudeData_t));
 *     if (xIMUDataQueue == NULL)
 *     {
 *         // 队列创建失败，系统无法正常工作
 *         while (1);
 *     }
 *     
 *     // 创建 JY61P 任务
 *     BaseType_t ret = xTaskCreate(vJY61PTask,      // 任务函数
 *                                  "vJY61PTask",    // 任务名称
 *                                  512,              // 栈大小
 *                                  NULL,             // 参数
 *                                  2,                // 优先级
 *                                  &xJY61PTaskHandle);
 *     if (ret != pdPASS)
 *     {
 *         while (1);  // 任务创建失败
 *     }
 * }
 * 
 * ============================================================================
 */

/*
 * ============================================================================
 * 步骤3：修改 JY61P 任务函数，使用队列发送数据
 * ============================================================================
 * 
 * static void vJY61PTask(void *pvParameters)
 * {
 *     // 初始化 JY61P 模块，绑定串口10（波特率230400）
 *     BSP_JY61P.Init(&huart10);
 *     
 *     // 定义数据结构体，用于存储姿态数据
 *     AttitudeData_t data;
 *     
 *     // 可选：用于控制打印频率的变量（如果需要保留调试打印）
 *     // uint32_t last_print = 0;
 *     
 *     while (1)
 *     {
 *         // 【关键修改2】获取 JY61P 姿态数据
 *         // 通过批量接口一次性获取所有数据（yaw, pitch, roll, accel, gyro, quat）
 *         BSP_JY61P.GetAttitudeData(&data);
 *         
 *         // 【关键修改3】通过队列发送数据
 *         // 使用 xQueueOverwrite 而非 xQueueSend 的原因：
 *         //   - xQueueOverwrite 在队列满时会直接覆盖旧数据，不会阻塞
 *         //   - 确保 JY61P 任务不会因为队列满而被挂起
 *         //   - 始终保留最新的 IMU 数据，丢弃过时的数据
 *         if (xIMUDataQueue != NULL)
 *         {
 *             xQueueOverwrite(xIMUDataQueue, &data);
 *         }
 *         
 *         // 【可选】如果需要保留调试打印，可以降低打印频率
 *         // 例如：每 500ms 打印一次，用于调试
 *         // uint32_t now = HAL_GetTick();
 *         // if (now - last_print > 500)
 *         // {
 *         //     last_print = now;
 *         //     float roll = BSP_JY61P.GetRoll();
 *         //     printf("yaw:%.2f pitch:%.2f roll:%.2f\r\n", 
 *         //            data.yaw, data.pitch, roll);
 *         // }
 *         
 *         // 延时 10ms，实现 100Hz 的采样频率
 *         // 注意：JY61P 的数据更新频率为 100Hz，延时 10ms 可以保证每次都能获取到新数据
 *         vTaskDelay(pdMS_TO_TICKS(10));
 *     }
 * }
 * 
 * ============================================================================
 */

/*
 * ============================================================================
 * 步骤4：在 VOFA 任务中读取队列数据
 * ============================================================================
 * 
 * // 在 vofa_task.cpp 中已经实现：
 * static void vVofaSendTask(void *pvParameters)
 * {
 *     AttitudeData_t imu_data;  // 用于接收 IMU 数据
 *     float yaw, pitch, roll;
 *     
 *     for(;;)
 *     {
 *         vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz 发送频率
 *         
 *         // 【关键修改4】从队列读取最新 IMU 数据
 *         // 使用 xQueuePeek 而非 xQueueReceive 的原因：
 *         //   - xQueuePeek 只读取数据，不从队列中移除
 *         //   - 多个任务可以同时读取同一份数据（虽然目前只有 VOFA 任务）
 *         //   - 保留数据供其他可能需要的任务使用
 *         if (xQueuePeek(xIMUDataQueue, &imu_data, 0) == pdTRUE)
 *         {
 *             yaw = imu_data.yaw;
 *             pitch = imu_data.pitch;
 *             roll = BSP_JY61P.GetRoll();  // roll 角直接从模块获取
 *             
 *             // 发送数据到 VOFA...
 *         }
 *     }
 * }
 * 
 * ============================================================================
 */

/*
 * ============================================================================
 * 修改总结与关键点
 * ============================================================================
 * 
 * 【队列选择】
 *   - 使用 xQueueCreate(1, sizeof(AttitudeData_t)) 创建长度为 1 的队列
 *   - 原因：只需要最新数据，历史数据无意义
 * 
 * 【写入方式】
 *   - 使用 xQueueOverwrite() 而非 xQueueSend()
 *   - 原因：避免队列满时任务阻塞，始终保留最新数据
 * 
 * 【读取方式】
 *   - 使用 xQueuePeek() 而非 xQueueReceive()
 *   - 原因：只读取不删除，保留数据供其他任务使用
 * 
 * 【数据流向】
 *   JY61P 硬件 → BSP_JY61P.GetAttitudeData() → 
 *   xQueueOverwrite() → xIMUDataQueue → 
 *   xQueuePeek() → VOFA 任务 → 上位机显示
 * 
 * 【优势】
 *   1. 数据解耦：JY61P 任务和 VOFA 任务独立运行，互不影响
 *   2. 实时性：队列覆盖模式确保始终传输最新数据
 *   3. 可扩展：其他任务也可以轻松添加类似的数据队列
 *   4. 资源节约：长度 1 的队列占用最小内存
 * 
 * ============================================================================
 */



// 注意：这里定义为全局变量，方便VOFA任务访问
QueueHandle_t xIMUDataQueue = NULL;

// 任务句柄
static TaskHandle_t xJY61PTaskHandle = NULL;

// 声明外部串口句柄
extern UART_HandleTypeDef huart7;   // 打印用
extern UART_HandleTypeDef huart10;  // JY61P用


void float_to_str(float value, char* buffer, int buffer_size)
{
    if (buffer == NULL || buffer_size <= 0) {
        return;
    }
    
    // 处理负数
    int is_negative = 0;
    if (value < 0) {
        is_negative = 1;
        value = -value;
    }
    
    // 处理超大数值
    if (value > 9999999.0f) {  // 根据实际需求调整
        snprintf(buffer, buffer_size, "%.2f", is_negative ? -value : value);
        return;
    }
    
    // 使用整数运算提高精度
    // 将浮点数转换为整数表示（乘以100）
    long long scaled_value = (long long)(value * 100.0f + 0.5f);
    
    // 处理进位后的小数部分（需要先取整）
    int int_part = (int)(scaled_value / 100);
    int frac_part = (int)(scaled_value % 100);
    
    // 格式化输出
    int written;
    if (is_negative) {
        written = snprintf(buffer, buffer_size, "-%d.%02d", int_part, frac_part);
    } else {
        written = snprintf(buffer, buffer_size, "%d.%02d", int_part, frac_part);
    }
    
    // 确保字符串以null结尾（snprintf已保证，但为了安全）
    if (written >= buffer_size && buffer_size > 0) {
        buffer[buffer_size - 1] = '\0';
    }
}

/**
 * @brief JY61P任务入口函数
 * 参考Control的实现风格
 */
static void vJY61PTask(void *pvParameters)
{
    // 初始化JY61P，绑定串口10
    BSP_JY61P.Init(&huart10);   // 波特率为230400
    AttitudeData_t data;

    uint32_t last_print = 0;
    
    while (1)
    {
        // 每50ms打印一次数据
        uint32_t now = HAL_GetTick();
        if (now - last_print > 50)
        {
            last_print = now;
            
            BSP_JY61P.GetAttitudeData(&data);
            
            
            // 使用 Overwrite 方式，只保留最新数据，避免队列满阻塞
            if (xIMUDataQueue != NULL)
            {
                xQueueOverwrite(xIMUDataQueue, &data);
            }

            // // 转换所有数据为字符串
            // char yaw_str[20], pitch_str[20], roll_str[20];
            // float_to_str(data.yaw, yaw_str, sizeof(yaw_str));
            // float_to_str(data.pitch, pitch_str, sizeof(pitch_str));
            // float_to_str(data.roll, roll_str, sizeof(roll_str));
            
            // char accel_x_str[20], accel_y_str[20], accel_z_str[20];
            // float_to_str(data.accel_x, accel_x_str, sizeof(accel_x_str));
            // float_to_str(data.accel_y, accel_y_str, sizeof(accel_y_str));
            // float_to_str(data.accel_z, accel_z_str, sizeof(accel_z_str));  
            
            // char gyro_x_str[20], gyro_y_str[20], gyro_z_str[20];
            // float_to_str(data.gyro_x, gyro_x_str, sizeof(gyro_x_str));
            // float_to_str(data.gyro_y, gyro_y_str, sizeof(gyro_y_str));
            // float_to_str(data.gyro_z, gyro_z_str, sizeof(gyro_z_str));
            
            // char quat_q0_str[20], quat_q1_str[20], quat_q2_str[20], quat_q3_str[20];
            // float_to_str(data.quat_q0, quat_q0_str, sizeof(quat_q0_str));
            // float_to_str(data.quat_q1, quat_q1_str, sizeof(quat_q1_str));
            // float_to_str(data.quat_q2, quat_q2_str, sizeof(quat_q2_str));
            // float_to_str(data.quat_q3, quat_q3_str, sizeof(quat_q3_str));
            
            // // 合并为一次发送
            // char buffer[512];
            // int dlen = snprintf(buffer, sizeof(buffer),
            //     "=== JY61P ===\r\n"
            //     "yaw:%s pitch:%s roll:%s\r\n"
            //     "accel{x:%s y:%s z:%s}\r\n"
            //     "gyro{x:%s y:%s z:%s}\r\n"
            //     "quat{q0:%s q1:%s q2:%s q3:%s}\r\n"
            //     "============\r\n",
            //     yaw_str, pitch_str, roll_str,
            //     accel_x_str, accel_y_str, accel_z_str,
            //     gyro_x_str, gyro_y_str, gyro_z_str,
            //     quat_q0_str, quat_q1_str, quat_q2_str, quat_q3_str);
            
            // HAL_UART_Transmit(&huart7, (uint8_t*)buffer, dlen, HAL_MAX_DELAY);

        }
        
        // 延时10ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}




/**
 * @brief 创建JY61P任务
 * 参考Control_Task_Create的实现
 */
void JY61P_Task_Create(void)
{
    BSP_Power.Init(false,false,true);

    // 队列长度为1，只保存最新数据
    // 如果创建失败，可以通过返回值判断
    xIMUDataQueue = xQueueCreate(1, sizeof(AttitudeData_t));
    if (xIMUDataQueue == NULL)
    {
        // 队列创建失败，可以根据需要添加错误处理
        // 例如：死循环或记录错误日志
        while (1)
        {
            // 队列创建失败，系统无法正常工作
        }
    }

    BaseType_t ret = xTaskCreate(vJY61PTask,          // 任务函数
                                 "vJY61PTask",        // 任务名称
                                 512,                  // 栈大小（字）
                                 NULL,                 // 参数
                                 2,                    // 优先级（与Control相同）
                                 &xJY61PTaskHandle);
    if (ret != pdPASS)
    {
        while (1)
        {
            // 任务创建失败，停止系统
        }
    }
}