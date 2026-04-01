#include "drv_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_jy61p.h"
#include "jy61p_task.h"
#include "bsp_power.h"
#include <stdbool.h>
#include <stdio.h>

#include "queue.h"  // 传递打印任务使用



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