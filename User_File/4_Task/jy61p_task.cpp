#include "drv_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_jy61p.h"
#include "jy61p_task.h"
#include "bsp_power.h"
#include <stdbool.h>
#include <stdio.h>

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
    
    uint32_t last_print = 0;
    
    while (1)
    {
        // 每50ms打印一次数据
        uint32_t now = HAL_GetTick();
        if (now - last_print > 50)
        {
            last_print = now;
            
            // 使用批量接口获取所有数据（一次临界区，效率高）
            AttitudeData_t data;
            BSP_JY61P.GetAttitudeData(&data);
            
            float roll = BSP_JY61P.GetRoll();

            // 转换所有数据为字符串
            char yaw_str[20], pitch_str[20], roll_str[20];
            float_to_str(data.yaw, yaw_str, sizeof(yaw_str));
            float_to_str(data.pitch, pitch_str, sizeof(pitch_str));
            float_to_str(roll, roll_str, sizeof(roll_str));
            
            char accel_x_str[20], accel_y_str[20], accel_z_str[20];
            float_to_str(data.accel_x, accel_x_str, sizeof(accel_x_str));
            float_to_str(data.accel_y, accel_y_str, sizeof(accel_y_str));
            float_to_str(data.accel_z, accel_z_str, sizeof(accel_z_str));  
            
            char gyro_x_str[20], gyro_y_str[20], gyro_z_str[20];
            float_to_str(data.gyro_x, gyro_x_str, sizeof(gyro_x_str));
            float_to_str(data.gyro_y, gyro_y_str, sizeof(gyro_y_str));
            float_to_str(data.gyro_z, gyro_z_str, sizeof(gyro_z_str));
            
            char quat_q0_str[20], quat_q1_str[20], quat_q2_str[20], quat_q3_str[20];
            float_to_str(data.quat_q0, quat_q0_str, sizeof(quat_q0_str));
            float_to_str(data.quat_q1, quat_q1_str, sizeof(quat_q1_str));
            float_to_str(data.quat_q2, quat_q2_str, sizeof(quat_q2_str));
            float_to_str(data.quat_q3, quat_q3_str, sizeof(quat_q3_str));
            
            // 合并为一次发送
            char buffer[512];
            int dlen = snprintf(buffer, sizeof(buffer),
                "=== JY61P ===\r\n"
                "yaw:%s pitch:%s roll:%s\r\n"
                "accel{x:%s y:%s z:%s}\r\n"
                "gyro{x:%s y:%s z:%s}\r\n"
                "quat{q0:%s q1:%s q2:%s q3:%s}\r\n"
                "============\r\n",
                yaw_str, pitch_str, roll_str,
                accel_x_str, accel_y_str, accel_z_str,
                gyro_x_str, gyro_y_str, gyro_z_str,
                quat_q0_str, quat_q1_str, quat_q2_str, quat_q3_str);
            
            HAL_UART_Transmit(&huart7, (uint8_t*)buffer, dlen, HAL_MAX_DELAY);

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