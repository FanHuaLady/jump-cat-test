/**
 * @file bsp_jy61p.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief JY61P陀螺仪BSP实现
 * @version 0.1
 * @date 2025-08-14
 * 
 * @copyright USTC-RoboWalker (c) 2025
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_jy61p.h"
#include "FreeRTOS.h"
#include "task.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 全局JY61P对象定义
Class_JY61P BSP_JY61P;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化JY61P，绑定UART并注册回调
 * 
 * @param huart UART句柄
 */
void Class_JY61P::Init(UART_HandleTypeDef *huart)
{
    // 初始化状态机
    rx_state = 0;
    rx_index = 0;
    expected_packet_type = PACKET_NONE;

    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;

    accel_x = 0.0f;
    accel_y = 0.0f;
    accel_z = 0.0f;
    
    gyro_x = 0.0f;
    gyro_y = 0.0f;
    gyro_z = 0.0f;
    
    quat_q0 = 1.0f;  // 四元数初始化为单位四元数
    quat_q1 = 0.0f;
    quat_q2 = 0.0f;
    quat_q3 = 0.0f;

    // 注册UART接收回调
    UART_Init(huart, UART_Callback);
}

/**
 * @brief UART接收回调包装函数
 * 
 * @param Buffer 数据缓冲区
 * @param Length 数据长度
 */
void Class_JY61P::UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    // 中断临界区保护，防止任务打断数据写入
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    
    // 调用全局对象的处理函数
    BSP_JY61P.DataProcess(Buffer, Length);
    
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/**
 * @brief 处理接收到的字节流
 * 
 * @param Buffer 数据缓冲区
 * @param Length 数据长度
 */
void Class_JY61P::DataProcess(uint8_t *Buffer, uint16_t Length)
{
    for (uint16_t i = 0; i < Length; i++)
    {
        ParseByte(Buffer[i]);
    }
}

/**
 * @brief 解析单个字节（状态机）
 * 
 * @param data 输入的字节
 */
void Class_JY61P::ParseByte(uint8_t data)
{
    uint8_t sum = 0;

    if (rx_state == 0)                     // 等待包头
    {
        if (data == 0x55)                   // 收到包头
        {
            rx_buffer[0] = data;
            rx_state = 1;
            rx_index = 1;
        }
    }
    else if (rx_state == 1)                 // 等待数据类型
    {
        if (data == 0x53)                   // 角度输出
        {
            rx_buffer[1] = data;
            rx_state = 2;
            rx_index = 2;
            expected_packet_type = ANGLE_PACKET;
        }
        else if (data == 0x51)              // 加速度输出
        {
            rx_buffer[1] = data;
            rx_state = 2;
            rx_index = 2;
            expected_packet_type = ACCEL_PACKET;
        }
        else if (data == 0x52)              // 角速度输出
        {
            rx_buffer[1] = data;
            rx_state = 2;
            rx_index = 2;
            expected_packet_type = GYRO_PACKET;
        }
        else if (data == 0x59)              // 四元数输出
        {
            rx_buffer[1] = data;
            rx_state = 2;
            rx_index = 2;
            expected_packet_type = QUAT_PACKET;
        }
        else                                 // 不是期望的类型，重置状态并重新处理该字节（可能是新包头）
        {
            rx_state = 0;
            rx_index = 0;
            expected_packet_type = PACKET_NONE;
            ParseByte(data);                 // 递归处理，避免丢失可能的包头
        }
    }
    else if (rx_state == 2)                 // 接收数据体
    {
        rx_buffer[rx_index++] = data;
        if (rx_index == 11)                  // 接收完一包
        {
            // 计算校验和
            for (uint8_t i = 0; i < 10; i++)
            {
                sum += rx_buffer[i];
            }
            if (sum == rx_buffer[10])         // 校验通过
            {
                // // 解析角度：低字节在前，高字节在后，转换为有符号16位整数
                // roll  = ((int16_t)((int16_t)rx_buffer[3] << 8 | (int16_t)rx_buffer[2])) / 32768.0f * 180.0f;
                // pitch = ((int16_t)((int16_t)rx_buffer[5] << 8 | (int16_t)rx_buffer[4])) / 32768.0f * 180.0f;
                // yaw   = ((int16_t)((int16_t)rx_buffer[7] << 8 | (int16_t)rx_buffer[6])) / 32768.0f * 180.0f;
                // ==================== 角度，加速度，角速度，四元数解析 ====================
                if (expected_packet_type == ANGLE_PACKET)
                {
                    // 解析角度：低字节在前，高字节在后，转换为有符号16位整数
                    // 范围：-180° ~ +180°
                    roll  = ((int16_t)((int16_t)rx_buffer[3] << 8 | (int16_t)rx_buffer[2])) / 32768.0f * 180.0f;
                    pitch = ((int16_t)((int16_t)rx_buffer[5] << 8 | (int16_t)rx_buffer[4])) / 32768.0f * 180.0f;
                    yaw   = ((int16_t)((int16_t)rx_buffer[7] << 8 | (int16_t)rx_buffer[6])) / 32768.0f * 180.0f;
                }
                
                else if (expected_packet_type == ACCEL_PACKET)
                {
                    // 解析加速度：低字节在前，高字节在后，转换为有符号16位整数
                    // 量程±16g，原始值范围-32768~32767对应±16g
                    accel_x = ((int16_t)((int16_t)rx_buffer[3] << 8 | (int16_t)rx_buffer[2])) / 32768.0f * 16.0f * 9.8f;
                    accel_y = ((int16_t)((int16_t)rx_buffer[5] << 8 | (int16_t)rx_buffer[4])) / 32768.0f * 16.0f * 9.8f;
                    accel_z = ((int16_t)((int16_t)rx_buffer[7] << 8 | (int16_t)rx_buffer[6])) / 32768.0f * 16.0f * 9.8f;
                }
                else if (expected_packet_type == GYRO_PACKET)
                {
                    // 解析角速度：低字节在前，高字节在后，转换为有符号16位整数
                    // 量程±2000度/秒，原始值范围-32768~32767对应±2000度/秒
                    gyro_x = ((int16_t)((int16_t)rx_buffer[3] << 8 | (int16_t)rx_buffer[2])) / 32768.0f * 2000.0f;
                    gyro_y = ((int16_t)((int16_t)rx_buffer[5] << 8 | (int16_t)rx_buffer[4])) / 32768.0f * 2000.0f;
                    gyro_z = ((int16_t)((int16_t)rx_buffer[7] << 8 | (int16_t)rx_buffer[6])) / 32768.0f * 2000.0f;
                }
                else if (expected_packet_type == QUAT_PACKET)
                {
                    // 解析四元数：低字节在前，高字节在后，转换为有符号16位整数
                    // 四元数范围：-32768~32767 对应 -1~1
                    quat_q0 = ((int16_t)((int16_t)rx_buffer[3] << 8 | (int16_t)rx_buffer[2])) / 32768.0f;
                    quat_q1 = ((int16_t)((int16_t)rx_buffer[5] << 8 | (int16_t)rx_buffer[4])) / 32768.0f;
                    quat_q2 = ((int16_t)((int16_t)rx_buffer[7] << 8 | (int16_t)rx_buffer[6])) / 32768.0f;
                    quat_q3 = ((int16_t)((int16_t)rx_buffer[9] << 8 | (int16_t)rx_buffer[8])) / 32768.0f;
                }
                
            }
            // 无论校验成功与否，重置状态机，准备下一包
            rx_state = 0;
            rx_index = 0;
            expected_packet_type = PACKET_NONE;
        }
    }
}




/**
 * @brief 获取滚转角
 * 
 * @return float 滚转角（度）
 */
float Class_JY61P::GetRoll() const
{
    float value;
    taskENTER_CRITICAL();
    value = roll;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取俯仰角
 * 
 * @return float 俯仰角（度）
 */
float Class_JY61P::GetPitch() const
{
    float value;
    taskENTER_CRITICAL();
    value = pitch;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取偏航角
 * 
 * @return float 偏航角（度）
 */
float Class_JY61P::GetYaw() const
{
    float value;
    taskENTER_CRITICAL();
    value = yaw;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取X轴加速度
 * 
 * @return float 加速度值（g）
 */
float Class_JY61P::GetAccelX() const
{
    float value;
    taskENTER_CRITICAL();
    value = accel_x;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取Y轴加速度
 * 
 * @return float 加速度值（g）
 */
float Class_JY61P::GetAccelY() const
{
    float value;
    taskENTER_CRITICAL();
    value = accel_y;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取Z轴加速度
 * 
 * @return float 加速度值（g）
 */
float Class_JY61P::GetAccelZ() const
{
    float value;
    taskENTER_CRITICAL();
    value = accel_z;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取X轴角速度
 * 
 * @return float 角速度值（度/秒）
 */
float Class_JY61P::GetGyroX() const
{
    float value;
    taskENTER_CRITICAL();
    value = gyro_x;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取Y轴角速度
 * 
 * @return float 角速度值（度/秒）
 */
float Class_JY61P::GetGyroY() const
{
    float value;
    taskENTER_CRITICAL();
    value = gyro_y;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取Z轴角速度
 * 
 * @return float 角速度值（度/秒）
 */
float Class_JY61P::GetGyroZ() const
{
    float value;
    taskENTER_CRITICAL();
    value = gyro_z;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取四元数q0
 * 
 * @return float 四元数分量
 */
float Class_JY61P::GetQuatQ0() const
{
    float value;
    taskENTER_CRITICAL();
    value = quat_q0;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取四元数q1
 * 
 * @return float 四元数分量
 */
float Class_JY61P::GetQuatQ1() const
{
    float value;
    taskENTER_CRITICAL();
    value = quat_q1;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取四元数q2
 * 
 * @return float 四元数分量
 */
float Class_JY61P::GetQuatQ2() const
{
    float value;
    taskENTER_CRITICAL();
    value = quat_q2;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 获取四元数q3
 * 
 * @return float 四元数分量
 */
float Class_JY61P::GetQuatQ3() const
{
    float value;
    taskENTER_CRITICAL();
    value = quat_q3;
    taskEXIT_CRITICAL();
    return value;
}

/**
 * @brief 批量获取所有姿态数据（线程安全，减少临界区进入次数）
 * 
 * @param data 输出的姿态数据指针
 */
void Class_JY61P::GetAttitudeData(AttitudeData_t *data) const
{
    if (data == nullptr) return;
    
    taskENTER_CRITICAL();
    
    data->roll = roll;
    data->pitch = pitch;
    data->yaw = yaw;
    
    data->accel_x = accel_x;
    data->accel_y = accel_y;
    data->accel_z = accel_z;
    
    data->gyro_x = gyro_x;
    data->gyro_y = gyro_y;
    data->gyro_z = gyro_z;
    
    data->quat_q0 = quat_q0;
    data->quat_q1 = quat_q1;
    data->quat_q2 = quat_q2;
    data->quat_q3 = quat_q3;
    
    taskEXIT_CRITICAL();
}


/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/