/**
 * @file bsp_jy61p.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief JY61P陀螺仪BSP，基于UART DMA + 空闲中断
 * @version 0.1
 * @date 2025-08-14
 * 
 * @copyright USTC-RoboWalker (c) 2025
 * 
 */

#ifndef BSP_JY61P_H
#define BSP_JY61P_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "drv_uart.h"

/* Exported types ------------------------------------------------------------*/

// ==================== 包类型枚举 ====================
enum PacketType {
    PACKET_NONE = 0,
    ANGLE_PACKET,   // 0x53 - 角度输出
    ACCEL_PACKET,   // 0x51 - 加速度输出
    GYRO_PACKET,    // 0x52 - 角速度输出
    QUAT_PACKET     // 0x59 - 四元数输出
};

typedef struct
{
    float roll;     // 滚转角（度）
    float pitch;    // 俯仰角（度）
    float yaw;      // 偏航角（度）
    
    float accel_x;  // X轴加速度（m/s²）
    float accel_y;  // Y轴加速度（m/s²）
    float accel_z;  // Z轴加速度（m/s²）
    
    float gyro_x;   // X轴角速度（度/秒）
    float gyro_y;   // Y轴角速度（度/秒）
    float gyro_z;   // Z轴角速度（度/秒）
    
    float quat_q0;  // 四元数分量q0
    float quat_q1;  // 四元数分量q1
    float quat_q2;  // 四元数分量q2
    float quat_q3;  // 四元数分量q3
} AttitudeData_t;

/**
 * @brief JY61P陀螺仪类
 * 
 */
class Class_JY61P
{
public:
    Class_JY61P() = default;
    /**
     * @brief 初始化JY61P，绑定UART并注册回调
     * 
     * @param huart 已初始化的UART句柄
     */
    void Init(UART_HandleTypeDef *huart);

    /**
     * @brief 获取当前滚转角（Roll）
     * 
     * @return float 角度值（度）
     */
    float GetRoll() const;

    /**
     * @brief 获取当前俯仰角（Pitch）
     * 
     * @return float 角度值（度）
     */
    float GetPitch() const;

    /**
     * @brief 获取当前偏航角（Yaw）
     * 
     * @return float 角度值（度）
     */
    float GetYaw() const;


    /**
     * @brief 获取X轴加速度
     * 
     * @return float 加速度值（g）
     */
    float GetAccelX() const;

    /**
     * @brief 获取Y轴加速度
     * 
     * @return float 加速度值（g）
     */
    float GetAccelY() const;

    /**
     * @brief 获取Z轴加速度
     * 
     * @return float 加速度值（g）
     */
    float GetAccelZ() const;

    /**
     * @brief 获取X轴角速度
     * 
     * @return float 角速度值（度/秒）
     */
    float GetGyroX() const;

    /**
     * @brief 获取Y轴角速度
     * 
     * @return float 角速度值（度/秒）
     */
    float GetGyroY() const;

    /**
     * @brief 获取Z轴角速度
     * 
     * @return float 角速度值（度/秒）
     */
    float GetGyroZ() const;

    /**
     * @brief 获取四元数q0
     * 
     * @return float 四元数分量
     */
    float GetQuatQ0() const;

    /**
     * @brief 获取四元数q1
     * 
     * @return float 四元数分量
     */
    float GetQuatQ1() const;

    /**
     * @brief 获取四元数q2
     * 
     * @return float 四元数分量
     */
    float GetQuatQ2() const;

    /**
     * @brief 获取四元数q3
     * 
     * @return float 四元数分量
     */
    float GetQuatQ3() const;

    /**
     * @brief 批量获取所有姿态数据（线程安全，推荐使用）
     * 
     * @param data 输出的姿态数据指针
     */
    void GetAttitudeData(AttitudeData_t *data) const;


private:
    // 禁止拷贝
    Class_JY61P(const Class_JY61P&) = delete;
    Class_JY61P& operator=(const Class_JY61P&) = delete;

    PacketType expected_packet_type;


    // 数据包解析状态机变量
    uint8_t rx_buffer[11];      ///< 接收缓冲区（一个完整数据包）
    uint8_t rx_state;            ///< 状态机状态：0-等待包头，1-等待类型，2-接收数据
    uint8_t rx_index;            ///< 当前接收索引

    // 角度数据（单位：度）
    float roll;
    float pitch;
    float yaw;

    // 加速度数据（单位：g）
    float accel_x;
    float accel_y;
    float accel_z;

    // 角速度数据（单位：度/秒）
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // 四元数数据（无量纲）
    float quat_q0;
    float quat_q1;
    float quat_q2;
    float quat_q3;

    /**
     * @brief 处理接收到的字节流（由UART回调调用）
     * 
     * @param Buffer 数据缓冲区
     * @param Length 数据长度
     */
    void DataProcess(uint8_t *Buffer, uint16_t Length);

    /**
     * @brief 解析单个字节（状态机核心）
     * 
     * @param data 输入的字节
     */
    void ParseByte(uint8_t data);

    /**
     * @brief UART接收回调包装函数（静态，用于注册到UART驱动）
     * 
     * @param Buffer 数据缓冲区
     * @param Length 数据长度
     */
    static void UART_Callback(uint8_t *Buffer, uint16_t Length);
};

/* Exported variables ---------------------------------------------------------*/

/**
 * @brief 全局JY61P对象，用户可直接使用
 * 
 */
extern Class_JY61P BSP_JY61P;

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/