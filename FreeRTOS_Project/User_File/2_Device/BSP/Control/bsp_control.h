#ifndef BSP_CONTROL_H
#define BSP_CONTROL_H

#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>

class Class_Control 
{
public:
    // 遥控器数据结构
    struct Data_t 
    {
        struct {
            int16_t ch[6];      // 通道 0~5（0右平，1右竖，2左平，3左竖，4旋钮A，5旋钮B）
            int8_t  s[4];       // 开关 0~3（值 -1/0/1）
        } rc;
        uint8_t failsafe;       // 失控保护标志（1=激活）
        uint8_t frame_lost;     // 丢帧标志（1=丢帧）
    };

    void Init(UART_HandleTypeDef *huart, uint32_t offline_ms = 100);
    void CheckOffline();                                    // 由任务周期性调用，检测掉线状态
    Data_t GetDataCopy(void) const;                         // 线程安全的读取接口
    bool   IsOffline(void) const;                           // 返回掉线标志

private:
    UART_HandleTypeDef *UART_Handler;                       // 串口句柄
    Data_t              Data;                               // 解析后的数据
    uint32_t            LastReceiveTick;                    // 最后接收时间戳 (ms)
    uint32_t            OfflineThreshold;                   // 掉线阈值 (ms)
    bool                OfflineFlag;                        // 当前掉线标志
    
    void ParseI6x(uint8_t *buf);                            // i6x 解析函数
    static void UART_RxCallback(uint8_t *Buffer, uint16_t Length);
};

extern Class_Control Control;

#endif // BSP_CONTROL_H
