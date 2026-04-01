#include "bsp_control.h"
#include "drv_uart.h"
#include <cstring>

// 全局对象
Class_Control Control;

void Class_Control::Init(UART_HandleTypeDef *huart, uint32_t offline_ms)
{
    UART_Handler = huart;
    OfflineThreshold = offline_ms;
    LastReceiveTick = xTaskGetTickCount();
    OfflineFlag = true;
    memset(&Data, 0, sizeof(Data));

    UART_Init(huart, UART_RxCallback);
}

void Class_Control::UART_RxCallback(uint8_t *Buffer, uint16_t Length)
{
    // 只处理完整的一帧（25字节）
    if (Length == 25)
    {
        // 中断临界区保护，防止任务打断数据写入
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        Control.ParseI6x(Buffer);
        Control.LastReceiveTick = xTaskGetTickCountFromISR();

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }
}

void Class_Control::CheckOffline()
{
    uint32_t now = xTaskGetTickCount();
    bool new_flag = ((now - LastReceiveTick) > OfflineThreshold) ? true : false;

    // 保护 OfflineFlag 的写操作
    taskENTER_CRITICAL();
    OfflineFlag = new_flag;
    taskEXIT_CRITICAL();
}

Class_Control::Data_t Class_Control::GetDataCopy() const
{
    Data_t copy;
    taskENTER_CRITICAL();
    copy = Data;                     // 结构体拷贝，原子性由临界区保证
    taskEXIT_CRITICAL();
    return copy;
}

bool Class_Control::IsOffline() const
{
    bool flag;
    taskENTER_CRITICAL();
    flag = OfflineFlag;
    taskEXIT_CRITICAL();
    return flag;
}

/**
 * @brief i6x 遥控器 SBUS 数据解析（25 字节），不兼容 DT7，直接使用原始值
 * @param buf 原始数据缓冲区指针
 */
void Class_Control::ParseI6x(uint8_t *buf)
{
    // 起始字节和结束字节检查
    if (buf[0] != 0x0F || buf[24] != 0x00) 
    {
        return;
    }

    // 原始通道解包（减去中值 1024）
    Data.rc.ch[0] = (int16_t)(((buf[1] | (buf[2] << 8)) & 0x07FF) - 1024);
    Data.rc.ch[1] = (int16_t)((((buf[2] >> 3) | (buf[3] << 5)) & 0x07FF) - 1024);
    Data.rc.ch[2] = (int16_t)((((buf[3] >> 6) | (buf[4] << 2) | (buf[5] << 10)) & 0x07FF) - 1024);
    Data.rc.ch[3] = (int16_t)((((buf[5] >> 1) | (buf[6] << 7)) & 0x07FF) - 1024);
    Data.rc.ch[4] = (int16_t)((((buf[6] >> 4) | (buf[7] << 4)) & 0x07FF) - 1024);
    Data.rc.ch[5] = (int16_t)((((buf[7] >> 7) | (buf[8] << 1) | (buf[9] << 9)) & 0x07FF) - 1024);

    // 原始开关解包并转换为 -1/0/1
    int16_t raw_sw[4];
    raw_sw[0] = (int16_t)((((buf[9] >> 2) | (buf[10] << 6)) & 0x07FF) - 1024);
    raw_sw[1] = (int16_t)((((buf[10] >> 5) | (buf[11] << 3)) & 0x07FF) - 1024);
    raw_sw[2] = (int16_t)(((buf[12] | (buf[13] << 8)) & 0x07FF) - 1024);
    raw_sw[3] = (int16_t)((((buf[13] >> 3) | (buf[14] << 5)) & 0x07FF) - 1024);

    auto to_stick = [](int16_t v) -> int8_t 
    {
        return (int8_t)((v < 0) - (v > 0));
    };

    Data.rc.s[0] = to_stick(raw_sw[0]);
    Data.rc.s[1] = to_stick(raw_sw[1]);
    Data.rc.s[2] = to_stick(raw_sw[2]);
    Data.rc.s[3] = to_stick(raw_sw[3]);

    // 标志位
    uint8_t flag = buf[23];
    Data.frame_lost = (flag >> 2) & 0x01;
    Data.failsafe   = (flag >> 3) & 0x01;
}
