#include "FreeRTOS.h"
#include "task.h"
#include "bsp_ws2812.h"

// 任务句柄
static TaskHandle_t xWS2812TaskHandle = NULL;

// 颜色渐变状态变量
static int32_t red = 0;
static int32_t green = 12;
static int32_t blue = 12;
static bool red_minus_flag = false;
static bool green_minus_flag = false;
static bool blue_minus_flag = true;

/**
 * @brief 彩灯任务函数
 * @param pvParameters 任务参数（未使用）
 */
static void vWS2812Task(void *pvParameters)
{
    BSP_WS2812.Init(0, 0, 0);                               // 初始化彩灯
    SPI_Init(&hspi6, nullptr);                              // WS2812的SPI

    while (1)
    {
        if (red >= 18)
            red_minus_flag = true;
        else if (red == 0)
            red_minus_flag = false;

        if (green >= 18)
            green_minus_flag = true;
        else if (green == 0)
            green_minus_flag = false;

        if (blue >= 18)
            blue_minus_flag = true;
        else if (blue == 0)
            blue_minus_flag = false;

        if (red_minus_flag)
            red--;
        else
            red++;

        if (green_minus_flag)
            green--;
        else
            green++;

        if (blue_minus_flag)
            blue--;
        else
            blue++;

        BSP_WS2812.Set_RGB(red, green, blue);               // 设置新颜色
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();  // 通过 SPI 发送数据到 WS2812

        vTaskDelay(pdMS_TO_TICKS(10));                      // 等待 10ms，控制更新频率
    }
}

/**
 * @brief 创建彩灯任务
 * @note 应在 FreeRTOS 调度器启动前调用，或由其他任务创建
 */
void WS2812_Task_Create(void)
{
    BaseType_t ret = xTaskCreate(vWS2812Task,   // 任务函数
                                 "vWS2812Task", // 任务名称
                                 256,           // 栈大小（字）
                                 NULL,          // 参数
                                 1,             // 优先级
                                 &xWS2812TaskHandle);
    if (ret != pdPASS)
    {
        while (1)
        {
           
        }
    }
}
