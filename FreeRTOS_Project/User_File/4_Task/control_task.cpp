#include "drv_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_control.h"

#include <stdbool.h>

// 任务句柄
static TaskHandle_t xControlTaskHandle = NULL;

static void vControlTask(void *pvParameters)
{
    Control.Init(&huart5, 100);
    
    while (1)
    {
        Control.CheckOffline();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Control_Task_Create(void)
{
    BaseType_t ret = xTaskCreate(vControlTask,          // 任务函数
                                 "vControlTask",        // 任务名称
                                 256,                   // 栈大小（字）
                                 NULL,                  // 参数
                                 2,                     // 优先级
                                 &xControlTaskHandle);
    if (ret != pdPASS)
    {
        while (1)
        {
           
        }
    }
}
