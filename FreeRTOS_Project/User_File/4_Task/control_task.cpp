
#include "FreeRTOS.h"
#include "task.h"

#include "drv_uart.h"
#include "bsp_control.h"

#include "control_task.h"
#include "rc_angle_speed_task.h"

#include <stdbool.h>
#include <cstdio>

/*
raw.rc.s[0] 表示遥控器从左数第1个的开关，上为1，中为0，下为-1
raw.rc.s[1] 表示遥控器从左数第2个的开关，上为1，中为0，下为-1
raw.rc.s[2] 表示遥控器从左数第3个的开关，上为1，中为0，下为-1
raw.rc.s[3] 表示遥控器从左数第4个的开关，上为1，中为0，下为-1
raw.rc.ch[0]表示遥控器右边摇杆的水平状态，左为负，右为正，范围是783到-779
raw.rc.ch[1]表示遥控器左边摇杆的竖直状态，下为负，上为正，范围是773到-782
raw.rc.ch[2]表示遥控器右边摇杆的竖直状态，下为负，上为正，范围是783到-784
raw.rc.ch[3]表示遥控器左边摇杆的水平状态，左为负，右为正，范围是773到-781
raw.rc.ch[4]表示遥控器上方的左边的旋钮，逆时针转减少，顺时针转增加，范围是783到-784
raw.rc.ch[5]表示遥控器上方的右边的旋钮，逆时针转减少，顺时针转增加，范围是783到-784

遥控器控制的话，在遥控器的DISPLAY页面，这个旋钮动20个数，DISPLAY才动一个格子。
*/


// 任务句柄
static TaskHandle_t xControlTaskHandle = NULL;

// ========== 外部引入 ==========
extern void Set_Target_Yaw_RC(float yaw_deg);
extern void Set_Target_Pitch_RC(float pitch_deg);
extern void Set_Target_Roll_RC(float roll_deg);
extern void Set_Control_Enable_RC(float enable);

// ========== 静态变量 ==========
static int16_t last_knob_yaw = 0;   // 记录上次值，用于死区判断
static int16_t last_knob_roll = 0;



// ========== 旋钮值映射为目标角度的函数 ==========
/**
 * @brief 将遥控器旋钮原始值映射为目标角度
 * @param knob_value 旋钮原始值（范围：RC_KNOB_MIN_VALUE ~ RC_KNOB_MAX_VALUE）
 * @return 目标角度（度），范围：TARGET_ANGLE_MIN ~ TARGET_ANGLE_MAX
 * 
 * 映射规则：
 * - 死区范围内（-RC_KNOB_DEADZONE ~ RC_KNOB_DEADZONE）返回 0°
 * - 正值区域（DEADZONE ~ MAX）线性映射到 0° ~ MAX_ANGLE
 * - 负值区域（MIN ~ -DEADZONE）线性映射到 MIN_ANGLE ~ 0°
 */
static float MapKnobToAngle(int16_t knob_value)
{
    float angle = 0.0f;
    
    // 死区处理：在死区范围内直接返回0
    if (knob_value >= -RC_KNOB_DEADZONE && knob_value <= RC_KNOB_DEADZONE)
    {
        return 0.0f;
    }
    
    // 正值区域映射
    if (knob_value > RC_KNOB_DEADZONE)
    {
        // 限制最大值
        int16_t clamped = (knob_value > RC_KNOB_MAX_VALUE) ? RC_KNOB_MAX_VALUE : knob_value;
        // 线性映射：死区上限 -> 0°，最大值 -> TARGET_ANGLE_MAX
        float ratio = (float)(clamped - RC_KNOB_DEADZONE) / (RC_KNOB_MAX_VALUE - RC_KNOB_DEADZONE);
        angle = ratio * TARGET_ANGLE_MAX;
    }
    // 负值区域映射
    else if (knob_value < -RC_KNOB_DEADZONE)
    {
        // 限制最小值
        int16_t clamped = (knob_value < RC_KNOB_MIN_VALUE) ? RC_KNOB_MIN_VALUE : knob_value;
        // 线性映射：死区下限 -> 0°，最小值 -> TARGET_ANGLE_MIN
        float ratio = (float)(clamped + RC_KNOB_DEADZONE) / (RC_KNOB_MIN_VALUE + RC_KNOB_DEADZONE);
        angle = ratio * TARGET_ANGLE_MIN;
    }
    
    return angle;
}





static void vControlTask(void *pvParameters)
{
    Control.Init(&huart5, 100);
    static uint32_t last_print = 0;
    while (1)
    {
        Control.CheckOffline();
        uint32_t now = HAL_GetTick();


        if (!Control.IsOffline() && (now - last_print > 100)) 
        {
            last_print = now;
            // 获取原始数据
            const auto &raw = Control.GetDataCopy();

            // 获取旋钮原始值
            int16_t knob_yaw = raw.rc.ch[4];   // 控制Yaw（水平）
            int16_t knob_roll = raw.rc.ch[5];  // 控制Roll（俯仰）
            int16_t switch_enable = raw.rc.s[2];
        
            // 如果变化小于阈值，使用上次的值，防止抖动
            if (abs(knob_yaw - last_knob_yaw) < 5)
            {
                knob_yaw = last_knob_yaw;
            }
            if (abs(knob_roll - last_knob_roll) < 5)
            {
                knob_roll = last_knob_roll;
            }
            last_knob_yaw = knob_yaw;
            last_knob_roll = knob_roll;


            // 映射为目标角度
            float target_yaw = MapKnobToAngle(knob_yaw);
            float target_roll = MapKnobToAngle(knob_roll);

            Set_Target_Yaw_RC(target_yaw);     // 遥控器写入目标yaw角
            Set_Target_Roll_RC(target_roll);

            if(switch_enable == 1)
            {
                float enable_value = 1.0f;
                Set_Control_Enable_RC(enable_value);
            }else if(switch_enable == 0)
            {
                float enable_value = 0.0f;
                Set_Control_Enable_RC(enable_value);
            }else
            {
                float enable_value = -1.0f;
                Set_Control_Enable_RC(enable_value);           
            }

            // // 测试打印开关值、四个摇杆通道值、以及两个旋钮通道值
            // char debug[192];  // 增大缓冲区
            // int dlen = snprintf(debug, sizeof(debug), 
            //     "s=[%d,%d,%d,%d] ch=[%d,%d,%d,%d] vr=[%d,%d]\r\n",
            //     raw.rc.s[0], raw.rc.s[1], raw.rc.s[2], raw.rc.s[3],
            //     raw.rc.ch[0], raw.rc.ch[1], raw.rc.ch[2], raw.rc.ch[3],
            //     raw.rc.ch[4], raw.rc.ch[5]);  // 旋钮 VrA 和 VrB
            // HAL_UART_Transmit(&huart7, (uint8_t*)debug, dlen, HAL_MAX_DELAY);
            
        }




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
