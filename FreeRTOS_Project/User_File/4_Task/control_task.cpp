
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
// 设定目标角度
extern void Set_Target_Yaw_RC(float yaw_deg);
extern void Set_Target_Pitch_RC(float pitch_deg);
extern void Set_Target_Roll_RC(float roll_deg);

//设定目标速度
extern void Set_Target_Speed_Yaw_RC(float yaw_speed);
extern void Set_Target_Speed_Pitch_RC(float pitch_speed);
extern void Set_Target_Speed_Roll_RC(float roll_speed);

//设置模式
extern void Set_Control_Enable_RC(float enable);

// ========== 静态变量 ==========
static int16_t last_knob_yaw = 0;      // 上次旋钮值（Yaw角度）
static int16_t last_knob_roll = 0;     // 上次旋钮值（Roll角度）
static int16_t last_stick_yaw = 0;     // 上次摇杆值（Yaw角速度）
static int16_t last_stick_roll = 0;    // 上次摇杆值（Roll角速度）



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


// ========== Yaw摇杆值映射为目标角速度 ==========
/**
 * @brief 将Yaw摇杆原始值映射为目标角速度
 * @param stick_value 摇杆原始值（范围：RC_STICK_MIN_VALUE ~ RC_STICK_MAX_VALUE）
 * @return 目标角速度（度/秒 （°/s）），范围：TARGET_YAW_SPEED_MIN ~ TARGET_YAW_SPEED_MAX
 */
static float MapStickToYawSpeed(int16_t stick_value)
{
    float speed = 0.0f;
    
    // 死区处理
    if (stick_value >= -RC_STICK_DEADZONE && stick_value <= RC_STICK_DEADZONE)
    {
        return 0.0f;
    }
    
    // 正值区域映射
    if (stick_value > RC_STICK_DEADZONE)
    {
        int16_t clamped = (stick_value > RC_STICK_MAX_VALUE) ? RC_STICK_MAX_VALUE : stick_value;
        float ratio = (float)(clamped - RC_STICK_DEADZONE) / (RC_STICK_MAX_VALUE - RC_STICK_DEADZONE);
        speed = ratio * TARGET_YAW_SPEED_MAX;
    }
    // 负值区域映射
    else if (stick_value < -RC_STICK_DEADZONE)
    {
        int16_t clamped = (stick_value < RC_STICK_MIN_VALUE) ? RC_STICK_MIN_VALUE : stick_value;
        float ratio = (float)(clamped + RC_STICK_DEADZONE) / (RC_STICK_MIN_VALUE + RC_STICK_DEADZONE);
        speed = ratio * TARGET_YAW_SPEED_MIN;
    }
    
    return speed;
}

// ========== Roll摇杆值映射为目标角速度 ==========
/**
 * @brief 将Roll摇杆原始值映射为目标角速度
 * @param stick_value 摇杆原始值（范围：RC_STICK_MIN_VALUE ~ RC_STICK_MAX_VALUE）
 * @return 目标角速度（度/秒（°/s）），范围：TARGET_ROLL_SPEED_MIN ~ TARGET_ROLL_SPEED_MAX
 */
static float MapStickToRollSpeed(int16_t stick_value)
{
    float speed = 0.0f;
    
    // 死区处理
    if (stick_value >= -RC_STICK_DEADZONE && stick_value <= RC_STICK_DEADZONE)
    {
        return 0.0f;
    }
    
    // 正值区域映射
    if (stick_value > RC_STICK_DEADZONE)
    {
        int16_t clamped = (stick_value > RC_STICK_MAX_VALUE) ? RC_STICK_MAX_VALUE : stick_value;
        float ratio = (float)(clamped - RC_STICK_DEADZONE) / (RC_STICK_MAX_VALUE - RC_STICK_DEADZONE);
        speed = ratio * TARGET_ROLL_SPEED_MAX;
    }
    // 负值区域映射
    else if (stick_value < -RC_STICK_DEADZONE)
    {
        int16_t clamped = (stick_value < RC_STICK_MIN_VALUE) ? RC_STICK_MIN_VALUE : stick_value;
        float ratio = (float)(clamped + RC_STICK_DEADZONE) / (RC_STICK_MIN_VALUE + RC_STICK_DEADZONE);
        speed = ratio * TARGET_ROLL_SPEED_MIN;
    }
    
    return speed;
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

            // ========== 获取旋钮原始值（角度控制）==========
            int16_t knob_yaw = raw.rc.ch[4];   // 控制Yaw（水平）
            int16_t knob_roll = raw.rc.ch[5];  // 控制Roll（俯仰）

            // ========== 获取摇杆原始值（角速度控制）==========
            int16_t stick_yaw = raw.rc.ch[0];      // 右边摇杆水平（Yaw角速度）
            int16_t stick_roll = raw.rc.ch[1];     // 右边摇杆竖直（Roll角速度）

            int16_t switch_enable = raw.rc.s[2];
        
            // ========== 旋钮防抖滤波 ==========
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

            // ========== 摇杆防抖滤波 ==========
            if (abs(stick_yaw - last_stick_yaw) < 5)
            {
                stick_yaw = last_stick_yaw;
            }
            if (abs(stick_roll - last_stick_roll) < 5)
            {
                stick_roll = last_stick_roll;
            }
            last_stick_yaw = stick_yaw;
            last_stick_roll = stick_roll;

            // 映射为目标角度
            float target_yaw = MapKnobToAngle(knob_yaw);
            float target_roll = MapKnobToAngle(knob_roll);

            // 映射为目标角速度
            float target_yaw_speed_dps = MapStickToYawSpeed(stick_yaw);
            float target_yaw_speed_rad = target_yaw_speed_dps * 0.01745329252f; //（先得到 °/s，再转 rad/s）

            float target_roll_speed_dps = MapStickToRollSpeed(stick_roll);
            float target_roll_speed_rad = target_roll_speed_dps * 0.01745329252f;

            Set_Target_Speed_Yaw_RC(target_yaw_speed_rad);
            Set_Target_Speed_Roll_RC(target_roll_speed_rad);


            Set_Target_Yaw_RC(target_yaw);     // 遥控器写入目标yaw角
            Set_Target_Roll_RC(target_roll);



            if(switch_enable == 1)
            {
                float enable_value = 1.0f;
                Set_Control_Enable_RC(enable_value);
            }else if(switch_enable == 0)        // 旋钮控制状态
            {
                float enable_value = 0.0f;
                Set_Control_Enable_RC(enable_value);
            }else                               // 摇杆速度控制
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
