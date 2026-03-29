#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "alg_pid.h"
#include "bsp_jy61p.h"

#include "set_angle_test_task.h"
#include "motor_dm_task.h"
#include "jy61p_task.h"
#include "vofa_task.h"

// JY61P的队列引用
extern QueueHandle_t xIMUDataQueue;

// ========== PID对象 ==========
Class_PID Set_Angle_PID;

// ========== 任务句柄 ==========
static TaskHandle_t xSetAngleTaskHandle = NULL;

// ========== 互斥锁 ==========
static SemaphoreHandle_t xTargetMutex = NULL;

// ========== 目标角度变量 ==========
static float target_yaw = 0.0f;
static float target_pitch = 0.0f;
static float target_roll = 0.0f;

// ========== 测试变量（供VOFA读取）==========
static float test_current_yaw = 0.0f;      // 测试用当前角度
static float test_current_pitch = 0.0f;
static float test_current_roll = 0.0f;

static float test_error_yaw = 0.0f;        // 测试用角度误差
static float test_error_pitch = 0.0f;
static float test_error_roll = 0.0f;

// ========== 通用读写函数 ==========
static void Set_Target_Angle(float *target, float value)
{
    xSemaphoreTake(xTargetMutex, portMAX_DELAY);
    *target = value;
    xSemaphoreGive(xTargetMutex);
}

static float Get_Target_Angle(float *target)
{
    float value;
    xSemaphoreTake(xTargetMutex, portMAX_DELAY);
    value = *target;
    xSemaphoreGive(xTargetMutex);
    return value;
}

// ========== 外部接口（VOFA写入）==========
void Set_Target_Yaw(float yaw_deg)
{
    Set_Target_Angle(&target_yaw, yaw_deg);
}

void Set_Target_Pitch(float pitch_deg)
{
    Set_Target_Angle(&target_pitch, pitch_deg);
}

void Set_Target_Roll(float roll_deg)
{
    Set_Target_Angle(&target_roll, roll_deg);
}

// ========== 内部接口（读取目标角度）==========
static float Get_Target_Yaw(void)
{
    return Get_Target_Angle(&target_yaw);
}

static float Get_Target_Pitch(void)
{
    return Get_Target_Angle(&target_pitch);
}

static float Get_Target_Roll(void)
{
    return Get_Target_Angle(&target_roll);
}

// ========== 测试接口（供VOFA读取）==========
float Test_Get_Target_Yaw(void)
{
    return Get_Target_Angle(&target_yaw);
}

float Test_Get_Target_Pitch(void)
{
    return Get_Target_Angle(&target_pitch);
}

float Test_Get_Target_Roll(void)
{
    return Get_Target_Angle(&target_roll);
}

float Test_Get_Current_Yaw(void)
{
    return test_current_yaw;
}

float Test_Get_Current_Pitch(void)
{
    return test_current_pitch;
}

float Test_Get_Current_Roll(void)
{
    return test_current_roll;
}

float Test_Get_Error_Yaw(void)
{
    return test_error_yaw;
}

float Test_Get_Error_Pitch(void)
{
    return test_error_pitch;
}

float Test_Get_Error_Roll(void)
{
    return test_error_roll;
}

// ========== 角度误差归一化 ==========
static float Normalize_Angle_Error(float target, float current)
{
    float error = target - current;
    
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;
    
    return error;
}

// ========== 获取当前角度 ==========
static void Get_Current_Angles(float *yaw, float *pitch, float *roll)
{
    static float last_yaw = 0.0f;
    static float last_pitch = 0.0f;
    static float last_roll = 0.0f;
    
    AttitudeData_t imu_data;
    
    if (xQueuePeek(xIMUDataQueue, &imu_data, 0) == pdTRUE)
    {
        last_yaw = imu_data.yaw;
        last_pitch = imu_data.pitch;
        last_roll = imu_data.roll;
    }
    
    *yaw = last_yaw;
    *pitch = last_pitch;
    *roll = last_roll;
    
    // 更新测试变量
    test_current_yaw = last_yaw;
    test_current_pitch = last_pitch;
    test_current_roll = last_roll;
}

// ========== 任务函数 ==========
static void vSetAngleTask(void *pvParameters)
{
    const TickType_t period_ms = pdMS_TO_TICKS(100);
    
    float local_yaw, local_pitch, local_roll;
    float target_yaw_local, target_pitch_local, target_roll_local;
    
    for(;;)
    {
        // 获取当前角度
        Get_Current_Angles(&local_yaw, &local_pitch, &local_roll);

        // 读取目标角度
        target_yaw_local = Get_Target_Yaw();
        target_pitch_local = Get_Target_Pitch();
        target_roll_local = Get_Target_Roll();

        // 计算角度误差（vofa更新测试变量使用）
        test_error_yaw = Normalize_Angle_Error(target_yaw_local, local_yaw);
        test_error_pitch = Normalize_Angle_Error(target_pitch_local, local_pitch);
        test_error_roll = Normalize_Angle_Error(target_roll_local, local_roll);

        
        
        vTaskDelay(period_ms);
    }
}

// ========== 任务创建函数 ==========
void Set_Angle_Task_Create(void)
{
    // 创建互斥锁
    xTargetMutex = xSemaphoreCreateMutex();
    if (xTargetMutex == NULL)
    {
        while (1) { }
    }
   
    // 初始化目标角度
    target_yaw = 0.0f;
    target_pitch = 0.0f;
    target_roll = 0.0f;
    
    // 初始化测试变量
    test_current_yaw = 0.0f;
    test_current_pitch = 0.0f;
    test_current_roll = 0.0f;
    test_error_yaw = 0.0f;
    test_error_pitch = 0.0f;
    test_error_roll = 0.0f;
    
    // 创建任务
    BaseType_t ret = xTaskCreate(vSetAngleTask, "SetAngleTask", 512, NULL, 2, &xSetAngleTaskHandle);
    if (ret != pdPASS)
    {
        while (1) { }
    }
}