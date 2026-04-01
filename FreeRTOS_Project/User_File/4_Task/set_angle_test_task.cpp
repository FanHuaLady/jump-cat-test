#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "alg_pid.h"
#include "bsp_jy61p.h"

#include "set_angle_test_task.h"
#include "motor_dm_task.h"
#include "jy61p_task.h"
#include "vofa_task.h"
#include "Control/axis_control.h"  // 添加 AxisControl 类

/*
本任务主要测试功能是：
实现通过VOFA上位机来设定目标yaw角，之后与JY61P测得的yaw角做差。角度误差（°）输入给PID,输出目标速度（rad/s）
之后把这个目标速度输入给电机的MIT模式移动到目标位置

那么文件的任务就是：
在vofa_task.cpp任务中实现上位机更改目标yaw角，

目标yaw角在该文件中入出，然后将JY61P测得的yaw角在该文件也引入
在该文件实现角度误差（°）输入给PID,输出目标速度（rad/s）

输出的目标速度从该文件引出，到motor_dm_task.cpp中引入

改进： 封装好AxisControl轴类，使用轴类来代替重复的引入引出操作

*/


// JY61P的队列引用
extern QueueHandle_t xIMUDataQueue;

// motor_dm_task的目标速度设置
extern void Motor_DM_Set_Speed(uint8_t motor_id, float speed_rad_s);

// ========== 使用 AxisControl 类创建轴控制对象 ==========
// 先声明对象（使用默认构造函数）
static AxisControl axisYaw;     // Yaw轴（水平电机，使用yaw角）
static AxisControl axisRoll;    // Roll轴（竖直电机，使用roll角）
static AxisControl axisPitch;   // Pitch轴（只记录，不控制电机）

// ========== 任务句柄 ==========
static TaskHandle_t xSetAngleTaskHandle = NULL;

// ========== 互斥锁 ==========
// 现在只需要保护使能标志（其他锁由 AxisControl 内部管理）
static SemaphoreHandle_t xEnableMutex = NULL;

// ========== 使能标志变量 ==========
static float control_enable = 0.0f;

// ========== 测试变量 ==========
static float test_error_yaw = 0.0f;
static float test_error_pitch = 0.0f;
static float test_error_roll = 0.0f;

// ========== 使能标志的读写函数 ==========
static void Set_Enable_Param(float *use_value, float value)
{
    xSemaphoreTake(xEnableMutex, portMAX_DELAY);
    *use_value = value;
    xSemaphoreGive(xEnableMutex);
}

static float Get_Enable_Param(float *use_value)
{
    float value;
    xSemaphoreTake(xEnableMutex, portMAX_DELAY);
    value = *use_value;
    xSemaphoreGive(xEnableMutex);
    return value;
}

// ========== 添加使能外部接口（供VOFA调用）==========
void Set_Control_Enable(float enable)
{
    Set_Enable_Param(&control_enable, enable);
}

// ========== 外部接口（VOFA读取使能标志）==========
float Get_Control_Enable(void)
{
    return Get_Enable_Param(&control_enable);
}

// ========== 内部接口（读取使能标志）==========
static bool Get_Control_Enable_IN(void)
{
    return (Get_Control_Enable() > 0);
}

// ========== 外部接口（VOFA写入目标角度）==========
// 这些接口现在调用 AxisControl 的方法
void Set_Target_Yaw(float yaw_deg)
{
    axisYaw.SetTargetAngle(yaw_deg);
}

void Set_Target_Pitch(float pitch_deg)
{
    axisPitch.SetTargetAngle(pitch_deg);
}

void Set_Target_Roll(float roll_deg)
{
    axisRoll.SetTargetAngle(roll_deg);
}

// ========== 外部接口（VOFA写入 PID 参数）==========
// Yaw轴 PID 参数设置（角度环）
void Set_Angle_PID_Kp_X(float kp)
{
    axisYaw.SetAngleKp(kp);     
}

void Set_Angle_PID_Ki_X(float ki)
{
    axisYaw.SetAngleKi(ki);      
}

void Set_Angle_PID_Kd_X(float kd)
{
    axisYaw.SetAngleKd(kd);      
}

// Roll轴 PID 参数设置（角度环）
void Set_Angle_PID_Kp_Y(float kp)
{
    axisRoll.SetAngleKp(kp);    
}

void Set_Angle_PID_Ki_Y(float ki)
{
    axisRoll.SetAngleKi(ki);     
}

void Set_Angle_PID_Kd_Y(float kd)
{
    axisRoll.SetAngleKd(kd);     
}

// ========== 测试外部接口（供VOFA读取 PID 参数）==========
// Yaw轴 PID 参数读取（角度环）
float Test_Get_PID_Kp_X(void)
{
    return axisYaw.GetAngleKp();  
}

float Test_Get_PID_Ki_X(void)
{
    return axisYaw.GetAngleKi();   
}

float Test_Get_PID_Kd_X(void)
{
    return axisYaw.GetAngleKd();   
}

// Roll轴 PID 参数读取（角度环）
float Test_Get_PID_Kp_Y(void)
{
    return axisRoll.GetAngleKp();  
}

float Test_Get_PID_Ki_Y(void)
{
    return axisRoll.GetAngleKi(); 
}

float Test_Get_PID_Kd_Y(void)
{
    return axisRoll.GetAngleKd(); 
}

// ========== 测试外部接口（供VOFA读取目标角度）==========
float Test_Get_Target_Yaw(void)
{
    return axisYaw.GetTargetAngle();
}

float Test_Get_Target_Pitch(void)
{
    return axisPitch.GetTargetAngle();
}

float Test_Get_Target_Roll(void)
{
    return axisRoll.GetTargetAngle();
}

// ========== 测试外部接口（供VOFA读取当前角度和误差）==========
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

// ========== 获取当前角度（内部使用）==========
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
}

// ========== 任务函数 ==========
static void vSetAngleTask(void *pvParameters)
{
    const TickType_t period_ms = pdMS_TO_TICKS(10);
    
    float local_yaw, local_pitch, local_roll;

    for(;;)
    {
        // 获取当前角度
        Get_Current_Angles(&local_yaw, &local_pitch, &local_roll);
        
        // 更新各个轴的当前角度
        axisYaw.SetCurrentAngle(local_yaw);      // Yaw轴使用yaw角
        axisRoll.SetCurrentAngle(local_roll);    // Roll轴使用roll角（控制电机）
        axisPitch.SetCurrentAngle(local_pitch);  // Pitch轴只记录，不控制电机
        
        // 更新测试误差（供VOFA读取）
        test_error_yaw = axisYaw.GetAngleError();   
        test_error_pitch = axisPitch.GetAngleError(); 
        test_error_roll = axisRoll.GetAngleError();   
        
        // 检查使能标志
        bool en_flag = Get_Control_Enable_IN();
        
        if(en_flag)
        {
            // Yaw轴控制（水平电机）
            float output_speed_dps_yaw = axisYaw.CalculateAngleOutput();  
            float output_speed_rad_s_yaw = output_speed_dps_yaw * BASIC_MATH_DEG_TO_RAD;
            Motor_DM_Set_Speed(MOTOR_X, output_speed_rad_s_yaw);
            
            // Roll轴控制（竖直电机，使用roll角）
            float output_speed_dps_roll = axisRoll.CalculateAngleOutput(); 
            float output_speed_rad_s_roll = output_speed_dps_roll * BASIC_MATH_DEG_TO_RAD;
            Motor_DM_Set_Speed(MOTOR_Y, output_speed_rad_s_roll);
            
            // Pitch轴只计算误差，不控制电机（只记录）
            axisPitch.CalculateAngleOutput();   
            
        } else {
            // 停止电机
            Motor_DM_Set_Speed(MOTOR_X, 0.0f);
            Motor_DM_Set_Speed(MOTOR_Y, 0.0f);
        }
        
        vTaskDelay(period_ms);
    }
}

// ========== 任务创建函数 ==========
void Set_Angle_Task_Create(void)
{
    // 创建使能标志互斥锁
    xEnableMutex = xSemaphoreCreateMutex();
    if (xEnableMutex == NULL)
    {
        while (1) { }
    }
    
    // ========== 初始化 AxisControl 对象 ==========
    // Yaw轴（水平电机，控制周期0.1秒，输出限幅200度/秒）
    axisYaw.Init(MOTOR_X, 0.1f, 200.0f);
    
    // Roll轴（竖直电机，控制周期0.1秒，输出限幅200度/秒）
    axisRoll.Init(MOTOR_Y, 0.1f, 200.0f);
    
    // Pitch轴（只记录，不控制电机）
    axisPitch.Init(MOTOR_NONE, 0.1f, 200.0f);
    
    // ========== 设置初始 PID 参数 ==========
    // Yaw轴 PID 参数（角度环）
    axisYaw.SetAnglePID(7.0f, 0.0f, 0.5f);  
    
    // Roll轴 PID 参数（角度环）
    axisRoll.SetAnglePID(7.0f, 0.0f, 0.5f);  
    
    // Pitch轴 PID 参数（虽然不控制电机，但可以设置参数用于记录）
    axisPitch.SetAnglePID(7.0f, 0.0f, 0.5f); 
    
    // ========== 初始化使能标志 ==========
    control_enable = 0.0f;
    
    // ========== 初始化测试变量 ==========
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