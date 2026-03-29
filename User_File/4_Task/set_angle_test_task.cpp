#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "alg_pid.h"
#include "bsp_jy61p.h"

#include "set_angle_test_task.h"
#include "motor_dm_task.h"
#include "jy61p_task.h"
#include "vofa_task.h"

/*
本任务主要测试功能是：
实现通过VOFA上位机来设定目标yaw角，之后与JY61P测得的yaw角做差。角度误差（°）输入给PID,输出目标速度（rad/s）
之后把这个目标速度输入给电机的MIT模式移动到目标位置

那么文件的任务就是：
在vofa_task.cpp任务中实现上位机更改目标yaw角，

目标yaw角在该文件中入出，然后将JY61P测得的yaw角在该文件也引入
在该文件实现角度误差（°）输入给PID,输出目标速度（rad/s）

输出的目标速度从该文件引出，到motor_dm_task.cpp中引入

*/


// JY61P的队列引用
extern QueueHandle_t xIMUDataQueue;

// motor_dm_task的目标速度设置
extern void Motor_DM_Set_Speed(uint8_t motor_id, float speed_rad_s);

// ========== PID对象 ==========
Class_PID Set_Angle_PID;

// ========== 任务句柄 ==========
static TaskHandle_t xSetAngleTaskHandle = NULL;

// ========== 互斥锁 ==========
static SemaphoreHandle_t xTargetMutex = NULL;
// ========== PID 参数互斥锁 ==========
static SemaphoreHandle_t xPIDMutex = NULL;

// ========== PID 参数变量 ==========
static float pid_kp = 0.5f;
static float pid_ki = 0.0f;
static float pid_kd = 0.0f;

// ========== 目标角度变量 ==========
static bool control_enable = false;

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

// ========== 通用角度 读写函数 ==========
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

// ========== 通用 PID 参数读写函数 ==========
static void Set_PID_Param(float *param, float value)
{
    if (xPIDMutex != NULL)
    {
        xSemaphoreTake(xPIDMutex, portMAX_DELAY);
        *param = value;
        xSemaphoreGive(xPIDMutex);
    }
}

static float Get_PID_Param(float *param)
{
    float value;
    if (xPIDMutex != NULL)
    {
        xSemaphoreTake(xPIDMutex, portMAX_DELAY);
        value = *param;
        xSemaphoreGive(xPIDMutex);
    }
    else
    {
        value = *param;
    }
    return value;
}



// ========== 外部接口（VOFA写入）==========
void Set_Target_Yaw(float yaw_deg)
{
    Set_Target_Angle(&target_yaw, yaw_deg);
    control_enable = true;  // 收到命令，使能控制
}

void Set_Target_Pitch(float pitch_deg)
{
    Set_Target_Angle(&target_pitch, pitch_deg);
    control_enable = true;  // 收到命令，使能控制
}

void Set_Target_Roll(float roll_deg)
{
    Set_Target_Angle(&target_roll, roll_deg);
    control_enable = true;  // 收到命令，使能控制
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

// ========== 外部接口（VOFA写入 PID 参数）==========
void Set_Angle_PID_Kp(float kp)
{
    Set_PID_Param(&pid_kp, kp);
    Set_Angle_PID.Set_K_P(kp);
}

void Set_Angle_PID_Ki(float ki)
{
    Set_PID_Param(&pid_ki, ki);
    Set_Angle_PID.Set_K_I(ki);
}

void Set_Angle_PID_Kd(float kd)
{
    Set_PID_Param(&pid_kd, kd);
    Set_Angle_PID.Set_K_D(kd);
}

// ========== 测试接口（供VOFA读取 PID 参数）==========
float Test_Get_PID_Kp(void)
{
    return Get_PID_Param(&pid_kp);
}

float Test_Get_PID_Ki(void)
{
    return Get_PID_Param(&pid_ki);
}

float Test_Get_PID_Kd(void)
{
    return Get_PID_Param(&pid_kd);
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

    float error_yaw_angle,error_pitch_angle,error_roll_angle;  // 记录误差输入给电机用的
    float output_speed_dps;      // PID输出速度（度/秒）
    float output_speed_rad_s;    // 转换后的速度（弧度/秒）
    
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


        error_yaw_angle = Normalize_Angle_Error(target_yaw_local, local_yaw);
        error_pitch_angle = Normalize_Angle_Error(target_pitch_local, local_pitch);
        error_roll_angle = Normalize_Angle_Error(target_roll_local, local_roll);



        if(control_enable)
        {
            Set_Angle_PID.Set_Target(0.0f);
            Set_Angle_PID.Set_Now(error_yaw_angle);


            Set_Angle_PID.TIM_Calculate_PeriodElapsedCallback();

            output_speed_dps = -Set_Angle_PID.Get_Out();
            output_speed_rad_s = output_speed_dps * BASIC_MATH_DEG_TO_RAD;


            Motor_DM_Set_Speed(MOTOR_X, output_speed_rad_s);

        }else
        {
            Motor_DM_Set_Speed(MOTOR_X,0.0f);

        }



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

    // 创建 PID 参数互斥锁
    xPIDMutex = xSemaphoreCreateMutex();
    if (xPIDMutex == NULL)
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

    // 初始化 PID 参数
    pid_kp = 0.5f;
    pid_ki = 0.0f;
    pid_kd = 0.0f;
    
     control_enable = false;

    Set_Angle_PID.Init(pid_kp, pid_ki, pid_kd, 0.0f, 0.0f, 200.0f, 0.1f);  
    // 创建任务
    BaseType_t ret = xTaskCreate(vSetAngleTask, "SetAngleTask", 512, NULL, 2, &xSetAngleTaskHandle);
    if (ret != pdPASS)
    {
        while (1) { }
    }
}