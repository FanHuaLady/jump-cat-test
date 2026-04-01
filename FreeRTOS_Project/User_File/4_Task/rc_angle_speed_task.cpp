#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "axis_control.h"           // 引入轴的类
#include <bsp_jy61p.h>

#include "jy61p_task.h"             // 获取当前角度
#include "control_task.h"           // 遥控器控制驱动
#include "motor_dm_task.h"          // 输出给电机
#include "rc_angle_speed_task.h"


/*
该文件实现功能：
通过遥控器实现两种方法控制云台
① 和VOFA上位机一样，通过两个旋钮来设定目标角度，让其追踪到目标角度位置
② 通过摇杆来直接控制目标速度，像控制穿越机一样，只控制yaw,pitch角来动，回到中心不动


①的实现思路：
通过遥控器其中一个开关进行使能。
实现通过遥控器旋钮来设定目标角(这里使用的是yaw角和roll角)，之后与JY61P测得的当前角做差。角度误差（°）输入给PID,输出目标速度（rad/s）
之后把这个目标速度输入给电机的MIT模式移动到目标位置

那么文件的任务就是：
在control_task.cpp任务中实现遥控器更改目标角，

目标角在该文件中入出，然后将JY61P测得的当前角在该文件也引入
在该文件实现角度误差（°）输入给PID,输出目标速度（rad/s）

输出的目标速度从该文件引出，到motor_dm_task.cpp中引入，可以使用轴的Class类


*/


// JY61P的队列引用
extern QueueHandle_t xIMUDataQueue;

// motor_dm_task的目标速度设置
extern void Motor_DM_Set_Speed(uint8_t motor_id, float speed_rad_s);

// ========== 创建独立的轴对象 ==========
static AxisControl axisYaw_RC;     // Yaw轴（水平电机）
static AxisControl axisRoll_RC;    // Roll轴（竖直电机）
static AxisControl axisPitch_RC;   // Pitch轴（只记录）

// ========== 使能标志（需要单独保护，因为不在 AxisControl 内）==========
static float control_enable_rc = 0.0f;
static SemaphoreHandle_t xEnableMutex_RC = NULL;

// ========== 任务句柄 ==========
static TaskHandle_t xRCControlTaskHandle = NULL;

// ========== 使能标志(开始遥控器控制)的通用读写函数 ==========
static void Set_Enable_Param_RC(float *use_value, float value)
{
    xSemaphoreTake(xEnableMutex_RC, portMAX_DELAY);
    *use_value = value;
    xSemaphoreGive(xEnableMutex_RC);
}

static float Get_Enable_Param_RC(float *use_value)
{
    float value;
    xSemaphoreTake(xEnableMutex_RC, portMAX_DELAY);
    value = *use_value;
    xSemaphoreGive(xEnableMutex_RC);
    return value;
}


// ========== 添加使能外部接口（供VOFA调用）==========
void Set_Control_Enable_RC(float enable)
{
    Set_Enable_Param_RC(&control_enable_rc, enable);
}

// ========== 内部接口（读取使能标志）==========
static float Get_Control_Enable_IN_RC(void)
{
    return Get_Enable_Param_RC(&control_enable_rc);
}


// ========== 外部接口（遥控器写入目标角度）==========
void Set_Target_Yaw_RC(float yaw_deg)
{
    axisYaw_RC.SetTargetAngle(yaw_deg);
}

void Set_Target_Pitch_RC(float pitch_deg)
{
    axisPitch_RC.SetTargetAngle(pitch_deg);
}

void Set_Target_Roll_RC(float roll_deg)
{
    axisRoll_RC.SetTargetAngle(roll_deg);
}

// ========== 外部接口（遥控器写入目标角速度）==========
void Set_Target_Speed_Yaw_RC(float yaw_speed)
{
    axisYaw_RC.SetTargetSpeed(yaw_speed);
}

void Set_Target_Speed_Pitch_RC(float pitch_speed)
{
    axisPitch_RC.SetTargetSpeed(pitch_speed);
}

void Set_Target_Speed_Roll_RC(float roll_speed)
{
    axisRoll_RC.SetTargetSpeed(roll_speed);
}

// ========== 外部接口（VOFA读出目标角度）==========
float Test_Get_Target_Yaw_RC(void)
{
    return axisYaw_RC.GetTargetAngle();
}

float Test_Get_Target_Pitch_RC(void)
{
    return axisPitch_RC.GetTargetAngle();
}

float Test_Get_Target_Roll_RC(void)
{
    return axisRoll_RC.GetTargetAngle();
}





// ========== 获取当前角度 ==========
static void Get_Current_Angles(float *yaw, float *pitch, float *roll)
{
    static float last_yaw = 0.0f;
    static float last_pitch = 0.0f;
    static float last_roll = 0.0f;
    
    AttitudeData_t imu_data_rc;
    
    if (xQueuePeek(xIMUDataQueue, &imu_data_rc, 0) == pdTRUE)
    {
        last_yaw = imu_data_rc.yaw;
        last_pitch = imu_data_rc.pitch;
        last_roll = imu_data_rc.roll;


    }
    
    *yaw = last_yaw;
    *pitch = last_pitch;
    *roll = last_roll;

}
// ========== 获取当前角速度 ==========
static void Get_Current_Speed(float *yaw_speed, float *pitch_speed, float *roll_speed)
{
    static float last_yaw_speed = 0.0f;
    static float last_pitch_speed = 0.0f;
    static float last_roll_speed = 0.0f;
    
    AttitudeData_t imu_data_rc;
    
    if (xQueuePeek(xIMUDataQueue, &imu_data_rc, 0) == pdTRUE)
    {
        last_yaw_speed = imu_data_rc.gyro_z * 0.01745329252f;
        last_pitch_speed = imu_data_rc.gyro_y * 0.01745329252f;
        last_roll_speed = imu_data_rc.gyro_x * 0.01745329252f;


    }
    
    *yaw_speed = last_yaw_speed;
    *pitch_speed = last_pitch_speed;
    *roll_speed = last_roll_speed;

}



static void vRCControlTask(void *pvParameters)
{
    const TickType_t period_ms = pdMS_TO_TICKS(10);
    float local_yaw, local_pitch, local_roll;
    float local_yaw_speed, local_pitch_speed, local_roll_speed;


    while (1)
    {
        // 获取当前角度
        Get_Current_Angles(&local_yaw, &local_pitch, &local_roll);
        // 获取当前角速度
        Get_Current_Speed(&local_yaw_speed,&local_pitch_speed,&local_roll_speed);

        // 更新各个轴的当前角度
        axisYaw_RC.SetCurrentAngle(local_yaw);      // Yaw轴使用yaw角
        axisRoll_RC.SetCurrentAngle(local_roll);    // Roll轴使用roll角（控制电机）
        axisPitch_RC.SetCurrentAngle(local_pitch);  // Pitch轴只记录，不控制电机

        // 更新各个轴的当前角速度
        axisYaw_RC.SetCurrentSpeed(local_yaw_speed);
        axisRoll_RC.SetCurrentSpeed(local_roll_speed);
        axisPitch_RC.SetCurrentSpeed(local_pitch_speed);
        
        float en_flag = Get_Control_Enable_IN_RC();

        if(en_flag == 0)        // 使能开关是0则使用目标角度控制
        {
            // Yaw轴控制（水平电机）
            float output_speed_dps_yaw = axisYaw_RC.CalculateAngleOutput();
            float output_speed_rad_s_yaw = output_speed_dps_yaw * BASIC_MATH_DEG_TO_RAD;
            Motor_DM_Set_Speed(MOTOR_X, output_speed_rad_s_yaw);
            
            // Roll轴控制（竖直电机，使用roll角）
            float output_speed_dps_roll = axisRoll_RC.CalculateAngleOutput();
            float output_speed_rad_s_roll = output_speed_dps_roll * BASIC_MATH_DEG_TO_RAD;
            Motor_DM_Set_Speed(MOTOR_Y, output_speed_rad_s_roll);

            // Pitch轴只计算误差，不控制电机（只记录）
            // axisPitch_RC.CalculateOutput();  

        }else if(en_flag == -1)     // 使能开关是-1则使用目标角速度控制
        {
            float output_speed_rad_s_yaw_gyro = axisYaw_RC.CalculateSpeedOutput();
            Motor_DM_Set_Speed(MOTOR_X, output_speed_rad_s_yaw_gyro);

            float output_speed_rad_s_roll_gyro = axisRoll_RC.CalculateSpeedOutput();
            Motor_DM_Set_Speed(MOTOR_Y, output_speed_rad_s_roll_gyro);

            // Pitch轴只计算误差，不控制电机（只记录）
            // axisPitch_RC.CalculateSpeedOutput();  

        }else
        {

            // 停止电机
            Motor_DM_Set_Speed(MOTOR_X, 0.0f);
            Motor_DM_Set_Speed(MOTOR_Y, 0.0f);
        }

        vTaskDelay(period_ms);
    }
}




void RC_Angle_Speed_Task_Create(void)
{
    // 创建使能标志互斥锁
    xEnableMutex_RC = xSemaphoreCreateMutex();
    if (xEnableMutex_RC == NULL)
    {
        while (1) { }
    }

    // ========== 初始化 AxisControl 对象 ==========
    // Yaw轴（水平电机，控制周期0.1秒，输出限幅200度/秒）
    axisYaw_RC.Init(MOTOR_X, 0.1f, 200.0f);
    
    // Roll轴（竖直电机，控制周期0.1秒，输出限幅200度/秒）
    axisRoll_RC.Init(MOTOR_Y, 0.1f, 200.0f);
    
    // Pitch轴（只记录，不控制电机）
    axisPitch_RC.Init(MOTOR_NONE, 0.1f, 200.0f);
    
    // ========== 设置初始 PID 参数 ==========
    // Yaw轴 PID 参数
    axisYaw_RC.SetAnglePID(6.5f, 0.0f, 0.0f);
    
    // Roll轴 PID 参数
    axisRoll_RC.SetAnglePID(6.5f, 0.0f, 0.0f);
    
    // Pitch轴 PID 参数（虽然不控制电机，但可以设置参数用于记录）
    axisPitch_RC.SetAnglePID(6.0f, 0.0f, 0.0f);

    axisYaw_RC.SetSpeedPID(0.5f, 0.0f, 0.1f);    // Yaw速度环
    axisRoll_RC.SetSpeedPID(0.5f, 0.0f, 0.1f);   // Roll速度环

    // ========== 初始化使能标志 ==========
    control_enable_rc = 1.0f;


    BaseType_t ret = xTaskCreate(vRCControlTask, "vRCControlTask",256,NULL,2,&xRCControlTaskHandle);
    if (ret != pdPASS)
    {
        while (1)
        {
           
        }
    }
}
