#include "axis_control.h"


// ========== 构造函数 ==========

AxisControl::AxisControl()
    : motor_id_(0)
    , control_period_sec_(0.0f)
    , output_limit_(0.0f)
    , target_angle_(0.0f)
    , current_angle_(0.0f)
    , error_(0.0f)
    , kp_(0.0f)
    , ki_(0.0f)
    , kd_(0.0f)
    , mutex_(NULL)
    , initialized_(false)
{
    // 创建互斥锁
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == NULL) 
    {
        while(1);  // 创建失败，死循环
    }
}

AxisControl::AxisControl(uint8_t motor_id, float control_period_sec, float output_limit)
    : motor_id_(motor_id)
    , control_period_sec_(control_period_sec)
    , output_limit_(output_limit)
    , target_angle_(0.0f)
    , current_angle_(0.0f)
    , error_(0.0f)
    , kp_(0.0f)
    , ki_(0.0f)
    , kd_(0.0f)
    , initialized_(false)
{
    // 创建互斥锁
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == NULL) 
    {
        while(1);
    }
    
    // 自动初始化
    Init(motor_id, control_period_sec, output_limit);
}

AxisControl::~AxisControl() 
{
    if (mutex_ != NULL) 
    {
        vSemaphoreDelete(mutex_);
    }
}

// ========== 初始化函数 ==========

void AxisControl::Init(uint8_t motor_id, float control_period_sec, float output_limit) 
{
    motor_id_ = motor_id;
    control_period_sec_ = control_period_sec;
    output_limit_ = output_limit;
    
    // 初始化PID（使用当前参数，默认为0）
    pid_.Init(kp_, ki_, kd_, 0.0f, 0.0f, output_limit_, control_period_sec_);
    
    initialized_ = true;
}

// ========== 角度设置和读取 ==========

void AxisControl::SetTargetAngle(float angle_deg) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    target_angle_ = angle_deg;
    xSemaphoreGive(mutex_);
}

float AxisControl::GetTargetAngle() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = target_angle_;
    xSemaphoreGive(mutex_);
    return value;
}

void AxisControl::SetCurrentAngle(float angle_deg) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    current_angle_ = angle_deg;
    xSemaphoreGive(mutex_);
}

float AxisControl::GetCurrentAngle() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = current_angle_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetError() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = error_;
    xSemaphoreGive(mutex_);
    return value;
}

// ========== PID参数设置和读取 ==========

void AxisControl::SetPIDParams(float kp, float ki, float kd) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    pid_.Set_K_P(kp);
    pid_.Set_K_I(ki);
    pid_.Set_K_D(kd);
    xSemaphoreGive(mutex_);
}

void AxisControl::GetPIDParams(float &kp, float &ki, float &kd) const 
{
    if (!initialized_) 
    {
        kp = ki = kd = 0.0f;
        return;
    }
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp = kp_;
    ki = ki_;
    kd = kd_;
    xSemaphoreGive(mutex_);
}

void AxisControl::SetKp(float kp) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp_ = kp;
    pid_.Set_K_P(kp);
    xSemaphoreGive(mutex_);
}

void AxisControl::SetKi(float ki) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ki_ = ki;
    pid_.Set_K_I(ki);
    xSemaphoreGive(mutex_);
}

void AxisControl::SetKd(float kd) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kd_ = kd;
    pid_.Set_K_D(kd);
    xSemaphoreGive(mutex_);
}

float AxisControl::GetKp() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = kp_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetKi() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = ki_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetKd() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = kd_;
    xSemaphoreGive(mutex_);
    return value;
}

// ========== 控制计算 ==========

float AxisControl::CalculateOutput() 
{
    if (!initialized_) return 0.0f;
    
    float target, current, output;
    
    // 读取当前目标角度和实际角度
    xSemaphoreTake(mutex_, portMAX_DELAY);
    target = target_angle_;
    current = current_angle_;
    xSemaphoreGive(mutex_);
    
    // 计算归一化误差
    float error = NormalizeAngleError(target, current);
    
    // 更新误差（供外部读取）
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_ = error;
    xSemaphoreGive(mutex_);
    
    // PID计算
    pid_.Set_Target(0.0f);
    pid_.Set_Now(error);
    pid_.TIM_Calculate_PeriodElapsedCallback();
    output = -pid_.Get_Out();  // 负号根据实际系统调整
    
    return output;
}

void AxisControl::Reset() 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    error_ = 0.0f;
    pid_.Set_Integral_Error(0.0f);
    xSemaphoreGive(mutex_);
}

bool AxisControl::IsEnabled(float enable_threshold) const 
{
    // 这个函数需要外部使能标志，暂时返回true
    return initialized_;
}

// ========== 内部辅助函数 ==========

float AxisControl::NormalizeAngleError(float target, float current) 
{
    float error = target - current;
    
    // 归一化到 [-180, 180]
    if (error > 180.0f) 
    {
        error -= 360.0f;
    }
    if (error < -180.0f) 
    {
        error += 360.0f;
    }
    
    return error;
}