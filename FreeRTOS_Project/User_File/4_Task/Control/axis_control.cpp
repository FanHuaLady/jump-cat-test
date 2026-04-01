#include "axis_control.h"

// ========== 构造函数 ==========

AxisControl::AxisControl()
    : motor_id_(0)
    , control_period_sec_(0.0f)
    , output_limit_(0.0f)
    , target_angle_(0.0f)
    , current_angle_(0.0f)
    , angle_error_(0.0f)
    , kp_angle_(0.0f)
    , ki_angle_(0.0f)
    , kd_angle_(0.0f)
    , target_speed_(0.0f)
    , current_speed_(0.0f)
    , speed_error_(0.0f)
    , kp_speed_(0.0f)
    , ki_speed_(0.0f)
    , kd_speed_(0.0f)
    , mutex_(NULL)
    , initialized_(false)
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == NULL) 
    {
        while(1);
    }
}

AxisControl::AxisControl(uint8_t motor_id, float control_period_sec, float output_limit)
    : motor_id_(motor_id)
    , control_period_sec_(control_period_sec)
    , output_limit_(output_limit)
    , target_angle_(0.0f)
    , current_angle_(0.0f)
    , angle_error_(0.0f)
    , kp_angle_(0.0f)
    , ki_angle_(0.0f)
    , kd_angle_(0.0f)
    , target_speed_(0.0f)
    , current_speed_(0.0f)
    , speed_error_(0.0f)
    , kp_speed_(0.0f)
    , ki_speed_(0.0f)
    , kd_speed_(0.0f)
    , initialized_(false)
{
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == NULL) 
    {
        while(1);
    }
    
    Init(motor_id, control_period_sec, output_limit);
}

AxisControl::~AxisControl() 
{
    if (mutex_ != NULL) 
    {
        vSemaphoreDelete(mutex_);
    }
}

// ========== 初始化 ==========

void AxisControl::Init(uint8_t motor_id, float control_period_sec, float output_limit) 
{
    xSemaphoreTake(mutex_, portMAX_DELAY);
    
    motor_id_ = motor_id;
    control_period_sec_ = control_period_sec;
    output_limit_ = output_limit;
    
    // 初始化角度环PID
    pid_angle_.Init(kp_angle_, ki_angle_, kd_angle_, 0.0f, 0.0f, output_limit_, control_period_sec_);
    
    // 初始化速度环PID
    pid_speed_.Init(kp_speed_, ki_speed_, kd_speed_, 0.0f, 0.0f, output_limit_, control_period_sec_);
    
    initialized_ = true;
    
    xSemaphoreGive(mutex_);
}

// ========== 位置控制相关 ==========

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

float AxisControl::GetAngleError() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = angle_error_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::CalculateAngleOutput() 
{
    if (!initialized_) return 0.0f;
    
    float target, current, output;
    
    // 读取目标角度和当前角度
    xSemaphoreTake(mutex_, portMAX_DELAY);
    target = target_angle_;
    current = current_angle_;
    xSemaphoreGive(mutex_);
    
    // 计算归一化角度误差
    float error = NormalizeAngleError(target, current);
    
    // 更新误差（供外部读取）
    xSemaphoreTake(mutex_, portMAX_DELAY);
    angle_error_ = error;
    xSemaphoreGive(mutex_);
    
    // PID计算
    pid_angle_.Set_Target(0.0f);
    pid_angle_.Set_Now(error);
    pid_angle_.TIM_Calculate_PeriodElapsedCallback();
    output = -pid_angle_.Get_Out();  // 负号根据实际系统调整
    
    return output;
}

// ========== 速度控制相关 ==========

void AxisControl::SetTargetSpeed(float speed_dps) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    target_speed_ = speed_dps;
    xSemaphoreGive(mutex_);
}

float AxisControl::GetTargetSpeed() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = target_speed_;
    xSemaphoreGive(mutex_);
    return value;
}

void AxisControl::SetCurrentSpeed(float speed_dps) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    current_speed_ = speed_dps;
    xSemaphoreGive(mutex_);
}

float AxisControl::GetCurrentSpeed() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = current_speed_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetSpeedError() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = speed_error_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::CalculateSpeedOutput() 
{
    if (!initialized_) return 0.0f;
    
    float target, current, output;
    
    // 读取目标速度和当前速度
    xSemaphoreTake(mutex_, portMAX_DELAY);
    target = target_speed_;
    current = current_speed_;
    xSemaphoreGive(mutex_);
    
    // 计算速度误差
    float error = target - current;
    
    // 更新速度误差（供外部读取）
    xSemaphoreTake(mutex_, portMAX_DELAY);
    speed_error_ = error;
    xSemaphoreGive(mutex_);
    
    // PID计算
    pid_speed_.Set_Target(0.0f);
    pid_speed_.Set_Now(error);
    pid_speed_.TIM_Calculate_PeriodElapsedCallback();
    output = pid_speed_.Get_Out();
    
    // 限幅
    if (output > output_limit_) output = output_limit_;
    if (output < -output_limit_) output = -output_limit_;
    
    return output;
}

// ========== 角度环PID参数 ==========

void AxisControl::SetAnglePID(float kp, float ki, float kd) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp_angle_ = kp;
    ki_angle_ = ki;
    kd_angle_ = kd;
    pid_angle_.Set_K_P(kp);
    pid_angle_.Set_K_I(ki);
    pid_angle_.Set_K_D(kd);
    xSemaphoreGive(mutex_);
}

void AxisControl::GetAnglePID(float &kp, float &ki, float &kd) const 
{
    if (!initialized_) 
    {
        kp = ki = kd = 0.0f;
        return;
    }
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp = kp_angle_;
    ki = ki_angle_;
    kd = kd_angle_;
    xSemaphoreGive(mutex_);
}

void AxisControl::SetAngleKp(float kp) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp_angle_ = kp;
    pid_angle_.Set_K_P(kp);
    xSemaphoreGive(mutex_);
}

void AxisControl::SetAngleKi(float ki) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ki_angle_ = ki;
    pid_angle_.Set_K_I(ki);
    xSemaphoreGive(mutex_);
}

void AxisControl::SetAngleKd(float kd) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kd_angle_ = kd;
    pid_angle_.Set_K_D(kd);
    xSemaphoreGive(mutex_);
}

float AxisControl::GetAngleKp() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = kp_angle_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetAngleKi() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = ki_angle_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetAngleKd() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = kd_angle_;
    xSemaphoreGive(mutex_);
    return value;
}

// ========== 速度环PID参数 ==========

void AxisControl::SetSpeedPID(float kp, float ki, float kd) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp_speed_ = kp;
    ki_speed_ = ki;
    kd_speed_ = kd;
    pid_speed_.Set_K_P(kp);
    pid_speed_.Set_K_I(ki);
    pid_speed_.Set_K_D(kd);
    xSemaphoreGive(mutex_);
}

void AxisControl::GetSpeedPID(float &kp, float &ki, float &kd) const 
{
    if (!initialized_) 
    {
        kp = ki = kd = 0.0f;
        return;
    }
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp = kp_speed_;
    ki = ki_speed_;
    kd = kd_speed_;
    xSemaphoreGive(mutex_);
}

void AxisControl::SetSpeedKp(float kp) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kp_speed_ = kp;
    pid_speed_.Set_K_P(kp);
    xSemaphoreGive(mutex_);
}

void AxisControl::SetSpeedKi(float ki) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    ki_speed_ = ki;
    pid_speed_.Set_K_I(ki);
    xSemaphoreGive(mutex_);
}

void AxisControl::SetSpeedKd(float kd) 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    kd_speed_ = kd;
    pid_speed_.Set_K_D(kd);
    xSemaphoreGive(mutex_);
}

float AxisControl::GetSpeedKp() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = kp_speed_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetSpeedKi() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = ki_speed_;
    xSemaphoreGive(mutex_);
    return value;
}

float AxisControl::GetSpeedKd() const 
{
    if (!initialized_) return 0.0f;
    
    float value;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    value = kd_speed_;
    xSemaphoreGive(mutex_);
    return value;
}

// ========== 通用接口 ==========

void AxisControl::Reset() 
{
    if (!initialized_) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    angle_error_ = 0.0f;
    speed_error_ = 0.0f;
    pid_angle_.Set_Integral_Error(0.0f);
    pid_speed_.Set_Integral_Error(0.0f);
    xSemaphoreGive(mutex_);
}

// ========== 内部辅助函数 ==========

float AxisControl::NormalizeAngleError(float target, float current) 
{
    float error = target - current;
    
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