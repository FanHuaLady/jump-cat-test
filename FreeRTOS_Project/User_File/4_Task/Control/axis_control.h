#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "alg_pid.h"



/*
AxisControl 类说明：
这个类是轴控制类，即云台 角度误差 到 电机目标速度 的类的封装
可以直接使用封装，套用在VOFA和电机调试、遥控器和电机调试
*/
class AxisControl 
{
public:
    /**
     * @brief 默认构造函数（需要后续调用 Init）
     */
    AxisControl();
    
    /**
     * @brief 带参数的构造函数（一步到位）
     * @param motor_id 电机ID
     * @param control_period_sec PID控制周期（秒）
     * @param output_limit 输出限幅（度/秒）
     */
    AxisControl(uint8_t motor_id, float control_period_sec, float output_limit = 200.0f);
    
    /**
     * @brief 析构函数
     */
    ~AxisControl();
    
    // 禁止拷贝（避免互斥锁被复制）
    AxisControl(const AxisControl&) = delete;
    AxisControl& operator=(const AxisControl&) = delete;
    
    // ========== 初始化函数 ==========
    
    /**
     * @brief 初始化（用于默认构造函数创建的对象）
     * @param motor_id 电机ID
     * @param control_period_sec PID控制周期（秒）
     * @param output_limit 输出限幅（度/秒）
     */
    void Init(uint8_t motor_id, float control_period_sec, float output_limit = 200.0f);
    
    // ========== 角度设置和读取 ==========
    
    void SetTargetAngle(float angle_deg);
    float GetTargetAngle() const;
    void SetCurrentAngle(float angle_deg);
    float GetCurrentAngle() const;
    float GetError() const;
    
    // ========== PID参数设置和读取 ==========
    
    void SetPIDParams(float kp, float ki, float kd);
    void GetPIDParams(float &kp, float &ki, float &kd) const;
    void SetKp(float kp);
    void SetKi(float ki);
    void SetKd(float kd);
    float GetKp() const;
    float GetKi() const;
    float GetKd() const;
    
    // ========== 控制计算 ==========
    
    float CalculateOutput();
    void Reset();
    uint8_t GetMotorID() const { return motor_id_; }
    bool IsEnabled(float enable_threshold = 0.5f) const;
    
    // ========== 状态检查 ==========
    
    /**
     * @brief 检查是否已初始化
     */
    bool IsInitialized() const { return initialized_; }

private:
    // 核心组件
    Class_PID pid_;
    uint8_t motor_id_;
    float control_period_sec_;
    float output_limit_;
    
    // 受保护的数据
    float target_angle_;
    float current_angle_;
    float error_;
    float kp_, ki_, kd_;
    
    // 互斥锁
    mutable SemaphoreHandle_t mutex_;
    
    // 状态标志
    bool initialized_;
    
    // 内部辅助函数
    static float NormalizeAngleError(float target, float current);
};

#endif // AXIS_CONTROL_H