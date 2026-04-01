#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "alg_pid.h"

/*
AxisControl 类说明：
轴控制类，将角度误差转换为电机目标速度

功能：
1. 位置控制：目标角度 → 角度误差 → PID → 目标速度
2. 速度控制：目标速度 → 速度误差 → PID → 目标速度（速度闭环）
3. 支持独立设置角度环和速度环的PID参数

注意：
- 位置控制和速度控制是独立的，由调用者决定使用哪个
- 所有公共接口都是线程安全的（内部有互斥锁）
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
     * @param motor_id 电机ID（仅用于标识，不控制电机）
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
    
    // ========== 初始化 ==========
    
    /**
     * @brief 初始化（用于默认构造函数创建的对象）
     */
    void Init(uint8_t motor_id, float control_period_sec, float output_limit = 200.0f);
    
    // ========== 位置控制相关（角度环）==========
    
    /**
     * @brief 设置目标角度
     * @param angle_deg 目标角度（度）
     */
    void SetTargetAngle(float angle_deg);
    
    /**
     * @brief 获取目标角度
     */
    float GetTargetAngle() const;
    
    /**
     * @brief 设置当前角度（从IMU读取）
     */
    void SetCurrentAngle(float angle_deg);
    
    /**
     * @brief 获取当前角度
     */
    float GetCurrentAngle() const;
    
    /**
     * @brief 获取角度误差（目标角度 - 当前角度，已归一化到 [-180, 180]）
     */
    float GetAngleError() const;
    
    /**
     * @brief 角度环PID计算
     * @return 目标速度（度/秒）
     * 
     * 内部流程：
     * 1. 读取目标角度和当前角度
     * 2. 计算归一化角度误差
     * 3. 经过PID计算
     * 4. 返回控制输出
     */
    float CalculateAngleOutput();
    
    // ========== 速度控制相关（速度环）==========
    
    /**
     * @brief 设置目标速度
     * @param speed_dps 目标角速度（度/秒）
     */
    void SetTargetSpeed(float speed_dps);
    
    /**
     * @brief 获取目标速度
     */
    float GetTargetSpeed() const;
    
    /**
     * @brief 设置当前速度（从IMU陀螺仪或电机编码器获取）
     */
    void SetCurrentSpeed(float speed_dps);
    
    /**
     * @brief 获取当前速度
     */
    float GetCurrentSpeed() const;
    
    /**
     * @brief 获取速度误差（目标速度 - 当前速度）
     */
    float GetSpeedError() const;
    
    /**
     * @brief 速度环PID计算
     * @return 目标速度（度/秒）
     * 
     * 内部流程：
     * 1. 读取目标速度和当前速度
     * 2. 计算速度误差
     * 3. 经过PID计算
     * 4. 返回控制输出
     */
    float CalculateSpeedOutput();
    
    // ========== 角度环PID参数 ==========
    
    void SetAnglePID(float kp, float ki, float kd);
    void GetAnglePID(float &kp, float &ki, float &kd) const;
    void SetAngleKp(float kp);
    void SetAngleKi(float ki);
    void SetAngleKd(float kd);
    float GetAngleKp() const;
    float GetAngleKi() const;
    float GetAngleKd() const;
    
    // ========== 速度环PID参数 ==========
    
    void SetSpeedPID(float kp, float ki, float kd);
    void GetSpeedPID(float &kp, float &ki, float &kd) const;
    void SetSpeedKp(float kp);
    void SetSpeedKi(float ki);
    void SetSpeedKd(float kd);
    float GetSpeedKp() const;
    float GetSpeedKi() const;
    float GetSpeedKd() const;
    
    // ========== 通用接口 ==========
    
    /**
     * @brief 重置PID状态（清零积分项和误差）
     */
    void Reset();
    
    /**
     * @brief 获取电机ID
     */
    uint8_t GetMotorID() const { return motor_id_; }
    
    /**
     * @brief 检查是否已初始化
     */
    bool IsInitialized() const { return initialized_; }

private:
    // PID控制器
    Class_PID pid_angle_;   // 角度环PID
    Class_PID pid_speed_;   // 速度环PID
    
    // 标识
    uint8_t motor_id_;
    float control_period_sec_;
    float output_limit_;
    
    // 角度相关数据
    float target_angle_;
    float current_angle_;
    float angle_error_;
    float kp_angle_, ki_angle_, kd_angle_;
    
    // 速度相关数据
    float target_speed_;
    float current_speed_;
    float speed_error_;
    float kp_speed_, ki_speed_, kd_speed_;
    
    // 互斥锁（保护所有成员变量）
    mutable SemaphoreHandle_t mutex_;
    
    // 状态标志
    bool initialized_;
    
    // 内部辅助函数
    static float NormalizeAngleError(float target, float current);
};

#endif // AXIS_CONTROL_H