#include "balance_motor_if.h"

#include <stddef.h>

namespace
{
    BalanceMotorBinding g_joint_binding[BALANCE_JOINT_NUM];
    BalanceMotorBinding g_wheel_binding[BALANCE_WHEEL_NUM];
    
    // 简单的限幅函数，将值限制在 [min_v, max_v] 区间内
    static inline float BalanceClamp(float x, float min_v, float max_v)
    {
        if (x < min_v) return min_v;
        if (x > max_v) return max_v;
        return x;
    }
    
    // 判断电机是否在线（使能状态）
    static inline bool BalanceMotorOnline(const BalanceDmMotor* motor)
    {
        if (motor == nullptr)
        {
            return false;
        }

        return (motor->Get_Status() == Motor_DM_Status_ENABLE);
    }
    
    // 将电机对象的状态填充到反馈结构体
    static inline void BalanceFillFdbFromDm(BalanceMotorFdb* out, const BalanceDmMotor* motor)
    {
        if (out == nullptr)
        {
            return;
        }

        if (motor == nullptr)
        {
            out->pos = 0.0f;
            out->vel = 0.0f;
            out->tor = 0.0f;
            out->online = false;
            return;
        }

        out->pos = motor->Get_Now_Angle();                          // 填充角度
        out->vel = motor->Get_Now_Omega();                          // 填充角速度
        out->tor = motor->Get_Now_Torque();                         // 填充力矩
        out->online = BalanceMotorOnline(motor);                    // 判断是否在线
    }
    
    // 将控制指令应用到电机对象
    static inline void BalanceApplyCmdToDm(BalanceDmMotor* motor,
                                           const BalanceMotorCmd* cmd,
                                           float torque_limit,
                                           float kp_limit,
                                           float kd_limit)
    {
        if ((motor == nullptr) || (cmd == nullptr))
        {
            return;
        }

        if (!cmd->enable)
        {
            // 这里不主动发 Exit，只把控制量清零
            // 这样更安全，也避免频繁使能/失能抖动
            motor->Set_Control_Angle(0.0f);
            motor->Set_Control_Omega(0.0f);
            motor->Set_Control_Torque(0.0f);
            motor->Set_K_P(0.0f);
            motor->Set_K_D(0.0f);
            return;
        }

        const float pos = cmd->pos;
        const float vel = cmd->vel;
        const float tor = BalanceClamp(cmd->tor, -torque_limit, torque_limit);  // 限幅
        const float kp  = BalanceClamp(cmd->kp,  0.0f, kp_limit);               // 限幅
        const float kd  = BalanceClamp(cmd->kd,  0.0f, kd_limit);               // 限幅

        motor->Set_Control_Angle(pos);                                          // 设定角度
        motor->Set_Control_Omega(vel);                                          // 设定角速度
        motor->Set_Control_Torque(tor);                                         // 设定力矩
        motor->Set_K_P(kp);                                                     // 设定Kp
        motor->Set_K_D(kd);                                                     // 设定Kd
    }
}

// 将所有 motor 指针置为 nullptr，registered 置为 false
void BalanceMotorIf_Init()
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        g_joint_binding[i].motor = nullptr;
        g_joint_binding[i].registered = false;
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        g_wheel_binding[i].motor = nullptr;
        g_wheel_binding[i].registered = false;
    }
}

// 将关节电机对象绑定到指定索引。检查索引范围和非空指针，成功后设置绑定结构体并返回 true
// 我们需要使用这个函数，将我们的电机进行注册
bool BalanceMotorIf_RegisterJoint(uint8_t index, BalanceDmMotor* motor)
{
    if (index >= BALANCE_JOINT_NUM)
    {
        return false;
    }

    if (motor == nullptr)
    {
        return false;
    }

    g_joint_binding[index].motor = motor;
    g_joint_binding[index].registered = true;
    return true;
}

// 将非关节电机对象绑定到指定索引。检查索引范围和非空指针，成功后设置绑定结构体并返回 true
bool BalanceMotorIf_RegisterWheel(uint8_t index, BalanceDmMotor* motor)
{
    if (index >= BALANCE_WHEEL_NUM)
    {
        return false;
    }

    if (motor == nullptr)
    {
        return false;
    }

    g_wheel_binding[index].motor = motor;
    g_wheel_binding[index].registered = true;
    return true;
}

// 需要传入JumpCat本体
// 遍历所有已注册的电机，调用 BalanceFillFdbFromDm 填充机器人的反馈数组
void BalanceMotorIf_UpdateFeedback(BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        // 遍历已经注册的电机
        if (g_joint_binding[i].registered)
        {
            // 调用这个函数
            // 传入整个机体的关节电机的接收数组
            // 传入我们的电机
            // 这其实算是一个数据转移，前提是电机对上
            
            // 总之这个函数会让robot->joint_motor_fdb[i]
            // 获取角度、角速度、力矩、在线情况
            
            // 注意注册的时候对照事先规定的关节电机索引
            // 这样才能正常的获得数据
            BalanceFillFdbFromDm(&robot->joint_motor_fdb[i], g_joint_binding[i].motor);
        }
        else
        {
            robot->joint_motor_fdb[i].pos = 0.0f;
            robot->joint_motor_fdb[i].vel = 0.0f;
            robot->joint_motor_fdb[i].tor = 0.0f;
            robot->joint_motor_fdb[i].online = false;
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered)
        {
            BalanceFillFdbFromDm(&robot->wheel_motor_fdb[i], g_wheel_binding[i].motor);
        }
        else
        {
            robot->wheel_motor_fdb[i].pos = 0.0f;
            robot->wheel_motor_fdb[i].vel = 0.0f;
            robot->wheel_motor_fdb[i].tor = 0.0f;
            robot->wheel_motor_fdb[i].online = false;
        }
    }
}

// 遍历所有已注册的电机，调用 BalanceApplyCmdToDm 将机器人控制指令（cmd 数组）下发给电机
void BalanceMotorIf_SendCommand(const BalanceRobot* robot)
{
    if (robot == nullptr)
    {
        return;
    }

    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (!g_joint_binding[i].registered)
        {
            continue;
        }

        BalanceApplyCmdToDm(g_joint_binding[i].motor,
                            &robot->joint_motor_cmd[i],
                            BALANCE_DEFAULT_JOINT_TORQUE_LIMIT,
                            BALANCE_DEFAULT_JOINT_KP_LIMIT,
                            BALANCE_DEFAULT_JOINT_KD_LIMIT);
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (!g_wheel_binding[i].registered)
        {
            continue;
        }

        BalanceApplyCmdToDm(g_wheel_binding[i].motor,
                            &robot->wheel_motor_cmd[i],
                            BALANCE_DEFAULT_WHEEL_TORQUE_LIMIT,
                            BALANCE_DEFAULT_JOINT_KP_LIMIT,
                            BALANCE_DEFAULT_JOINT_KD_LIMIT);
    }
}

// 立即将所有已注册电机的控制量和PID系数置零
void BalanceMotorIf_DisableAll()
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (!g_joint_binding[i].registered || g_joint_binding[i].motor == nullptr)
        {
            continue;
        }

        g_joint_binding[i].motor->Set_Control_Angle(0.0f);
        g_joint_binding[i].motor->Set_Control_Omega(0.0f);
        g_joint_binding[i].motor->Set_Control_Torque(0.0f);
        g_joint_binding[i].motor->Set_K_P(0.0f);
        g_joint_binding[i].motor->Set_K_D(0.0f);
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (!g_wheel_binding[i].registered || g_wheel_binding[i].motor == nullptr)
        {
            continue;
        }

        g_wheel_binding[i].motor->Set_Control_Angle(0.0f);
        g_wheel_binding[i].motor->Set_Control_Omega(0.0f);
        g_wheel_binding[i].motor->Set_Control_Torque(0.0f);
        g_wheel_binding[i].motor->Set_K_P(0.0f);
        g_wheel_binding[i].motor->Set_K_D(0.0f);
    }
}

// 分别调用每个已注册电机的 CAN_Send_Exit()
void BalanceMotorIf_SendExitAll()
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->CAN_Send_Exit();
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->CAN_Send_Exit();
        }
    }
}

// 分别调用每个已注册电机的 CAN_Send_Enter()
void BalanceMotorIf_SendEnterAll()
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->CAN_Send_Enter();
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->CAN_Send_Enter();
        }
    }
}

void BalanceMotorIf_TxAllPeriodic()
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->TIM_Send_PeriodElapsedCallback();
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->TIM_Send_PeriodElapsedCallback();
        }
    }
}

void BalanceMotorIf_AliveAllPeriodic()
{
    for (uint8_t i = 0; i < BALANCE_JOINT_NUM; ++i)
    {
        if (g_joint_binding[i].registered && g_joint_binding[i].motor != nullptr)
        {
            g_joint_binding[i].motor->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }

    for (uint8_t i = 0; i < BALANCE_WHEEL_NUM; ++i)
    {
        if (g_wheel_binding[i].registered && g_wheel_binding[i].motor != nullptr)
        {
            g_wheel_binding[i].motor->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}