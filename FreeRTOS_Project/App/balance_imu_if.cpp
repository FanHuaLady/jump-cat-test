#include "balance_imu_if.h"

#include <math.h>
#include <stddef.h>

#include "bsp_jy61p.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)

namespace
{
    static bool g_balance_imu_if_inited = false;

    static inline float DegToRad(float deg)
    {
        return deg * DEG_TO_RAD;
    }
}

void BalanceImuIf_Init(void)
{
    g_balance_imu_if_inited = true;
}

void BalanceImuIf_Update(BalanceImuData* imu)
{
    if (imu == nullptr)
    {
        return;
    }

    if (!g_balance_imu_if_inited)
    {
        BalanceImuIf_Init();
    }

    // JY61P原始接口：
    // 角度     -> 度
    // 角速度   -> 度/秒
    // 加速度   -> 在bsp_jy61p.cpp里已换算成 m/s^2

    imu->roll  = DegToRad(BSP_JY61P.GetRoll());
    imu->pitch = DegToRad(BSP_JY61P.GetPitch());
    imu->yaw   = DegToRad(BSP_JY61P.GetYaw());

    imu->roll_dot  = DegToRad(BSP_JY61P.GetGyroX());
    imu->pitch_dot = DegToRad(BSP_JY61P.GetGyroY());
    imu->yaw_dot   = DegToRad(BSP_JY61P.GetGyroZ());

    imu->ax = BSP_JY61P.GetAccelX();
    imu->ay = BSP_JY61P.GetAccelY();
    imu->az = BSP_JY61P.GetAccelZ();

    // 先简单认为在线
    // 后面如果你想更严谨，可以加 UART 超时判断
    imu->online = true;
}
