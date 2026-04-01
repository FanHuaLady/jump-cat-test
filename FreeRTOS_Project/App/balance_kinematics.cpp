#include "balance_kinematics.h"

#include <math.h>
#include <stddef.h>

#include "balance_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace
{
    static inline float SafeSqrt(float x)
    {
        return (x <= 0.0f) ? 0.0f : sqrtf(x);
    }

    static inline float SafeAtan2(float y, float x)
    {
        return atan2f(y, x);
    }
}

void BalanceCalcL0Phi0(float phi1, float phi4, float out[2])
{
    if (out == nullptr)
    {
        return;
    }

    const float L1 = BALANCE_DEFAULT_LEG_L1;
    const float L2 = BALANCE_DEFAULT_LEG_L2;
    const float L3 = BALANCE_DEFAULT_LEG_L3;
    const float L4 = BALANCE_DEFAULT_LEG_L4;
    const float L5 = BALANCE_DEFAULT_LEG_L5;

    // 下面这套推导对应你前面给过的原平衡步兵五连杆/退化四连杆抽象
    const float xb = L1 * cosf(phi1) - L4 * cosf(phi4) + L5;
    const float yb = L1 * sinf(phi1) - L4 * sinf(phi4);

    const float b_len = SafeSqrt(xb * xb + yb * yb);

    // 避免奇异和除零
    if (b_len < 1e-6f)
    {
        out[0] = BALANCE_DEFAULT_LEG_LEN_STAND;
        out[1] = 0.0f;
        return;
    }

    // 中间杆夹角
    float cos_alpha = (L2 * L2 + b_len * b_len - L3 * L3) / (2.0f * L2 * b_len);
    if (cos_alpha > 1.0f) cos_alpha = 1.0f;
    if (cos_alpha < -1.0f) cos_alpha = -1.0f;

    const float alpha = acosf(cos_alpha);
    const float beta = SafeAtan2(yb, xb);

    // 原工程常用定义：phi0 为虚拟腿摆角
    const float phi0 = beta - alpha;

    // 足端位置（等效）
    const float xc = L1 * cosf(phi1) + L2 * cosf(phi0);
    const float yc = L1 * sinf(phi1) + L2 * sinf(phi0);

    const float l0 = SafeSqrt(xc * xc + yc * yc);

    out[0] = l0;
    out[1] = phi0;
}

void BalanceCalcJacobian(float phi1, float phi4, float J[2][2])
{
    if (J == nullptr)
    {
        return;
    }

    // 用数值微分构造雅可比，优点是稳、实现快，适合你当前先跑通
    // J:
    // [ dL0/dphi1    dL0/dphi4   ]
    // [ dPhi0/dphi1  dPhi0/dphi4 ]
    //
    // 后面如果你愿意，再换成原工程的解析式版本

    const float h = 1e-5f;

    float base[2] = {0.0f, 0.0f};
    float p1[2] = {0.0f, 0.0f};
    float p2[2] = {0.0f, 0.0f};

    BalanceCalcL0Phi0(phi1, phi4, base);

    BalanceCalcL0Phi0(phi1 + h, phi4, p1);
    BalanceCalcL0Phi0(phi1, phi4 + h, p2);

    J[0][0] = (p1[0] - base[0]) / h;
    J[1][0] = (p1[1] - base[1]) / h;

    J[0][1] = (p2[0] - base[0]) / h;
    J[1][1] = (p2[1] - base[1]) / h;
}

void BalanceCalcdL0dPhi0(float J[2][2], float dphi1, float dphi4, float out[2])
{
    if (J == nullptr || out == nullptr)
    {
        return;
    }

    // [dL0   ]   [dL0/dphi1   dL0/dphi4 ] [dphi1]
    // [dPhi0 ] = [dP0/dphi1   dP0/dphi4 ] [dphi4]

    out[0] = J[0][0] * dphi1 + J[0][1] * dphi4;
    out[1] = J[1][0] * dphi1 + J[1][1] * dphi4;
}

void BalanceCalcVmc(float F0, float Tp, float J[2][2], float out[2])
{
    if (J == nullptr || out == nullptr)
    {
        return;
    }

    // 关节力矩 = J^T * [F0, Tp]^T
    out[0] = J[0][0] * F0 + J[1][0] * Tp;
    out[1] = J[0][1] * F0 + J[1][1] * Tp;
}

void BalanceCalcLegForce(float J[2][2], float T1, float T2, float out[2])
{
    if (J == nullptr || out == nullptr)
    {
        return;
    }

    // 解线性方程：
    // [T1]   [J00 J10] [F0]
    // [T2] = [J01 J11] [Tp]
    //
    // 即：
    // tau = J^T * f
    //
    // => f = (J^T)^(-1) * tau

    const float a = J[0][0];
    const float b = J[1][0];
    const float c = J[0][1];
    const float d = J[1][1];

    const float det = a * d - b * c;

    if (fabsf(det) < 1e-8f)
    {
        out[0] = 0.0f;
        out[1] = 0.0f;
        return;
    }

    // inv([a b; c d]) = 1/det [ d -b; -c a ]
    out[0] = ( d * T1 - b * T2) / det;
    out[1] = (-c * T1 + a * T2) / det;
}
