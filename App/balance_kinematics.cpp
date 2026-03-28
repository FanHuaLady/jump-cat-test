#include "balance_kinematics.h"

#include <math.h>
#include <stddef.h>

#include "balance_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace
{
    struct Point2D
    {
        float x;
        float y;
    };

    static inline float SafeSqrt(float x)
    {
        return (x <= 0.0f) ? 0.0f : sqrtf(x);
    }

    static inline float Norm(const Point2D& a)
    {
        return SafeSqrt(a.x * a.x + a.y * a.y);
    }

    static inline Point2D Add(const Point2D& a, const Point2D& b)
    {
        return {a.x + b.x, a.y + b.y};
    }

    static inline Point2D Sub(const Point2D& a, const Point2D& b)
    {
        return {a.x - b.x, a.y - b.y};
    }

    static inline Point2D Scale(const Point2D& a, float s)
    {
        return {a.x * s, a.y * s};
    }

    static inline float Cross(const Point2D& a, const Point2D& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    static inline Point2D Normalize(const Point2D& a)
    {
        const float n = Norm(a);
        if (n < BALANCE_DEFAULT_MIN_OP_LEN)
        {
            // 给一个默认向下方向，避免奇异时崩掉
            return {0.0f, 1.0f};
        }
        return {a.x / n, a.y / n};
    }

    static Point2D GetTopCenterO()
    {
        // 上部公共等效点 O
        return {0.0f, 0.0f};
    }
    
    // 计算出C点的坐标
    static Point2D GetPointC(float phi1)
    {
        const float L1 = BALANCE_DEFAULT_LEG_L1;
        return {L1 * cosf(phi1), L1 * sinf(phi1)};
    }
    
    // 计算出B点的坐标
    static Point2D GetPointB(float phi4)
    {
        const float L4 = BALANCE_DEFAULT_LEG_L4;
        return {L4 * cosf(phi4), L4 * sinf(phi4)};
    }
    
    // 计算出P点的坐标
    static Point2D GetPointP(float phi1, float phi4)
    {
        // 上部闭链：
        // C -> P = L2
        // B -> P = L3
        //
        // P 是以 C 为圆心、L2 为半径的圆
        // 与以 B 为圆心、L3 为半径的圆的交点

        const float L2 = BALANCE_DEFAULT_LEG_L2;
        const float L3 = BALANCE_DEFAULT_LEG_L3;

        const Point2D C = GetPointC(phi1);
        const Point2D B = GetPointB(phi4);

        const Point2D CB = Sub(B, C);
        const float d = Norm(CB);

        if (d < 1e-6f)
        {
            return {0.0f, BALANCE_DEFAULT_LEG_LEN_STAND};
        }

        float a = (L2 * L2 - L3 * L3 + d * d) / (2.0f * d);
        float h2 = L2 * L2 - a * a;
        if (h2 < 0.0f)
        {
            h2 = 0.0f;
        }
        const float h = sqrtf(h2);

        const Point2D ex = {CB.x / d, CB.y / d};
        const Point2D ey = {-ex.y, ex.x};

        const Point2D mid = Add(C, Scale(ex, a));

        const Point2D P1 = Add(mid, Scale(ey, h));
        const Point2D P2 = Add(mid, Scale(ey, -h));

        // 默认取“更下方”的交点作为 P
        // 后面若发现支链反了，改这里
        return (P1.y < P2.y) ? P1 : P2;
    }
    
    // 获取点N
    static Point2D GetPointN(float phi1)
    {
        // 由：
        // O、C、N 共线
        // ON = 常数
        //
        // 直接构造 N

        const Point2D O = GetTopCenterO();              // 原点
        const Point2D C = GetPointC(phi1);              // C点

        const Point2D dir_oc = Normalize(Sub(C, O));    // 获得方向
        
        // N 在 OC 这条线上，并且距 O 的距离固定
        return Add(O, Scale(dir_oc, BALANCE_DEFAULT_ON_SIGN * BALANCE_DEFAULT_ON_LENGTH));
    }
    
    // 获得W点
    static Point2D GetWheelCenterW(float phi1, float phi4)
    {
        // 由：
        // WN ∥ CP
        // |WN| = 常数
        //
        // 直接构造 W
        //
        // 然后用 O、P、W 共线关系做一致性检查（不强制修正）

        const Point2D C = GetPointC(phi1);
        const Point2D P = GetPointP(phi1, phi4);
        const Point2D N = GetPointN(phi1);

        const Point2D dir_cp = Normalize(Sub(P, C));

        const Point2D W = Add(
            N,
            Scale(dir_cp, BALANCE_DEFAULT_WN_SIGN * BALANCE_DEFAULT_WN_LENGTH));

        return W;
    }
    
    // 获得虚拟腿
    static void GetVirtualLeg(float phi1, float phi4, float* l0, float* phi0)
    {
        if (l0 == nullptr || phi0 == nullptr)
        {
            return;
        }

        const Point2D O = GetTopCenterO();
        const Point2D W = GetWheelCenterW(phi1, phi4);

        const Point2D OW = Sub(W, O);

        *l0 = Norm(OW);                             // OW长度就是虚拟腿长
        *phi0 = atan2f(OW.y, OW.x);                 // 虚拟腿角度
    }
}

void BalanceCalcL0Phi0(float phi1, float phi4, float out[2])
{
    if (out == nullptr)
    {
        return;
    }

    float l0 = 0.0f;
    float phi0 = 0.0f;

    GetVirtualLeg(phi1, phi4, &l0, &phi0);

    out[0] = l0;
    out[1] = phi0;
}

void BalanceCalcJacobian(float phi1, float phi4, float J[2][2])
{
    if (J == nullptr)
    {
        return;
    }

    // 数值微分 Jacobian
    const float h = 1e-5f;

    float base[2] = {0.0f, 0.0f};
    float p1[2] = {0.0f, 0.0f};
    float p2[2] = {0.0f, 0.0f};

    BalanceCalcL0Phi0(phi1, phi4, base);
    BalanceCalcL0Phi0(phi1 + h, phi4, p1);
    BalanceCalcL0Phi0(phi1, phi4 + h, p2);

    // J:
    // [ dL0/dphi1    dL0/dphi4   ]
    // [ dPhi0/dphi1  dPhi0/dphi4 ]
    J[0][0] = (p1[0] - base[0]) / h;
    J[1][0] = (p1[1] - base[1]) / h;

    J[0][1] = (p2[0] - base[0]) / h;
    J[1][1] = (p2[1] - base[1]) / h;
}

// 获得虚拟腿长度变化率 dl0 和虚拟腿摆角角速度 dphi0
void BalanceCalcdL0dPhi0(float J[2][2], float dphi1, float dphi4, float out[2])
{
    if (J == nullptr || out == nullptr)
    {
        return;
    }

    out[0] = J[0][0] * dphi1 + J[0][1] * dphi4;
    out[1] = J[1][0] * dphi1 + J[1][1] * dphi4;
}

void BalanceCalcVmc(float F0, float Tp, float J[2][2], float out[2])
{
    if (J == nullptr || out == nullptr)
    {
        return;
    }

    // tau = J^T * f
    out[0] = J[0][0] * F0 + J[1][0] * Tp;
    out[1] = J[0][1] * F0 + J[1][1] * Tp;
}

void BalanceCalcLegForce(float J[2][2], float T1, float T2, float out[2])
{
    if (J == nullptr || out == nullptr)
    {
        return;
    }

    // tau = J^T * f
    // [T1]   [J00 J10] [F0]
    // [T2] = [J01 J11] [Tp]

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

    out[0] = ( d * T1 - b * T2) / det;
    out[1] = (-c * T1 + a * T2) / det;
}