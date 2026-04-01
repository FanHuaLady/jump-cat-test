#pragma once

// 正运动学：由两个关节角求虚拟腿长 l0 与虚拟腿角 phi0
void BalanceCalcL0Phi0(float phi1, float phi4, float out[2]);

// 雅可比矩阵：由两个关节角求 J
void BalanceCalcJacobian(float phi1, float phi4, float J[2][2]);

// 关节角速度 -> 虚拟腿长速度 / 虚拟腿角速度
void BalanceCalcdL0dPhi0(float J[2][2], float dphi1, float dphi4, float out[2]);

// 虚拟力 -> 关节力矩
void BalanceCalcVmc(float F0, float Tp, float J[2][2], float out[2]);

// 关节力矩 -> 虚拟力
void BalanceCalcLegForce(float J[2][2], float T1, float T2, float out[2]);
