#pragma once

#include <stdarg.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct BalanceRobot;

struct BalanceRefPoseState
{
    bool active;
    bool finished;
    uint16_t stable_count;

    float target_cont[2];
};

void BalanceRefPose_Init(BalanceRefPoseState* state);
void BalanceRefPose_Start(BalanceRefPoseState* state);
void BalanceRefPose_Stop(BalanceRefPoseState* state);

void BalanceRefPose_Update(BalanceRefPoseState* state, BalanceRobot* robot);

bool BalanceRefPose_IsActive(const BalanceRefPoseState* state);
bool BalanceRefPose_IsFinished(const BalanceRefPoseState* state);

#ifdef __cplusplus
}
#endif
