#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct BalanceRobot;

typedef enum
{
    BALANCE_REF_POSE_MODE_LEFT_ONLY = 0,
    BALANCE_REF_POSE_MODE_RIGHT_ONLY = 1,
    BALANCE_REF_POSE_MODE_BOTH = 2,
} BalanceRefPoseMode;

struct BalanceRefPoseState
{
    bool active;
    bool finished;
    uint16_t stable_count;

    BalanceRefPoseMode mode;

    // [0] = L0
    // [1] = L1
    // [2] = R0
    // [3] = R1
    float target_cont[4];
};

void BalanceRefPose_Init(BalanceRefPoseState* state);
void BalanceRefPose_Start(BalanceRefPoseState* state);
void BalanceRefPose_StartWithMode(BalanceRefPoseState* state, BalanceRefPoseMode mode);
void BalanceRefPose_Stop(BalanceRefPoseState* state);

void BalanceRefPose_Update(BalanceRefPoseState* state, BalanceRobot* robot);

bool BalanceRefPose_IsActive(const BalanceRefPoseState* state);
bool BalanceRefPose_IsFinished(const BalanceRefPoseState* state);

#ifdef __cplusplus
}
#endif