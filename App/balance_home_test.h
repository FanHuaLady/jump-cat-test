#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void BalanceHomeTest_Task_Create(void);

void BalanceHomeTest_Enable(void);
void BalanceHomeTest_Disable(void);

bool BalanceHomeTest_IsHomeDone(void);

#ifdef __cplusplus
}
#endif
