#pragma once

#include <stdarg.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void BalanceTool_Print(const char* fmt, ...);
void BalanceTool_PrintRaw(const char* text);

void BalanceTool_PrintFloat4(const char* name, float value);

// 不依赖 %f，专门打印 4 位小数
void BalanceTool_PrintFloat4Line(const char* name0, float value0,
                                 const char* name1, float value1);

float BalanceTool_RadToDeg(float rad);

float BalanceTool_WrapRad(float rad);
float BalanceTool_AngleDiffRad(float target, float current);

#ifdef __cplusplus
}
#endif