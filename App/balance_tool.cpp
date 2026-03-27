#include "balance_tool.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "usart.h"

namespace
{
    constexpr uint16_t kToolTxTimeoutMs = 20U;
    constexpr float kRadToDeg = 57.2957795f;
    
    int AppendFloat4(char* buf, size_t buf_size, const char* name, float value)
    {
        if ((buf == nullptr) || (buf_size == 0U) || (name == nullptr))
        {
            return 0;
        }

        const char sign = (value < 0.0f) ? '-' : '\0';
        const float abs_value = (value < 0.0f) ? -value : value;
        const long scaled = static_cast<long>(abs_value * 10000.0f + 0.5f);
        const long integer_part = scaled / 10000L;
        const long fraction_part = scaled % 10000L;

        if (sign != '\0')
        {
            return snprintf(buf, buf_size, "%s=%c%ld.%04ld",
                            name, sign, integer_part, fraction_part);
        }

        return snprintf(buf, buf_size, "%s=%ld.%04ld",
                        name, integer_part, fraction_part);
    }
    
}

float BalanceTool_RadToDeg(float rad)
{
    return rad * kRadToDeg;
}

void BalanceTool_PrintRaw(const char* text)
{
    if (text == nullptr)
    {
        return;
    }

    const size_t len = strlen(text);
    if (len == 0U)
    {
        return;
    }

    HAL_UART_Transmit(&huart7,
                      reinterpret_cast<uint8_t*>(const_cast<char*>(text)),
                      static_cast<uint16_t>(len),
                      kToolTxTimeoutMs);
}

void BalanceTool_Print(const char* fmt, ...)
{
    if (fmt == nullptr)
    {
        return;
    }

    char buf[128] = {0};

    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (written <= 0)
    {
        return;
    }

    BalanceTool_PrintRaw(buf);
}

void BalanceTool_PrintFloat4(const char* name, float value)
{
    char buf[128] = {0};

    const int written = AppendFloat4(buf, sizeof(buf), name, value);
    if (written <= 0)
    {
        return;
    }

    const size_t used = static_cast<size_t>(written);
    if (used < sizeof(buf))
    {
        snprintf(buf + used, sizeof(buf) - used, "\r\n");
    }

    BalanceTool_PrintRaw(buf);
}

void BalanceTool_PrintFloat4Line(const char* name0, float value0,
                                 const char* name1, float value1)
{
    char buf[128] = {0};
    size_t used = 0U;

    int written = AppendFloat4(buf + used, sizeof(buf) - used, name0, value0);
    if (written > 0) used += static_cast<size_t>(written);

    if (used < sizeof(buf))
    {
        written = snprintf(buf + used, sizeof(buf) - used, ", ");
        if (written > 0) used += static_cast<size_t>(written);
    }

    if (used < sizeof(buf))
    {
        written = AppendFloat4(buf + used, sizeof(buf) - used, name1, value1);
        if (written > 0) used += static_cast<size_t>(written);
    }

    if (used < sizeof(buf))
    {
        snprintf(buf + used, sizeof(buf) - used, "\r\n");
    }

    BalanceTool_PrintRaw(buf);
}
