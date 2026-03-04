/*
 * log.c
 *
 * Debug logging over UART. Option to enable/disable at compile (LOG_ENABLE)
 * and runtime (Log_Enable / Log_Disable).
 */

#include "log.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_BUF_SIZE 256

static UART_HandleTypeDef *s_huart;
static volatile bool s_enabled = true;

void Log_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_enabled = true;
}

void Log_Enable(void)
{
    s_enabled = true;
}

void Log_Disable(void)
{
    s_enabled = false;
}

bool Log_IsEnabled(void)
{
    return s_enabled && (s_huart != NULL);
}

void Log_Write(const char *buf, int len)
{
    if (!s_enabled || !s_huart || len <= 0) return;
    (void)HAL_UART_Transmit(s_huart, (const uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

#if LOG_ENABLE
void Log_Printf(const char *fmt, ...)
{
    if (!s_enabled || !s_huart) return;

    char buf[LOG_BUF_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
        Log_Write(buf, n);
    }
}
#endif
