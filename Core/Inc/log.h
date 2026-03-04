/*
 * log.h
 *
 * Debug logging over UART1. Enable/disable at compile time and runtime.
 */

#pragma once

#include "stm32l4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Set to 0 to remove all log output at compile time */
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

/** Initialize log with UART handle (e.g. &huart1 for USART1). */
void Log_Init(UART_HandleTypeDef *huart);

/** Enable logging (runtime). */
void Log_Enable(void);

/** Disable logging (runtime). */
void Log_Disable(void);

/** Return true if logging is enabled. */
bool Log_IsEnabled(void);

/** Write raw bytes to log UART. No-op if disabled. Used by _write for printf. */
void Log_Write(const char *buf, int len);

#if LOG_ENABLE
/** Printf-style log; only outputs when enabled. */
void Log_Printf(const char *fmt, ...);
#else
#define Log_Printf(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif
