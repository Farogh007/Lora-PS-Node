# Log module (UART1 debug logging)

Debug logging over **USART1**. Used for `printf` and optional `Log_Printf` output during development.

**Sources:** `Core/Inc/log.h`, `Core/Src/log.c` (in main project build).

---

## Enable / disable

- **Compile time:** In `Core/Inc/log.h` set `#define LOG_ENABLE 0` to remove log output and `Log_Printf` calls.
- **Runtime:** Call `Log_Enable()` or `Log_Disable()`. When disabled, `printf` and `Log_Printf` produce no output.

---

## API

| Function | Description |
|----------|-------------|
| `Log_Init(UART_HandleTypeDef *huart)` | Set UART (e.g. `&huart1` for USART1). |
| `Log_Enable()` | Turn logging on. |
| `Log_Disable()` | Turn logging off. |
| `Log_IsEnabled()` | Returns true if logging is on and UART is set. |
| `Log_Write(const char *buf, int len)` | Raw write; used by `_write()` for `printf`. |
| `Log_Printf(const char *fmt, ...)` | Printf-style; no-op when `LOG_ENABLE` is 0. |

---

## Usage

- In `main.c`, after HAL init: `Log_Init(&huart1); Log_Enable();`
- All `printf(...)` and `Log_Printf(...)` then go to USART1 when logging is enabled.
- Connect a serial terminal (e.g. 115200 8N1) to the USART1 pins to view debug output.

For full project architecture and components, see the root **README.md**.
