/*
 * RAK.h
 *
 *  Created on: 02.03.2026
 *      Author: FI
 */

#pragma once

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------- User config (edit these) --------
#define RAK_JOIN_EUI   "0000000000000000"
#define RAK_APP_KEY    "D08C968D11E39AE4DDA95710C29E12C3"

// Germany / EU868 = BAND 4 in RUI3
#define RAK_BAND_EU868 4

// Use a fixed application port for your car status
#define RAK_FPORT_CAR_STATUS 10

// UART line sizes
#define RAK_LINE_MAX 128

typedef enum {
    RAK_OK = 0,
    RAK_ERR_PARAM,
    RAK_ERR_BUSY,
    RAK_ERR_TIMEOUT,
    RAK_ERR_NOT_JOINED,
    RAK_ERR_AT_FAIL
} RAK_Status;

typedef struct {
    UART_HandleTypeDef *huart;
} RAK_Cfg;

typedef struct {
    RAK_Cfg cfg;

    // RX byte buffer for IRQ receive
    uint8_t rx_byte;

    // Line assembly
    char line_buf[RAK_LINE_MAX];
    uint16_t line_len;
    volatile bool line_ready;

    // State
    bool joined;
    bool join_in_progress;
    bool tx_in_progress;

    // command/response waiting
    volatile bool wait_ok;
    volatile bool got_ok;
    volatile bool got_error;
    volatile bool got_joined_evt;
    volatile bool got_join_fail_evt;
    volatile bool got_tx_done_evt;

    uint32_t cmd_deadline_ms;

    // init/join sequencing
    uint8_t init_step;
    uint8_t join_attempts;
    uint32_t next_action_ms;

} RAK_Handle;

// Initialize driver and start UART RX interrupt
void RAK_Init(RAK_Handle *h, const RAK_Cfg *cfg);

// Must be called regularly in main loop to progress init/join and parse lines
void RAK_Task(RAK_Handle *h);

// Feed UART RX complete callback
void RAK_OnUartRxCplt(RAK_Handle *h);

// Query joined state
bool RAK_IsJoined(const RAK_Handle *h);

// Send car detected status (0/1). Non-blocking: it schedules the TX and returns quickly.
// Returns BUSY if previous TX still in progress.
RAK_Status RAK_SendCarDetected(RAK_Handle *h, uint8_t detected);

// Optional: force rejoin (non-blocking, handled in RAK_Task)
void RAK_RequestRejoin(RAK_Handle *h);

#ifdef __cplusplus
}
#endif
