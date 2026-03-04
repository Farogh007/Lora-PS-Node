/*
 * RAK.c
 *
 *  Created on: 02.03.2026
 *      Author: FI
 */

#include "RAK.h"
#include <string.h>
#include <stdio.h>

static uint32_t now_ms(void) { return HAL_GetTick(); }

static void uart_start_rx_it(RAK_Handle *h)
{
    // Start 1-byte interrupt receive
    (void)HAL_UART_Receive_IT(h->cfg.huart, &h->rx_byte, 1);
}

static void uart_send_str(RAK_Handle *h, const char *s)
{
    // Short commands only; blocking TX is OK here (few bytes).
    (void)HAL_UART_Transmit(h->cfg.huart, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static void flags_reset_wait(RAK_Handle *h)
{
    h->got_ok = false;
    h->got_error = false;
    h->got_joined_evt = false;
    h->got_join_fail_evt = false;
    h->got_tx_done_evt = false;
}

static void send_cmd_wait_ok(RAK_Handle *h, const char *cmd, uint32_t timeout_ms)
{
    // Prepare wait for OK/ERROR
    flags_reset_wait(h);
    h->wait_ok = true;
    h->cmd_deadline_ms = now_ms() + timeout_ms;

    uart_send_str(h, cmd);
    uart_send_str(h, "\r\n");
}

// Returns true when OK or ERROR or timeout is reached.
static bool wait_ok_done(RAK_Handle *h, RAK_Status *out_status)
{
    if (!h->wait_ok) {
        if (out_status) *out_status = RAK_OK;
        return true;
    }

    if (h->got_ok) {
        h->wait_ok = false;
        if (out_status) *out_status = RAK_OK;
        return true;
    }

    if (h->got_error) {
        h->wait_ok = false;
        if (out_status) *out_status = RAK_ERR_AT_FAIL;
        return true;
    }

    // Timeout
    if ((int32_t)(now_ms() - h->cmd_deadline_ms) >= 0) {
        h->wait_ok = false;
        if (out_status) *out_status = RAK_ERR_TIMEOUT;
        return true;
    }

    return false;
}

static void parse_line(RAK_Handle *h, const char *line)
{
    // Typical RUI3 answers:
    // "OK"
    // "ERROR"
    // "+EVT:JOINED"
    // "+EVT:JOIN FAILED"
    // "+EVT:TX_DONE"
    // "AT_COMMAND_NOT_FOUND"

    if (strcmp(line, "OK") == 0) {
        h->got_ok = true;
        return;
    }

    if (strcmp(line, "ERROR") == 0) {
        h->got_error = true;
        return;
    }

    if (strstr(line, "AT_COMMAND_NOT_FOUND")) {
        h->got_error = true;
        return;
    }

    if (strstr(line, "+EVT:JOINED")) {
        h->joined = true;
        h->join_in_progress = false;
        h->got_joined_evt = true;
        return;
    }

    if (strstr(line, "+EVT:JOIN FAILED") || strstr(line, "+EVT:JOIN_FAIL")) {
        h->joined = false;
        h->join_in_progress = false;
        h->got_join_fail_evt = true;
        return;
    }

    if (strstr(line, "+EVT:TX_DONE")) {
        h->tx_in_progress = false;
        h->got_tx_done_evt = true;
        return;
    }

    // You can extend this to capture downlinks, RX events, etc.
}

void RAK_Init(RAK_Handle *h, const RAK_Cfg *cfg)
{
    if (!h || !cfg || !cfg->huart) return;

    memset(h, 0, sizeof(*h));
    h->cfg = *cfg;

    h->joined = false;
    h->join_in_progress = false;
    h->tx_in_progress = false;

    h->init_step = 0;
    h->join_attempts = 0;
    h->next_action_ms = now_ms();

    // Start UART RX IRQ
    uart_start_rx_it(h);
}

void RAK_OnUartRxCplt(RAK_Handle *h)
{
    if (!h) return;

    char c = (char)h->rx_byte;

    // Assemble line by CR/LF
    if (c == '\r') {
        // ignore
    } else if (c == '\n') {
        if (h->line_len > 0) {
            h->line_buf[h->line_len] = '\0';
            h->line_ready = true;
        }
        h->line_len = 0;
    } else {
        if (h->line_len < (RAK_LINE_MAX - 1)) {
            h->line_buf[h->line_len++] = c;
        } else {
            // overflow -> reset line
            h->line_len = 0;
        }
    }

    // Re-arm RX
    uart_start_rx_it(h);
}

static void service_rx_lines(RAK_Handle *h)
{
    if (!h->line_ready) return;

    h->line_ready = false;
    parse_line(h, h->line_buf);
}

bool RAK_IsJoined(const RAK_Handle *h)
{
    return h ? h->joined : false;
}

void RAK_RequestRejoin(RAK_Handle *h)
{
    if (!h) return;
    h->joined = false;
    h->join_in_progress = false;
    h->join_attempts = 0;

    // Kick state machine soon
    h->init_step = 100; // jump to join logic
    h->next_action_ms = now_ms();
}

static bool time_reached(uint32_t t)
{
    return (int32_t)(now_ms() - t) >= 0;
}

void RAK_Task(RAK_Handle *h)
{
    if (!h) return;

    // 1) Parse UART lines (events, OK, etc.)
    service_rx_lines(h);

    // 2) If waiting for OK/ERROR, do not spam commands
    //    but we still allow async events to update states.
    RAK_Status st;
    if (!wait_ok_done(h, &st)) {
        return;
    }

    // 3) Rate-limit init steps
    if (!time_reached(h->next_action_ms)) return;

    // -------- Init / config state machine --------
    // Steps are non-blocking; each step issues one AT command and waits for OK.
    // If timeout happens, it retries that step after a delay.

    // 0..99 : initial config
    if (h->init_step < 100) {
        switch (h->init_step) {
        case 0:
            // Basic AT test (wakes up module)
            send_cmd_wait_ok(h, "AT", 800);
            h->init_step++;
            h->next_action_ms = now_ms() + 100;
            break;

        case 1:
            // Set LoRaWAN mode
            send_cmd_wait_ok(h, "AT+NWM=1", 800);
            h->init_step++;
            h->next_action_ms = now_ms() + 100;
            break;

        case 2: {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "AT+BAND=%d", RAK_BAND_EU868);
            send_cmd_wait_ok(h, cmd, 800);
            h->init_step++;
            h->next_action_ms = now_ms() + 100;
            break;
        }

        case 3:
            // OTAA
            send_cmd_wait_ok(h, "AT+NJM=1", 800);
            h->init_step++;
            h->next_action_ms = now_ms() + 100;
            break;

        case 4: {
            char cmd[64];
            snprintf(cmd, sizeof(cmd), "AT+APPEUI=%s", RAK_JOIN_EUI);
            send_cmd_wait_ok(h, cmd, 1000);
            h->init_step++;
            h->next_action_ms = now_ms() + 100;
            break;
        }

        case 5: {
            char cmd[96];
            snprintf(cmd, sizeof(cmd), "AT+APPKEY=%s", RAK_APP_KEY);
            send_cmd_wait_ok(h, cmd, 1200);
            h->init_step++;
            h->next_action_ms = now_ms() + 100;
            break;
        }

        case 6:
            // Optional: ADR on (good if near gateway)
            send_cmd_wait_ok(h, "AT+ADR=1", 800);
            h->init_step = 100; // go to join phase
            h->next_action_ms = now_ms() + 200;
            break;

        default:
            h->init_step = 100;
            break;
        }

        // Handle errors by retrying same step after delay
        if (st != RAK_OK && st != RAK_OK /* redundant */) {
            // If previous wait_ok_done returned timeout/fail, it already cleared wait_ok.
            // Retry current step after some backoff.
            h->next_action_ms = now_ms() + 500;
        }

        return;
    }

    // -------- Join phase (step >= 100) --------
    if (!h->joined && !h->join_in_progress) {

        // backoff between join attempts
        if (h->join_attempts >= 10) {
            // Too many attempts: pause longer
            h->next_action_ms = now_ms() + 15000;
            h->join_attempts = 0;
            return;
        }

        // Start join
        send_cmd_wait_ok(h, "AT+JOIN=1", 2000);
        h->join_in_progress = true;
        h->join_attempts++;
        // We expect +EVT:JOINED asynchronously; give it time
        h->next_action_ms = now_ms() + 6000;
        return;
    }

    // If join is in progress, check if it completed (via events)
    if (h->join_in_progress) {
        // If we got a join fail event, allow retry
        if (h->got_join_fail_evt) {
            h->join_in_progress = false;
            h->joined = false;
            h->next_action_ms = now_ms() + 3000;
            return;
        }

        // If join succeeded, we're done
        if (h->joined) {
            h->join_in_progress = false;
            h->next_action_ms = now_ms() + 1000;
            return;
        }

        // No event yet -> keep waiting (but don’t lock up)
        h->next_action_ms = now_ms() + 2000;
        return;
    }

    // Joined and idle
    h->next_action_ms = now_ms() + 1000;
}

RAK_Status RAK_SendCarDetected(RAK_Handle *h, uint8_t detected)
{
    if (!h) return RAK_ERR_PARAM;

    if (!h->joined) return RAK_ERR_NOT_JOINED;
    if (h->tx_in_progress || h->wait_ok) return RAK_ERR_BUSY;

    detected = detected ? 1 : 0;

    char cmd[48];
    // 1 byte payload: 00 or 01
    snprintf(cmd, sizeof(cmd), "AT+SEND=%d:%02X", RAK_FPORT_CAR_STATUS, detected);

    // Send command and wait OK quickly; TX_DONE comes async
    send_cmd_wait_ok(h, cmd, 1200);
    h->tx_in_progress = true;

    return RAK_OK;
}
