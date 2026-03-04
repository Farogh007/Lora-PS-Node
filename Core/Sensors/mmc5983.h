/*
 * mmc5983.h
 *
 * Polling-only MMC5983MA driver for STM32 HAL.
 * - No INT dependency
 * - Provides nonblocking-friendly building blocks used by magnetic_sensor.c state machine
 */

#pragma once
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 7-bit I2C address per datasheet
#define MMC5983_I2C_ADDR_7BIT      0x30
#define MMC5983_I2C_ADDR_8BIT      (MMC5983_I2C_ADDR_7BIT << 1)

// Registers
#define MMC5983_REG_XOUT0        0x00
#define MMC5983_REG_XYZOUT2       0x06
#define MMC5983_REG_STATUS       0x08
#define MMC5983_REG_CTRL0        0x09
#define MMC5983_REG_PRODUCT_ID    0x2F

// STATUS bits
#define MMC5983_STATUS_MEAS_M_DONE   (1u << 0)

// CTRL0 bits (write-only)
#define MMC5983_CTRL0_TM_M       (1u << 0)  // trigger magnetic measurement
#define MMC5983_CTRL0_TM_T       (1u << 1)  // trigger temperature measurement
#define MMC5983_CTRL0_INT_EN     (1u << 2)  // enable interrupt pin (we don't rely on it)
#define MMC5983_CTRL0_SET        (1u << 3)  // SET operation
#define MMC5983_CTRL0_RESET      (1u << 4)  // RESET operation

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr8;
    uint32_t timeout_ms;
    uint8_t product_id;
} MMC5983_Handle;

typedef struct {
    uint32_t x; // 18-bit
    uint32_t y;
    uint32_t z;
} MMC5983_Raw18;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} MMC5983_FieldCounts;

// --- Init / basic IO ---
HAL_StatusTypeDef MMC5983_Init(MMC5983_Handle *dev,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t addr8,
                               uint32_t timeout_ms);

HAL_StatusTypeDef MMC5983_ReadProductId(MMC5983_Handle *dev, uint8_t *id);

// --- Measurement control (polling workflow) ---
HAL_StatusTypeDef MMC5983_WriteCtrl0(MMC5983_Handle *dev, uint8_t ctrl0);
HAL_StatusTypeDef MMC5983_ReadStatus(MMC5983_Handle *dev, uint8_t *status);
HAL_StatusTypeDef MMC5983_ClearStatus(MMC5983_Handle *dev, uint8_t mask_w1c);

// Trigger a magnetic measurement (TM_M)
HAL_StatusTypeDef MMC5983_TriggerMag(MMC5983_Handle *dev);

// Perform SET or RESET pulse (no measurement triggered here)
HAL_StatusTypeDef MMC5983_PulseSet(MMC5983_Handle *dev);
HAL_StatusTypeDef MMC5983_PulseReset(MMC5983_Handle *dev);

// Read 18-bit raw XYZ (7 bytes)
HAL_StatusTypeDef MMC5983_ReadRaw18(MMC5983_Handle *dev, MMC5983_Raw18 *raw);

// Compute centered counts from SET/RESET pair: counts = (SET - RESET)/2
MMC5983_FieldCounts MMC5983_ComputeCounts(const MMC5983_Raw18 *raw_set,
                                          const MMC5983_Raw18 *raw_reset);

#ifdef __cplusplus
}
#endif
