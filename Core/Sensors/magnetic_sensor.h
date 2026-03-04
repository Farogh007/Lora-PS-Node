/*
 * magnetic_sensor.h
 *
 * Polling pipeline for MMC5983 magnetic sensor:
 * - Periodic sampling at fixed rate
 * - Nonblocking state machine (no HAL_Delay)
 * - Ring buffer of samples
 * - Startup baseline calibration (fixed after N samples)
 * - Car detection flag 0/1 using delta magnitude threshold + hysteresis + debounce
 */

#pragma once
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#include "mmc5983.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAGNETIC_SENSOR_RING_SIZE
#define MAGNETIC_SENSOR_RING_SIZE 64u
#endif

typedef struct {
    uint32_t timestamp_ms;

    // centered counts from SET/RESET
    int32_t x_counts;
    int32_t y_counts;
    int32_t z_counts;

    // uT scaled by 100 (centi-uT)
    int32_t x_uT_x100;
    int32_t y_uT_x100;
    int32_t z_uT_x100;

    // magnitude (centi-uT)
    uint32_t mag_uT_x100;

    // baseline + delta (centi-uT)
    uint32_t baseline_mag_uT_x100;
    int32_t  delta_mag_uT_x100;

    // detection flag
    uint8_t car_present;
} MagneticSensorSample_t;

typedef struct {
    // --- config ---
    uint32_t sample_period_ms;     // e.g. 20ms for 50Hz

    // baseline calibration:
    uint16_t baseline_samples;     // e.g. 100 samples (~2s at 50Hz)

    // detection thresholds (centi-uT)
    int32_t thresh_on_uT_x100;     // e.g. 300  => 3.00 uT
    int32_t thresh_off_uT_x100;    // e.g. 200  => 2.00 uT

    // debounce counts (consecutive samples)
    uint8_t debounce_on;
    uint8_t debounce_off;
    uint32_t hold_on_ms;        // e.g. 2000ms

} MagneticSensorConfig_t;

// Main sensor object
typedef struct {
    MMC5983_Handle mmc;

    MagneticSensorConfig_t cfg;

    // ring buffer
    MagneticSensorSample_t ring[MAGNETIC_SENSOR_RING_SIZE];
    uint16_t wr;
    uint16_t rd;
    uint16_t count;

    // scheduling
    uint32_t next_sample_ms;

    // error counters
    uint32_t i2c_errors;
    uint32_t timeouts;
    uint32_t samples_ok;

    // baseline state
    uint8_t baseline_ready;
    uint16_t baseline_acc_n;
    uint64_t baseline_acc_sum; // sum of mag_uT_x100

    // detection state
    uint8_t car_present;
    uint8_t on_streak;
    uint8_t off_streak;

    // internal state machine
    uint8_t st;
    uint32_t st_deadline_ms;

    MMC5983_Raw18 raw_reset;
    MMC5983_Raw18 raw_set;

    // last produced sample (for GetLatest fast path)
    MagneticSensorSample_t last;
    uint8_t last_valid;

    uint32_t car_on_until_ms;   // latch timer

    uint32_t baseline_mag_uT_x100;  // frozen baseline once ready


} MagneticSensor_t;

// --- API ---
void MagneticSensor_DefaultConfig(MagneticSensorConfig_t *cfg);

HAL_StatusTypeDef MagneticSensor_Init(MagneticSensor_t *s,
                                     I2C_HandleTypeDef *hi2c,
                                     uint16_t mmc_addr8,
                                     uint32_t i2c_timeout_ms,
                                     const MagneticSensorConfig_t *cfg);

// Call frequently from while(1) with current time in ms
void MagneticSensor_Tick(MagneticSensor_t *s, uint32_t now_ms);

// Buffer access
uint16_t MagneticSensor_Available(const MagneticSensor_t *s);
bool MagneticSensor_Pop(MagneticSensor_t *s, MagneticSensorSample_t *out);
bool MagneticSensor_GetLatest(const MagneticSensor_t *s, MagneticSensorSample_t *out);

// Quick getters
uint8_t MagneticSensor_GetCarPresent(const MagneticSensor_t *s);
uint8_t MagneticSensor_IsBaselineReady(const MagneticSensor_t *s);

// Error counters
uint32_t MagneticSensor_GetI2CErrors(const MagneticSensor_t *s);
uint32_t MagneticSensor_GetTimeouts(const MagneticSensor_t *s);

#ifdef __cplusplus
}
#endif
