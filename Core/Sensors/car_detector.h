/*
 * car_detector.h
 *
 *  Created on: 08.02.2026
 *      Author: FI
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "magnetic_sensor.h"  // MMC5983 magnetometer pipeline
#include "radar_presence.h"   // Acconeer A121 radar presence wrapper

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    // Magnet thresholds already exist in SensorConfig_t.
    // Here we configure fusion behavior:
    uint32_t radar_period_ms;           // e.g. 100ms
    int32_t  radar_score_on_x1000;      // Score threshold for ON (e.g. 400-800)
    int32_t  radar_score_off_x1000;     // Score threshold for OFF (hysteresis, e.g. 200-400)
    int32_t  radar_distance_min_mm;     // Minimum distance (e.g. 200mm = 20cm)
    int32_t  radar_distance_max_mm;     // Maximum distance (e.g. 1500mm = 1.5m for parking)
    uint8_t  radar_debounce_on;         // Consecutive detections needed to turn ON
    uint8_t  radar_debounce_off;        // Consecutive non-detections needed to turn OFF

    uint32_t on_confirm_ms;             // if only one sensor triggers, confirm after this
    uint32_t off_confirm_ms;            // require "no detection" for this long to clear

    uint32_t present_hold_ms;           // latch extension while any sensor sees presence
} CarDetectorConfig_t;

typedef struct
{
    MagneticSensor_t mag;
    RadarPresence_t  radar;
    CarDetectorConfig_t cfg;

    bool     car_present_fused;

    uint32_t suspect_since_ms;
    uint32_t off_since_ms;
    uint32_t present_until_ms;
} CarDetector_t;

void CarDetector_DefaultConfig(CarDetectorConfig_t *cfg);

bool CarDetector_Init(CarDetector_t *cd,
                      I2C_HandleTypeDef *hi2c,
                      uint16_t mmc_addr8,
                      uint32_t i2c_timeout_ms,
                      const MagneticSensorConfig_t *mag_cfg,
                      const CarDetectorConfig_t *cfg,
                      RadarPreset_t radar_preset);

void CarDetector_Tick(CarDetector_t *cd, uint32_t now_ms);

bool CarDetector_GetCarPresent(const CarDetector_t *cd);

// Optional debug helpers
int32_t CarDetector_GetRadarScore(const CarDetector_t *cd);
bool CarDetector_GetRadarPresence(const CarDetector_t *cd);

#ifdef __cplusplus
}
#endif

