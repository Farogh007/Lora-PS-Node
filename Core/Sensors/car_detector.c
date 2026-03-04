/*
 * car_detector.c
 *
 *  Created on: 08.02.2026
 *      Author: FI
 */


#include "car_detector.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <limits.h>

// Safe time difference calculation that handles wrap-around
// Returns the difference if now >= then, otherwise assumes wrap-around occurred
static inline uint32_t u32_now_minus(uint32_t now, uint32_t then)
{
    // Handle wrap-around: if now < then, assume wrap-around occurred
    // This is safe for time differences up to ~49 days (UINT32_MAX ms)
    if (now >= then) {
        return now - then;
    } else {
        // Wrap-around case: (UINT32_MAX - then) + now + 1
        return (UINT32_MAX - then) + now + 1;
    }
}

void CarDetector_DefaultConfig(CarDetectorConfig_t *cfg)
{
    if (!cfg) return;
    cfg->radar_period_ms           = 100;   // 10 Hz
    cfg->radar_score_on_x1000     = 600;   // Score threshold to turn ON (tune based on logs)
    cfg->radar_score_off_x1000    = 300;   // Score threshold to turn OFF (hysteresis)
    cfg->radar_distance_min_mm    = 200;   // 20cm minimum (ignore too close)
    cfg->radar_distance_max_mm    = 1500;  // 1.5m maximum (cars should be within this)
    cfg->radar_debounce_on        = 3;     // Need 3 consecutive detections
    cfg->radar_debounce_off       = 5;     // Need 5 consecutive non-detections
    cfg->on_confirm_ms            = 800;   // confirm if only one sensor triggers
    cfg->off_confirm_ms           = 1500;  // require quiet time to clear
    cfg->present_hold_ms         = 2000;  // "keep present" a bit
}

bool CarDetector_Init(CarDetector_t *cd,
                      I2C_HandleTypeDef *hi2c,
                      uint16_t mmc_addr8,
                      uint32_t i2c_timeout_ms,
                      const MagneticSensorConfig_t *mag_cfg,
                      const CarDetectorConfig_t *cfg,
                      RadarPreset_t radar_preset)
{
    if (!cd) return false;
    memset(cd, 0, sizeof(*cd));

    if (cfg) cd->cfg = *cfg;
    else CarDetector_DefaultConfig(&cd->cfg);

    // Init magnetometer pipeline
    if (MagneticSensor_Init(&cd->mag, hi2c, mmc_addr8, i2c_timeout_ms, mag_cfg) != HAL_OK)
    {
        printf("[FUSION] MagneticSensor_Init (mag) failed\r\n");
        return false;
    }

    // Init radar
    RadarPresence_Default(&cd->radar, cd->cfg.radar_period_ms);
    if (!RadarPresence_Init(&cd->radar, radar_preset))
    {
        printf("[FUSION] RadarPresence_Init failed\r\n");
        return false;
    }

    cd->car_present_fused = false;
    return true;
}

static bool radar_is_on(const CarDetector_t *cd)
{
    if (!cd) return false;
    
    // Use the debounced car_detected flag from radar_presence
    // This already includes score-based filtering, distance filtering, and debouncing
    return cd->radar.car_detected;
}

void CarDetector_Tick(CarDetector_t *cd, uint32_t now_ms)
{
    if (!cd) return;

    // Update both sensors
    MagneticSensor_Tick(&cd->mag, now_ms);
    RadarPresence_Tick(&cd->radar, now_ms);
    
    // Process radar detection with filtering and debouncing
    RadarPresence_ProcessDetection(&cd->radar,
                                    cd->cfg.radar_score_on_x1000,
                                    cd->cfg.radar_score_off_x1000,
                                    cd->cfg.radar_distance_min_mm,
                                    cd->cfg.radar_distance_max_mm,
                                    cd->cfg.radar_debounce_on,
                                    cd->cfg.radar_debounce_off);

    const bool mag_ready = (MagneticSensor_IsBaselineReady(&cd->mag) != 0);
    const bool mag_on    = mag_ready && (MagneticSensor_GetCarPresent(&cd->mag) != 0);
    const bool rad_on    = radar_is_on(cd);

    // Fusion logic:
    // - If both trigger -> immediate PRESENT
    // - If only one triggers -> SUSPECT; confirm after on_confirm_ms
    // - For clearing -> require both off for off_confirm_ms
    if (!cd->car_present_fused)
    {
        if (mag_on && rad_on)
        {
            cd->car_present_fused  = true;
            cd->present_until_ms   = now_ms + cd->cfg.present_hold_ms;
            cd->suspect_since_ms   = 0;
            cd->off_since_ms       = 0;
        }
        else if (mag_on || rad_on)
        {
            if (cd->suspect_since_ms == 0) cd->suspect_since_ms = now_ms;

            if (u32_now_minus(now_ms, cd->suspect_since_ms) >= cd->cfg.on_confirm_ms)
            {
                cd->car_present_fused = true;
                cd->present_until_ms  = now_ms + cd->cfg.present_hold_ms;
                cd->suspect_since_ms  = 0;
                cd->off_since_ms      = 0;
            }
        }
        else
        {
            cd->suspect_since_ms = 0;
        }
    }
    else
    {
        // While present: extend hold if any sensor still indicates presence
        if (mag_on || rad_on)
        {
            cd->present_until_ms = now_ms + cd->cfg.present_hold_ms;
            cd->off_since_ms     = 0;
        }
        else
        {
            // still within hold window => keep present
            if ((int32_t)(now_ms - cd->present_until_ms) < 0)
            {
                // stay present
            }
            else
            {
                if (cd->off_since_ms == 0) cd->off_since_ms = now_ms;

                if (u32_now_minus(now_ms, cd->off_since_ms) >= cd->cfg.off_confirm_ms)
                {
                    cd->car_present_fused = false;
                    cd->off_since_ms      = 0;
                    cd->suspect_since_ms  = 0;
                    cd->present_until_ms  = 0;
                }
            }
        }
    }
}

bool CarDetector_GetCarPresent(const CarDetector_t *cd)
{
    return cd ? cd->car_present_fused : false;
}

int32_t CarDetector_GetRadarScore(const CarDetector_t *cd)
{
    if (!cd) return 0;
    // pick one; intra is usually the “fast” part
    return cd->radar.intra_score_x1000;
}

bool CarDetector_GetRadarPresence(const CarDetector_t *cd)
{
    return cd ? cd->radar.presence_detected : false;
}
