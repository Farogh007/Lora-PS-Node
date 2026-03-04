/*
 * radar_presence.h
 *
 *  Created on: 08.02.2026
 *      Author: FI
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "acc_detector_presence.h"
#include "acc_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RADAR_PRESET_NONE = 0,
    RADAR_PRESET_SHORT_RANGE,
    RADAR_PRESET_MEDIUM_RANGE,
    RADAR_PRESET_LONG_RANGE,
    RADAR_PRESET_LOW_POWER_WAKEUP,
} RadarPreset_t;

typedef struct
{
    // Scheduling
    uint32_t period_ms;     // e.g. 100ms (10 Hz)
    uint32_t next_run_ms;

    // Latest raw output from Acconeer (scores scaled *1000)
    bool     presence_detected;      // Raw boolean from Acconeer
    int32_t  intra_score_x1000;      // Intra-frame presence score
    int32_t  inter_score_x1000;      // Inter-frame presence score
    int32_t  distance_mm;            // Distance in mm
    bool     data_saturated;
    bool     frame_delayed;

    // Filtered/debounced output
    bool     car_detected;           // Debounced car detection
    uint8_t  on_streak;              // Consecutive detections
    uint8_t  off_streak;             // Consecutive non-detections

    // Internal Acconeer handles/resources
    acc_detector_presence_config_t  *config;
    acc_detector_presence_handle_t  *handle;
    acc_detector_presence_metadata_t metadata;
    acc_sensor_t                    *sensor;
    void                            *buffer;
    uint32_t                         buffer_size;

    bool                             initialized;
} RadarPresence_t;

void RadarPresence_Default(RadarPresence_t *r, uint32_t period_ms);
bool RadarPresence_Init(RadarPresence_t *r, RadarPreset_t preset);
void RadarPresence_Deinit(RadarPresence_t *r);

// Call often (from while(1)); runs one measurement when period elapses
void RadarPresence_Tick(RadarPresence_t *r, uint32_t now_ms);

// Process detection with filtering, debouncing, and thresholds
// Call this after RadarPresence_Tick to update car_detected flag
void RadarPresence_ProcessDetection(RadarPresence_t *r,
                                     int32_t score_threshold_on_x1000,
                                     int32_t score_threshold_off_x1000,
                                     int32_t distance_min_mm,
                                     int32_t distance_max_mm,
                                     uint8_t debounce_on,
                                     uint8_t debounce_off);

#ifdef __cplusplus
}
#endif

