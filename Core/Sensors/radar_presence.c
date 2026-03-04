#include "radar_presence.h"

#include <stdio.h>
#include <string.h>

#include "acc_detector_presence.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"

#define SENSOR_ID         (1U)
#define SENSOR_TIMEOUT_MS (2000U)

// --- forward (copy from example style) ---
static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *cal_result, void *buffer, uint32_t buffer_size)
{
    bool           status              = false;
    bool           cal_complete        = false;
    const uint16_t calibration_retries = 1U;

    for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
    {
        acc_hal_integration_sensor_disable(SENSOR_ID);
        acc_hal_integration_sensor_enable(SENSOR_ID);

        do
        {
            status = acc_sensor_calibrate(sensor, &cal_complete, cal_result, buffer, buffer_size);

            if (status && !cal_complete)
            {
                status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
            }
        } while (status && !cal_complete);
    }

    if (status)
    {
        acc_hal_integration_sensor_disable(SENSOR_ID);
        acc_hal_integration_sensor_enable(SENSOR_ID);
    }

    return status;
}


static void set_config(acc_detector_presence_config_t *presence_config, RadarPreset_t preset)
{
    switch (preset)
    {
        case RADAR_PRESET_SHORT_RANGE:
            acc_detector_presence_config_start_set(presence_config, 0.20f);
            acc_detector_presence_config_end_set(presence_config, 1.00f);
            break;

        case RADAR_PRESET_MEDIUM_RANGE:
            acc_detector_presence_config_start_set(presence_config, 0.20f);
            acc_detector_presence_config_end_set(presence_config, 2.00f);
            break;

        case RADAR_PRESET_LONG_RANGE:
            acc_detector_presence_config_start_set(presence_config, 0.20f);
            acc_detector_presence_config_end_set(presence_config, 4.00f);
            break;

        case RADAR_PRESET_LOW_POWER_WAKEUP:
            acc_detector_presence_config_start_set(presence_config, 0.20f);
            acc_detector_presence_config_end_set(presence_config, 2.00f);
            acc_detector_presence_config_frame_rate_set(presence_config, 2.0f);
            break;

        case RADAR_PRESET_NONE:
        default:
            break;
    }
}

void RadarPresence_Default(RadarPresence_t *r, uint32_t period_ms)
{
    if (!r) return;
    memset(r, 0, sizeof(*r));
    r->period_ms = (period_ms == 0) ? 100U : period_ms;
    r->next_run_ms = 0;
    r->car_detected = false;
    r->on_streak = 0;
    r->off_streak = 0;
}

bool RadarPresence_Init(RadarPresence_t *r, RadarPreset_t preset)
{
    if (!r) return false;
    RadarPresence_Deinit(r);

    printf("Acconeer software version %s\r\n", acc_version_get());

    // Register HAL (this is the “activation” in your SDK)
    const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();
    if (!acc_rss_hal_register(hal))
    {
        printf("acc_rss_hal_register failed\r\n");
        return false;
    }

    r->config = acc_detector_presence_config_create();
    if (!r->config)
    {
        printf("acc_detector_presence_config_create failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    set_config(r->config, preset);

    r->handle = acc_detector_presence_create(r->config, &r->metadata);
    if (!r->handle)
    {
        printf("acc_detector_presence_create failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    if (!acc_detector_presence_get_buffer_size(r->handle, &r->buffer_size))
    {
        printf("acc_detector_presence_get_buffer_size failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    r->buffer = acc_integration_mem_alloc(r->buffer_size);
    if (!r->buffer)
    {
        printf("buffer alloc failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    // Power + enable sensor (your SDK expects this)
    acc_hal_integration_sensor_supply_on(SENSOR_ID);
    acc_hal_integration_sensor_enable(SENSOR_ID);

    r->sensor = acc_sensor_create(SENSOR_ID);
    if (!r->sensor)
    {
        printf("acc_sensor_create failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    // Calibration (exactly like example)
    acc_cal_result_t cal_result;
    if (!do_sensor_calibration(r->sensor, &cal_result, r->buffer, r->buffer_size))
    {
        printf("do_sensor_calibration failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    if (!acc_detector_presence_prepare(r->handle, r->config, r->sensor, &cal_result, r->buffer, r->buffer_size))
    {
        printf("acc_detector_presence_prepare failed\r\n");
        RadarPresence_Deinit(r);
        return false;
    }

    r->initialized = true;
    return true;
}

void RadarPresence_Deinit(RadarPresence_t *r)
{
    if (!r) return;

    if (r->sensor)
    {
        acc_sensor_destroy(r->sensor);
        r->sensor = NULL;
    }

    // If you want to also disable power/enable pin on shutdown:
    // acc_hal_integration_sensor_disable(SENSOR_ID);
    // acc_hal_integration_sensor_supply_off(SENSOR_ID);

    if (r->buffer)
    {
        acc_integration_mem_free(r->buffer);
        r->buffer = NULL;
    }

    if (r->handle)
    {
        acc_detector_presence_destroy(r->handle);
        r->handle = NULL;
    }

    if (r->config)
    {
        acc_detector_presence_config_destroy(r->config);
        r->config = NULL;
    }

    r->buffer_size = 0;
    r->initialized = false;
}

// Process radar detection with debouncing and filtering
// This should be called after RadarPresence_Tick to update car_detected
void RadarPresence_ProcessDetection(RadarPresence_t *r, 
                                     int32_t score_threshold_on_x1000,
                                     int32_t score_threshold_off_x1000,
                                     int32_t distance_min_mm,
                                     int32_t distance_max_mm,
                                     uint8_t debounce_on,
                                     uint8_t debounce_off)
{
    if (!r) return;

    // Ignore unreliable frames
    if (r->data_saturated || r->frame_delayed) {
        // Don't update detection state on bad frames
        return;
    }

    // Use the maximum of intra and inter scores
    int32_t max_score = (r->intra_score_x1000 > r->inter_score_x1000) ? 
                        r->intra_score_x1000 : r->inter_score_x1000;

    // Check distance filter
    bool distance_ok = (r->distance_mm >= distance_min_mm) && 
                       (r->distance_mm <= distance_max_mm);

    // Determine if current frame indicates presence
    bool frame_indicates_presence = false;
    if (distance_ok && max_score >= score_threshold_on_x1000) {
        frame_indicates_presence = true;
    }

    // Debouncing logic with hysteresis
    if (r->car_detected) {
        // Currently ON: use lower threshold (hysteresis) to turn OFF
        if (frame_indicates_presence && max_score >= score_threshold_off_x1000) {
            // Still above OFF threshold, keep ON
            r->off_streak = 0;
        } else {
            // Below OFF threshold, increment off streak
            if (r->off_streak < 255) r->off_streak++;
            if (r->off_streak >= debounce_off) {
                r->car_detected = false;
                r->off_streak = 0;
                r->on_streak = 0;
            }
        }
    } else {
        // Currently OFF: need to exceed ON threshold
        if (frame_indicates_presence) {
            if (r->on_streak < 255) r->on_streak++;
            if (r->on_streak >= debounce_on) {
                r->car_detected = true;
                r->on_streak = 0;
                r->off_streak = 0;
            }
        } else {
            r->on_streak = 0;
        }
    }
}

void RadarPresence_Tick(RadarPresence_t *r, uint32_t now_ms)
{
    if (!r || !r->initialized) return;
    if ((int32_t)(now_ms - r->next_run_ms) < 0) return;

    acc_detector_presence_result_t result;

    if (!acc_sensor_measure(r->sensor))
    {
        printf("[RADAR] acc_sensor_measure failed\r\n");
        r->next_run_ms = now_ms + r->period_ms;
        return;
    }

    if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
    {
        printf("[RADAR] Sensor interrupt timeout\r\n");
        r->next_run_ms = now_ms + r->period_ms;
        return;
    }

    if (!acc_sensor_read(r->sensor, r->buffer, r->buffer_size))
    {
        printf("[RADAR] acc_sensor_read failed\r\n");
        r->next_run_ms = now_ms + r->period_ms;
        return;
    }

    if (!acc_detector_presence_process(r->handle, r->buffer, &result))
    {
        printf("[RADAR] acc_detector_presence_process failed\r\n");
        r->next_run_ms = now_ms + r->period_ms;
        return;
    }

    // Store raw results
    r->presence_detected = result.presence_detected;
    r->intra_score_x1000 = (int32_t)(result.intra_presence_score * 1000.0f);
    r->inter_score_x1000 = (int32_t)(result.inter_presence_score * 1000.0f);
    r->distance_mm       = (int32_t)(result.presence_distance * 1000.0f);
    r->data_saturated    = result.processing_result.data_saturated;
    r->frame_delayed     = result.processing_result.frame_delayed;

    // Optional: handle recalibration_needed like example (recommended)
    // if (result.processing_result.calibration_needed) { recalibrate + prepare again }

    r->next_run_ms = now_ms + r->period_ms;
}
