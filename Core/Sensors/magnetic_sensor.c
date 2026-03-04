/*
 * magnetic_sensor.c
 */

#include "magnetic_sensor.h"
#include <string.h>
#include <stdio.h>

// ---------- Private helpers (integer math, no float printf needed) ----------

// uT = counts * 100 / 16384
// uT(x100) = counts * 10000 / 16384
static int32_t counts_to_uT_x100(int32_t counts)
{
    int64_t num = (int64_t)counts * 10000;
    // rounding
    if (num >= 0) num += 8192;
    else          num -= 8192;
    return (int32_t)(num / 16384);
}

static uint32_t isqrt_u64(uint64_t x)
{
    uint64_t op = x;
    uint64_t res = 0;
    uint64_t one = (uint64_t)1 << 62;

    while (one > op) one >>= 2;

    while (one != 0) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return (uint32_t)res;
}

static uint32_t mag_uT_x100_from_axes(int32_t x_uT_x100, int32_t y_uT_x100, int32_t z_uT_x100)
{
    int64_t x = x_uT_x100;
    int64_t y = y_uT_x100;
    int64_t z = z_uT_x100;

    uint64_t mag2 = (uint64_t)(x * x) + (uint64_t)(y * y) + (uint64_t)(z * z);
    return isqrt_u64(mag2);
}

// ring: overwrite oldest when full
static void ring_push(MagneticSensor_t *s, const MagneticSensorSample_t *in)
{
    s->ring[s->wr] = *in;
    s->wr = (uint16_t)((s->wr + 1u) % MAGNETIC_SENSOR_RING_SIZE);

    if (s->count < MAGNETIC_SENSOR_RING_SIZE) {
        s->count++;
    } else {
        // full -> drop oldest
        s->rd = (uint16_t)((s->rd + 1u) % MAGNETIC_SENSOR_RING_SIZE);
    }

    s->last = *in;
    s->last_valid = 1;
}

static int32_t iabs32(int32_t v) { return (v < 0) ? -v : v; }

// ---------- Public API ----------

void MagneticSensor_DefaultConfig(MagneticSensorConfig_t *cfg)
{
    if (!cfg) return;
    cfg->sample_period_ms = 20;       // 50 Hz
    cfg->baseline_samples = 100;      // ~2 seconds at 50 Hz

    cfg->thresh_on_uT_x100  = 600;    // 6.00 uT
    cfg->thresh_off_uT_x100 = 300;    // 3.00 uT

    cfg->debounce_on  = 3;            // 3 consecutive samples
    cfg->debounce_off = 8;            // a bit more conservative off

    cfg->hold_on_ms = 2000;          // 2 seconds latch
}

HAL_StatusTypeDef MagneticSensor_Init(MagneticSensor_t *s,
                                      I2C_HandleTypeDef *hi2c,
                                      uint16_t mmc_addr8,
                                      uint32_t i2c_timeout_ms,
                                      const MagneticSensorConfig_t *cfg)
{
    if (!s || !hi2c) return HAL_ERROR;

    memset(s, 0, sizeof(*s));

    if (cfg) s->cfg = *cfg;
    else MagneticSensor_DefaultConfig(&s->cfg);

    // initialize MMC
    HAL_StatusTypeDef st = MMC5983_Init(&s->mmc, hi2c, mmc_addr8, i2c_timeout_ms);
    if (st != HAL_OK) return st;

    // schedule first sample shortly after boot
    s->next_sample_ms = 0;

    // internal state machine starts idle
    s->st = 0;
    s->st_deadline_ms = 0;

    return HAL_OK;
}

uint16_t MagneticSensor_Available(const MagneticSensor_t *s)
{
    return s ? s->count : 0;
}

bool MagneticSensor_Pop(MagneticSensor_t *s, MagneticSensorSample_t *out)
{
    if (!s || !out) return false;
    if (s->count == 0) return false;

    *out = s->ring[s->rd];
    s->rd = (uint16_t)((s->rd + 1u) % MAGNETIC_SENSOR_RING_SIZE);
    s->count--;
    return true;
}

bool MagneticSensor_GetLatest(const MagneticSensor_t *s, MagneticSensorSample_t *out)
{
    if (!s || !out) return false;
    if (!s->last_valid) return false;
    *out = s->last;
    return true;
}

uint8_t MagneticSensor_GetCarPresent(const MagneticSensor_t *s)
{
    return s ? s->car_present : 0;
}

uint8_t MagneticSensor_IsBaselineReady(const MagneticSensor_t *s)
{
    return s ? s->baseline_ready : 0;
}

uint32_t MagneticSensor_GetI2CErrors(const MagneticSensor_t *s)
{
    return s ? s->i2c_errors : 0;
}

uint32_t MagneticSensor_GetTimeouts(const MagneticSensor_t *s)
{
    return s ? s->timeouts : 0;
}

// ---------- Nonblocking sampling state machine ----------
//
// States:
// 0 IDLE (wait until next_sample_ms)
// 1 RESET_PULSE
// 2 TRIG_RESET_MEAS
// 3 WAIT_RESET_DONE
// 4 READ_RESET
// 5 SET_PULSE
// 6 TRIG_SET_MEAS
// 7 WAIT_SET_DONE
// 8 READ_SET + COMPUTE + PUSH
//
void MagneticSensor_Tick(MagneticSensor_t *s, uint32_t now_ms)
{
    if (!s) return;

    // wait for next period
    if (s->st == 0) {
        if ((int32_t)(now_ms - s->next_sample_ms) < 0) {
            return;
        }
        // start new sample sequence
        s->st = 1;
    }

    // state machine progresses one step per Tick call (fast calls are fine)
    switch (s->st)
    {
        case 1: // RESET_PULSE
        {
            // Clear status register (write-1-to-clear for MEAS_M_DONE bit)
            if (MMC5983_ClearStatus(&s->mmc, MMC5983_STATUS_MEAS_M_DONE) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            if (MMC5983_PulseReset(&s->mmc) != HAL_OK)        { s->i2c_errors++; s->st = 0; break; }
            s->st_deadline_ms = now_ms + 5; // give it a moment (no delay)
            s->st = 2;
        } break;

        case 2: // TRIG_RESET_MEAS
        {
            if ((int32_t)(now_ms - s->st_deadline_ms) < 0) return;
            if (MMC5983_TriggerMag(&s->mmc) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            s->st_deadline_ms = now_ms + 30; // max wait for done
            s->st = 3;
        } break;

        case 3: // WAIT_RESET_DONE
        {
            uint8_t st = 0;
            if (MMC5983_ReadStatus(&s->mmc, &st) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            if (st & MMC5983_STATUS_MEAS_M_DONE) {
                s->st = 4;
                break;
            }
            if ((int32_t)(now_ms - s->st_deadline_ms) >= 0) {
                s->timeouts++;
                s->st = 0;
            }
        } break;

        case 4: // READ_RESET
        {
            if (MMC5983_ReadRaw18(&s->mmc, &s->raw_reset) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            s->st = 5;
        } break;

        case 5: // SET_PULSE
        {
            // Clear status register (write-1-to-clear for MEAS_M_DONE bit)
            if (MMC5983_ClearStatus(&s->mmc, MMC5983_STATUS_MEAS_M_DONE) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            if (MMC5983_PulseSet(&s->mmc) != HAL_OK)          { s->i2c_errors++; s->st = 0; break; }
            s->st_deadline_ms = now_ms + 5;
            s->st = 6;
        } break;

        case 6: // TRIG_SET_MEAS
        {
            if ((int32_t)(now_ms - s->st_deadline_ms) < 0) return;
            if (MMC5983_TriggerMag(&s->mmc) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            s->st_deadline_ms = now_ms + 30;
            s->st = 7;
        } break;

        case 7: // WAIT_SET_DONE
        {
            uint8_t st = 0;
            if (MMC5983_ReadStatus(&s->mmc, &st) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }
            if (st & MMC5983_STATUS_MEAS_M_DONE) {
                s->st = 8;
                break;
            }
            if ((int32_t)(now_ms - s->st_deadline_ms) >= 0) {
                s->timeouts++;
                s->st = 0;
            }
        } break;

        case 8: // READ_SET + COMPUTE + PUSH
        {
            if (MMC5983_ReadRaw18(&s->mmc, &s->raw_set) != HAL_OK) { s->i2c_errors++; s->st = 0; break; }

            MMC5983_FieldCounts c = MMC5983_ComputeCounts(&s->raw_set, &s->raw_reset);

            MagneticSensorSample_t out;
            memset(&out, 0, sizeof(out));
            out.timestamp_ms = now_ms;

            out.x_counts = c.x;
            out.y_counts = c.y;
            out.z_counts = c.z;

            out.x_uT_x100 = counts_to_uT_x100(out.x_counts);
            out.y_uT_x100 = counts_to_uT_x100(out.y_counts);
            out.z_uT_x100 = counts_to_uT_x100(out.z_counts);

            out.mag_uT_x100 = mag_uT_x100_from_axes(out.x_uT_x100, out.y_uT_x100, out.z_uT_x100);

            // ---- baseline calibration (freeze after N samples) ----
            if (!s->baseline_ready) {
                s->baseline_acc_sum += out.mag_uT_x100;
                s->baseline_acc_n++;

                if (s->cfg.baseline_samples > 0 && s->baseline_acc_n >= s->cfg.baseline_samples) {
                    // Calculate average with proper rounding
                    s->baseline_mag_uT_x100 = (uint32_t)((s->baseline_acc_sum + (s->baseline_acc_n / 2)) / s->baseline_acc_n);
                    s->baseline_ready = 1;

                    // reset detection state when baseline becomes valid
                    s->car_present = 0;
                    s->car_on_until_ms = 0;
                    s->on_streak = 0;
                    s->off_streak = 0;
                }
            }

            uint32_t baseline = s->baseline_ready ? s->baseline_mag_uT_x100 : 0;
            out.baseline_mag_uT_x100 = baseline;


            // ---- delta + detection (hysteresis + hold time) ----
            out.delta_mag_uT_x100 = (int32_t)out.mag_uT_x100 - (int32_t)baseline;

            if (s->baseline_ready) {
                int32_t absd = iabs32(out.delta_mag_uT_x100);

                // If we are currently ON and still within hold time, keep it ON no matter what.
                // (cast to int32_t to handle tick wrap safely)
                // If ON and still within hold window, keep ON (wrap-safe signed compare)
                if (s->car_present && (int32_t)(now_ms - s->car_on_until_ms) < 0) {
                    // still latched ON -> skip OFF evaluation
                } else {
                    // normal hysteresis + debounce logic
                    if (!s->car_present) {
                        if (absd >= s->cfg.thresh_on_uT_x100) {
                            if (s->on_streak < 255) s->on_streak++;
                        } else {
                            s->on_streak = 0;
                        }

                        if (s->on_streak >= s->cfg.debounce_on) {
                            s->car_present = 1;
                            s->car_on_until_ms = now_ms + s->cfg.hold_on_ms;
                            s->on_streak = 0;
                            s->off_streak = 0;
                        }
                    } else {
                        if (absd <= s->cfg.thresh_off_uT_x100) {
                            if (s->off_streak < 255) s->off_streak++;
                        } else {
                            s->off_streak = 0;
                        }

                        if (s->off_streak >= s->cfg.debounce_off) {
                            s->car_present = 0;
                            s->car_on_until_ms = 0;
                            s->on_streak = 0;
                            s->off_streak = 0;
                        }
                    }
                }

            } else {
                // during calibration always "no car"
                s->car_present = 0;
                s->car_on_until_ms = 0;
                s->on_streak = 0;
                s->off_streak = 0;
            }

            out.car_present = s->car_present;

            ring_push(s, &out);
            s->samples_ok++;

            // schedule next sample (fixed rate)
            s->next_sample_ms = now_ms + s->cfg.sample_period_ms;

            // back to idle
            s->st = 0;
        } break;

        default:
            s->st = 0;
            break;
    }
}
