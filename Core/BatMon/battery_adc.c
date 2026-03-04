/*
 * battery_adc.c
 *
 *  Created on: 03.03.2026
 *      Author: FI
 */

#include "battery_adc.h"
#include "main.h"   // for BAT_MON_Pin, BAT_MON_GPIO_Port (CubeMX)
#include "adc.h"    // for hadc1 extern if you prefer, but we pass handle in init

/* Module state */
static ADC_HandleTypeDef *s_hadc = NULL;
static uint32_t s_vdda_mV = 3300u;

static void bat_mon_enable(bool en)
{
    HAL_GPIO_WritePin(BAT_MON_GPIO_Port, BAT_MON_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* One conversion, returns raw 12-bit */
static bool adc_read_once(uint16_t *out_raw)
{
    if (!s_hadc || !out_raw) return false;

    if (HAL_ADC_Start(s_hadc) != HAL_OK) return false;

    if (HAL_ADC_PollForConversion(s_hadc, BAT_ADC_POLL_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_ADC_Stop(s_hadc);
        return false;
    }

    uint32_t v = HAL_ADC_GetValue(s_hadc);
    (void)HAL_ADC_Stop(s_hadc);

    if (v > 0xFFFFu) v = 0xFFFFu;
    *out_raw = (uint16_t)v;
    return true;
}

/* Average N samples, includes a dummy throwaway */
static bool adc_read_avg(uint16_t *out_avg)
{
    if (!out_avg) return false;

    /* Dummy sample (discard) */
    uint16_t dummy = 0;
    (void)adc_read_once(&dummy);

    uint32_t sum = 0;
    for (uint32_t i = 0; i < BAT_ADC_SAMPLES; i++)
    {
        uint16_t raw = 0;
        if (!adc_read_once(&raw)) return false;
        sum += raw;
    }

    *out_avg = (uint16_t)(sum / BAT_ADC_SAMPLES);
    return true;
}

void BAT_ADC_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
    s_vdda_mV = 3300u;

    /* Ensure disabled by default */
    bat_mon_enable(false);
}

void BAT_ADC_SetVDDA_mV(uint32_t vdda_mV)
{
    if (vdda_mV < 1000u) vdda_mV = 1000u;
    if (vdda_mV > 3600u) vdda_mV = 3600u;
    s_vdda_mV = vdda_mV;
}

/* Convert raw ADC to node voltage (mV) */
static uint32_t raw_to_node_mV(uint16_t raw)
{
    /* 12-bit: 0..4095 */
    return (uint32_t)((((uint64_t)raw) * s_vdda_mV) / 4095u);
}

/* Convert node voltage to battery voltage based on divider */
static uint32_t node_to_bat_mV(uint32_t v_node_mV)
{
    /* Vbat = Vnode * (Rtop + Rbot) / Rbot */
    const float rtop = BAT_ADC_RTOP_OHMS;
    const float rbot = BAT_ADC_RBOT_OHMS;

    float vbat = (float)v_node_mV * (rtop + rbot) / rbot;

    if (vbat < 0.0f) vbat = 0.0f;
    return (uint32_t)(vbat + 0.5f);
}

uint32_t BAT_ADC_ReadNode_mV(uint16_t *out_raw_avg)
{
    if (!s_hadc) return 0;

    bat_mon_enable(true);
    HAL_Delay(BAT_ADC_SETTLE_MS);

    uint16_t raw_avg = 0;
    bool ok = adc_read_avg(&raw_avg);

    bat_mon_enable(false);

    if (!ok) return 0;

    if (out_raw_avg) *out_raw_avg = raw_avg;

    return raw_to_node_mV(raw_avg);
}

uint32_t BAT_ADC_Read_mV(uint16_t *out_raw_avg)
{
    uint16_t raw_avg = 0;
    uint32_t v_node_mV = BAT_ADC_ReadNode_mV(&raw_avg);
    if (v_node_mV == 0) return 0;

    if (out_raw_avg) *out_raw_avg = raw_avg;

    return node_to_bat_mV(v_node_mV);
}
