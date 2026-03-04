// Copyright (c) Acconeer AB, 2022-2023
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

/*
 * Board wiring (your project):
 *  - SPI:  SPI1 (CubeMX: hspi1)
 *  - CS:   PB0 -> A121_CS (GPIO output, active low)
 *  - EN:   PA3 -> A121_EN (GPIO output, active high)
 *  - INT:  PA4 -> A121_INT (GPIO input / EXTI, active high)
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include "acc_definitions_common.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"

#include "main.h"
#include "spi.h"
#include "gpio.h"

/* ----------------------------------------------------------------------------
 * SPI handle selection
 * ----------------------------------------------------------------------------
 * Default: use CubeMX-generated hspi1.
 * If you want a custom symbol, define ACC_A121_SPI_HANDLE_EXTERN
 * and provide it in your project.
 */
#ifndef ACC_A121_SPI_HANDLE_EXTERN
#define ACC_A121_SPI_HANDLE (&hspi1)
#else
extern SPI_HandleTypeDef ACC_A121_SPI_HANDLE_EXTERN;
#define ACC_A121_SPI_HANDLE (&ACC_A121_SPI_HANDLE_EXTERN)
#endif

#define SENSOR_COUNT                (1u)
#define STM32_SPI_MAX_TRANSFER_SIZE (65535u)

/* ---------- IRQ helpers ---------- */
static inline void disable_interrupts(void)
{
    __disable_irq();
}

static inline void enable_interrupts(void)
{
    __enable_irq();
    __ISB();
}

/* ---------- Optional DMA support ---------- */
#ifdef STM32_USE_SPI_DMA
static volatile bool spi_transfer_complete = false;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h_spi)
{
    (void)h_spi;
    spi_transfer_complete = true;
}
#endif

/* ----------------------------------------------------------------------------
 * GPIO helpers (mapped to your CubeMX pins)
 * ---------------------------------------------------------------------------- */
static inline void a121_cs(bool high)
{
    HAL_GPIO_WritePin(A121_CS_GPIO_Port, A121_CS_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void a121_en(bool high)
{
    HAL_GPIO_WritePin(A121_EN_GPIO_Port, A121_EN_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline GPIO_PinState a121_int_read(void)
{
    return HAL_GPIO_ReadPin(A121_INT_GPIO_Port, A121_INT_Pin);
}

/* ----------------------------------------------------------------------------
 * Implementation of RSS HAL handlers
 * ---------------------------------------------------------------------------- */

static void acc_hal_integration_sensor_transfer(acc_sensor_id_t sensor_id,
                                                uint8_t *buffer,
                                                size_t buffer_size)
{
    if ((sensor_id == 0u) || (sensor_id > SENSOR_COUNT) || (buffer == NULL))
    {
        Error_Handler();
    }

    const uint32_t SPI_TIMEOUT_MS = 5000u;

    /* Ensure CS idle high then select */
    a121_cs(true);
    a121_cs(false);

#ifdef STM32_USE_SPI_DMA
    /* DMA path (chunk because HAL size is uint16_t) */
    size_t offset = 0u;

    while (offset < buffer_size)
    {
        size_t remaining = buffer_size - offset;
        uint16_t chunk   = (remaining > STM32_SPI_MAX_TRANSFER_SIZE)
                           ? (uint16_t)STM32_SPI_MAX_TRANSFER_SIZE
                           : (uint16_t)remaining;

        spi_transfer_complete = false;

        HAL_StatusTypeDef st =
            HAL_SPI_TransmitReceive_DMA(ACC_A121_SPI_HANDLE,
                                        &buffer[offset],
                                        &buffer[offset],
                                        chunk);

        if (st != HAL_OK)
        {
            a121_cs(true);
            Error_Handler();
        }

        uint32_t start = HAL_GetTick();
        while (!spi_transfer_complete && ((HAL_GetTick() - start) < SPI_TIMEOUT_MS))
        {
            disable_interrupts();
            if (!spi_transfer_complete)
            {
                __WFI();
            }
            enable_interrupts();
        }

        if (!spi_transfer_complete)
        {
            a121_cs(true);
            Error_Handler();
        }

        offset += chunk;
    }

#else
    /* Blocking path (chunk for safety) */
    size_t offset = 0u;

    while (offset < buffer_size)
    {
        size_t remaining = buffer_size - offset;
        uint16_t chunk   = (remaining > STM32_SPI_MAX_TRANSFER_SIZE)
                           ? (uint16_t)STM32_SPI_MAX_TRANSFER_SIZE
                           : (uint16_t)remaining;

        HAL_StatusTypeDef st =
            HAL_SPI_TransmitReceive(ACC_A121_SPI_HANDLE,
                                    &buffer[offset],
                                    &buffer[offset],
                                    chunk,
                                    SPI_TIMEOUT_MS);

        if (st != HAL_OK)
        {
            a121_cs(true);
            Error_Handler();
        }

        offset += chunk;
    }
#endif

    /* De-select sensor */
    a121_cs(true);
}

void acc_hal_integration_sensor_supply_on(acc_sensor_id_t sensor_id)
{
    if ((sensor_id == 0u) || (sensor_id > SENSOR_COUNT))
    {
        Error_Handler();
    }

    /* If you later add a load switch for radar supply, control it here.
       For now, do nothing. */
}

void acc_hal_integration_sensor_supply_off(acc_sensor_id_t sensor_id)
{
    if ((sensor_id == 0u) || (sensor_id > SENSOR_COUNT))
    {
        Error_Handler();
    }

    /* No supply switch currently */
}

void acc_hal_integration_sensor_enable(acc_sensor_id_t sensor_id)
{
    if ((sensor_id == 0u) || (sensor_id > SENSOR_COUNT))
    {
        Error_Handler();
    }

    a121_en(true);

    /* Give sensor time to stabilize after enable */
    acc_integration_sleep_us(2000u);
}

void acc_hal_integration_sensor_disable(acc_sensor_id_t sensor_id)
{
    if ((sensor_id == 0u) || (sensor_id > SENSOR_COUNT))
    {
        Error_Handler();
    }

    a121_en(false);

    /* Small delay to leave sensor in known state */
    acc_integration_sleep_us(2000u);
}

bool acc_hal_integration_wait_for_sensor_interrupt(acc_sensor_id_t sensor_id, uint32_t timeout_ms)
{
    if ((sensor_id == 0u) || (sensor_id > SENSOR_COUNT))
    {
        Error_Handler();
    }

    const uint32_t begin_ms = HAL_GetTick();

    while ((a121_int_read() != GPIO_PIN_SET) && ((HAL_GetTick() - begin_ms) < timeout_ms))
    {
        disable_interrupts();

        if (a121_int_read() != GPIO_PIN_SET)
        {
            __WFI();
        }

        enable_interrupts();
    }

    return (a121_int_read() == GPIO_PIN_SET);
}

uint16_t acc_hal_integration_sensor_count(void)
{
    return (uint16_t)SENSOR_COUNT;
}

const acc_hal_a121_t *acc_hal_rss_integration_get_implementation(void)
{
    static const acc_hal_a121_t hal = {
        .max_spi_transfer_size = STM32_SPI_MAX_TRANSFER_SIZE,

        .mem_alloc = malloc,
        .mem_free  = free,

        .transfer  = acc_hal_integration_sensor_transfer,
        .log       = acc_integration_log,

        .optimization.transfer16 = NULL,
    };

    return &hal;
}
