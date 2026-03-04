/*
 * mmc5983.c
 */

#include "mmc5983.h"

// ---- Internal helpers ----
static HAL_StatusTypeDef mmc_read(MMC5983_Handle *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr8, reg, I2C_MEMADD_SIZE_8BIT, buf, len, dev->timeout_ms);
}

static HAL_StatusTypeDef mmc_write_u8(MMC5983_Handle *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr8, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, dev->timeout_ms);
}

HAL_StatusTypeDef MMC5983_Init(MMC5983_Handle *dev,
                               I2C_HandleTypeDef *hi2c,
                               uint16_t addr8,
                               uint32_t timeout_ms)
{
    if (!dev || !hi2c) return HAL_ERROR;

    dev->hi2c = hi2c;
    dev->addr8 = addr8;
    dev->timeout_ms = timeout_ms;
    dev->product_id = 0;

    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->addr8, 3, dev->timeout_ms) != HAL_OK)
        return HAL_ERROR;

    uint8_t id = 0;
    if (MMC5983_ReadProductId(dev, &id) != HAL_OK)
        return HAL_ERROR;

    dev->product_id = id;
    return HAL_OK;
}

HAL_StatusTypeDef MMC5983_ReadProductId(MMC5983_Handle *dev, uint8_t *id)
{
    if (!dev || !id) return HAL_ERROR;
    return mmc_read(dev, MMC5983_REG_PRODUCT_ID, id, 1);
}

HAL_StatusTypeDef MMC5983_WriteCtrl0(MMC5983_Handle *dev, uint8_t ctrl0)
{
    if (!dev) return HAL_ERROR;
    return mmc_write_u8(dev, MMC5983_REG_CTRL0, ctrl0);
}

HAL_StatusTypeDef MMC5983_ReadStatus(MMC5983_Handle *dev, uint8_t *status)
{
    if (!dev || !status) return HAL_ERROR;
    return mmc_read(dev, MMC5983_REG_STATUS, status, 1);
}

HAL_StatusTypeDef MMC5983_ClearStatus(MMC5983_Handle *dev, uint8_t mask_w1c)
{
    // STATUS register supports write-1-to-clear for interrupt bits.
    // We'll write mask bits to clear them.
    if (!dev) return HAL_ERROR;
    return mmc_write_u8(dev, MMC5983_REG_STATUS, mask_w1c);
}

HAL_StatusTypeDef MMC5983_TriggerMag(MMC5983_Handle *dev)
{
    if (!dev) return HAL_ERROR;
    return MMC5983_WriteCtrl0(dev, (uint8_t)MMC5983_CTRL0_TM_M);
}

HAL_StatusTypeDef MMC5983_PulseSet(MMC5983_Handle *dev)
{
    if (!dev) return HAL_ERROR;
    return MMC5983_WriteCtrl0(dev, (uint8_t)MMC5983_CTRL0_SET);
}

HAL_StatusTypeDef MMC5983_PulseReset(MMC5983_Handle *dev)
{
    if (!dev) return HAL_ERROR;
    return MMC5983_WriteCtrl0(dev, (uint8_t)MMC5983_CTRL0_RESET);
}

HAL_StatusTypeDef MMC5983_ReadRaw18(MMC5983_Handle *dev, MMC5983_Raw18 *raw)
{
    if (!dev || !raw) return HAL_ERROR;

    uint8_t b[7] = {0};
    HAL_StatusTypeDef st = mmc_read(dev, MMC5983_REG_XOUT0, b, 7);
    if (st != HAL_OK) return st;

    raw->x = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | ((b[6] >> 6) & 0x03);
    raw->y = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x03);
    raw->z = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x03);
    return HAL_OK;
}

MMC5983_FieldCounts MMC5983_ComputeCounts(const MMC5983_Raw18 *raw_set,
                                          const MMC5983_Raw18 *raw_reset)
{
    MMC5983_FieldCounts c;
    c.x = ((int32_t)raw_set->x - (int32_t)raw_reset->x) / 2;
    c.y = ((int32_t)raw_set->y - (int32_t)raw_reset->y) / 2;
    c.z = ((int32_t)raw_set->z - (int32_t)raw_reset->z) / 2;
    return c;
}
