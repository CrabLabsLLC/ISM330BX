/**
 * @file ism330bx.c
 * @brief ISM330BX 6-axis IMU driver implementation (minimal scope).
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#include "ism330bx.h"

#include <stddef.h>
#include <string.h>

/* ── Constants ────────────────────────────────────────────────────────────── */

/* Per-axis sensitivity (LSB → milli-units) at each full-scale.
 * Accel: count × FS_g × 1000 / 32768 → milli-g.
 *   2g  → 0.061 mg/LSB → 61   nano-g/LSB ×1000
 *   4g  → 0.122 mg/LSB → 122  nano-g/LSB ×1000
 *   8g  → 0.244 mg/LSB → 244  nano-g/LSB ×1000
 * Stored ×1000 (nano-g/LSB) so int math yields milli-g via /1e6. */
static const uint32_t ism330bx_accel_sens_nano_2g = 61000U;
static const uint32_t ism330bx_accel_sens_nano_4g = 122000U;
static const uint32_t ism330bx_accel_sens_nano_8g = 244000U;

/* Gyro: count × FS_dps × 1000 / 32768 → milli-dps.
 *   125dps  →  4.375 mdps/LSB → 4375  µdps/LSB
 *   250dps  →  8.75  mdps/LSB → 8750
 *   500dps  → 17.5   mdps/LSB → 17500
 *   1000dps → 35     mdps/LSB → 35000
 *   2000dps → 70     mdps/LSB → 70000
 *   4000dps → 140    mdps/LSB → 140000
 * Stored ×1000 (micro-dps/LSB) for integer milli-dps via /1000. */
static const uint32_t ism330bx_gyro_sens_micro_125_dps = 4375U;
static const uint32_t ism330bx_gyro_sens_micro_250_dps = 8750U;
static const uint32_t ism330bx_gyro_sens_micro_500_dps = 17500U;
static const uint32_t ism330bx_gyro_sens_micro_1000_dps = 35000U;
static const uint32_t ism330bx_gyro_sens_micro_2000_dps = 70000U;
static const uint32_t ism330bx_gyro_sens_micro_4000_dps = 140000U;

static const uint32_t ism330bx_reset_timeout_ms = 10U;

/* ── Forward declarations ─────────────────────────────────────────────────── */

static ISM330BXError ism330bxReadRegisters(const ISM330BX* const dev,
                                           const uint8_t reg,
                                           void* const data,
                                           const uint16_t length);
static ISM330BXError ism330bxWriteRegisters(const ISM330BX* const dev,
                                            const uint8_t reg,
                                            const void* const data,
                                            const uint16_t length);
static ISM330BXError ism330bxReadByte(const ISM330BX* const dev,
                                      const uint8_t reg,
                                      uint8_t* const value);
static ISM330BXError ism330bxWriteByte(const ISM330BX* const dev,
                                       const uint8_t reg,
                                       const uint8_t value);
static ISM330BXError ism330bxModifyByte(const ISM330BX* const dev,
                                        const uint8_t reg,
                                        const uint8_t mask,
                                        const uint8_t value);
static ISM330BXError ism330bxApplyAccelConfig(ISM330BX* const dev,
                                              const ISM330BXAccelConfig* const cfg);
static ISM330BXError ism330bxApplyGyroConfig(ISM330BX* const dev,
                                             const ISM330BXGyroConfig* const cfg);
static uint32_t ism330bxAccelSensNanoG(const ISM330BXAccelFullScale fs);
static uint32_t ism330bxGyroSensMicroDps(const ISM330BXGyroFullScale fs);

/* ── 1. Initialization / Lifecycle / Identity ─────────────────────────────── */

ISM330BXError ism330bxInit(ISM330BX* const       dev,
                           const ISM330BXConfig* const config,
                           const ISM330BXHAL* const    hal)
{
	if (dev == NULL || config == NULL || hal == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (hal->readReg == NULL || hal->writeReg == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;

	dev->hal = *hal;
	dev->accel_full_scale = config->accel.full_scale;
	dev->gyro_full_scale = config->gyro.full_scale;
	dev->is_initialized = false;

	/* Verify chip presence via WHO_AM_I before issuing any writes. */
	uint8_t who_am_i = 0U;
	if (dev->hal.readReg(ISM330BX_REG_WHO_AM_I, &who_am_i, 1U) != 0)
		return ISM330BX_ERROR_COMM_FAIL;
	if (who_am_i != ISM330BX_WHO_AM_I_VALUE)
		return ISM330BX_ERROR_BAD_ID;

	/* Software reset, then poll until the SW_RESET bit clears. */
	const uint8_t sw_reset = ISM330BX_CTRL3_SW_RESET_BIT;
	if (dev->hal.writeReg(ISM330BX_REG_CTRL3, &sw_reset, 1U) != 0)
		return ISM330BX_ERROR_COMM_FAIL;
	bool reset_cleared = false;
	for (uint32_t i = 0U; i < ism330bx_reset_timeout_ms; i++)
	{
		if (dev->hal.delayMs != NULL)
			dev->hal.delayMs(1U);
		uint8_t ctrl3 = 0U;
		if (dev->hal.readReg(ISM330BX_REG_CTRL3, &ctrl3, 1U) != 0)
			return ISM330BX_ERROR_COMM_FAIL;
		if ((ctrl3 & ISM330BX_CTRL3_SW_RESET_BIT) == 0U)
		{
			reset_cleared = true;
			break;
		}
	}
	if (!reset_cleared)
		return ISM330BX_ERROR_TIMEOUT;

	/* CTRL3: enable IF_INC and BDU per config. */
	uint8_t ctrl3_value = ISM330BX_CTRL3_IF_INC_BIT;
	if (config->block_data_update)
		ctrl3_value |= ISM330BX_CTRL3_BDU_BIT;
	if (dev->hal.writeReg(ISM330BX_REG_CTRL3, &ctrl3_value, 1U) != 0)
		return ISM330BX_ERROR_COMM_FAIL;

	/* IF_CFG: INT pin polarity and drive type. */
	uint8_t if_cfg = 0U;
	if (config->interrupt_polarity == ISM330BX_INT_POL_ACTIVE_LOW)
		if_cfg |= ISM330BX_IF_CFG_H_LACTIVE_BIT;
	if (config->interrupt_drive == ISM330BX_INT_DRIVE_OPEN_DRAIN)
		if_cfg |= ISM330BX_IF_CFG_PP_OD_BIT;
	if (dev->hal.writeReg(ISM330BX_REG_IF_CFG, &if_cfg, 1U) != 0)
		return ISM330BX_ERROR_COMM_FAIL;

	/* is_initialized must be true before ApplyAccel/GyroConfig (they go through
	 * the public Set* helpers which gate on it). */
	dev->is_initialized = true;

	const ISM330BXError accel_err = ism330bxApplyAccelConfig(dev, &config->accel);
	if (accel_err != ISM330BX_ERROR_OK)
	{
		dev->is_initialized = false;
		return accel_err;
	}
	const ISM330BXError gyro_err = ism330bxApplyGyroConfig(dev, &config->gyro);
	if (gyro_err != ISM330BX_ERROR_OK)
	{
		dev->is_initialized = false;
		return gyro_err;
	}

	return ISM330BX_ERROR_OK;
}

ISM330BXError ism330bxReset(ISM330BX* const dev)
{
	if (dev == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	const uint8_t sw_reset = ISM330BX_CTRL3_SW_RESET_BIT;
	if (dev->hal.writeReg(ISM330BX_REG_CTRL3, &sw_reset, 1U) != 0)
		return ISM330BX_ERROR_COMM_FAIL;
	for (uint32_t i = 0U; i < ism330bx_reset_timeout_ms; i++)
	{
		if (dev->hal.delayMs != NULL)
			dev->hal.delayMs(1U);
		uint8_t ctrl3 = 0U;
		if (dev->hal.readReg(ISM330BX_REG_CTRL3, &ctrl3, 1U) != 0)
			return ISM330BX_ERROR_COMM_FAIL;
		if ((ctrl3 & ISM330BX_CTRL3_SW_RESET_BIT) == 0U)
			return ISM330BX_ERROR_OK;
	}
	return ISM330BX_ERROR_TIMEOUT;
}

ISM330BXError ism330bxGetWHOAMI(const ISM330BX* const dev, uint8_t* const who_am_i)
{
	if (dev == NULL || who_am_i == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	return ism330bxReadByte(dev, ISM330BX_REG_WHO_AM_I, who_am_i);
}

/* ── 2. Accelerometer ─────────────────────────────────────────────────────── */

ISM330BXError ism330bxSetAccelConfig(ISM330BX* const            dev,
                                     const ISM330BXAccelConfig* const config)
{
	if (dev == NULL || config == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	return ism330bxApplyAccelConfig(dev, config);
}

ISM330BXError ism330bxReadAccelRaw(const ISM330BX* const dev,
                                   ISM330BXAxesRaw* const      out)
{
	if (dev == NULL || out == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	/* Accel block on BX is Z,Y,X (NOT X,Y,Z): 0x28=Z_L, 0x29=Z_H, 0x2A=Y_L,
	 * 0x2B=Y_H, 0x2C=X_L, 0x2D=X_H. Re-order into standard {x,y,z}. */
	uint8_t             buf[6];
	const ISM330BXError err = ism330bxReadRegisters(dev, ISM330BX_REG_OUTZ_L_A, buf, 6U);
	if (err != ISM330BX_ERROR_OK)
		return err;

	out->z = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
	out->y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
	out->x = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
	return ISM330BX_ERROR_OK;
}

ISM330BXError ism330bxReadAccelMilliG(const ISM330BX* const dev,
                                      ISM330BXAxesMilli* const    out)
{
	if (dev == NULL || out == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	ISM330BXAxesRaw     raw;
	const ISM330BXError err = ism330bxReadAccelRaw(dev, &raw);
	if (err != ISM330BX_ERROR_OK)
		return err;

	const int64_t sens_nano = (int64_t)ism330bxAccelSensNanoG(dev->accel_full_scale);
	out->x = (int32_t)(((int64_t)raw.x * sens_nano) / 1000000);
	out->y = (int32_t)(((int64_t)raw.y * sens_nano) / 1000000);
	out->z = (int32_t)(((int64_t)raw.z * sens_nano) / 1000000);
	return ISM330BX_ERROR_OK;
}

/* ── 3. Gyroscope ─────────────────────────────────────────────────────────── */

ISM330BXError ism330bxSetGyroConfig(ISM330BX* const           dev,
                                    const ISM330BXGyroConfig* const config)
{
	if (dev == NULL || config == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	return ism330bxApplyGyroConfig(dev, config);
}

ISM330BXError ism330bxReadGyroRaw(const ISM330BX* const dev,
                                  ISM330BXAxesRaw* const      out)
{
	if (dev == NULL || out == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	uint8_t             buf[6];
	const ISM330BXError err = ism330bxReadRegisters(dev, ISM330BX_REG_OUTX_L_G, buf, 6U);
	if (err != ISM330BX_ERROR_OK)
		return err;

	out->x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
	out->y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
	out->z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
	return ISM330BX_ERROR_OK;
}

ISM330BXError ism330bxReadGyroMilliDPS(const ISM330BX* const dev,
                                       ISM330BXAxesMilli* const    out)
{
	if (dev == NULL || out == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	ISM330BXAxesRaw     raw;
	const ISM330BXError err = ism330bxReadGyroRaw(dev, &raw);
	if (err != ISM330BX_ERROR_OK)
		return err;

	const int64_t sens_micro = (int64_t)ism330bxGyroSensMicroDps(dev->gyro_full_scale);
	out->x = (int32_t)(((int64_t)raw.x * sens_micro) / 1000);
	out->y = (int32_t)(((int64_t)raw.y * sens_micro) / 1000);
	out->z = (int32_t)(((int64_t)raw.z * sens_micro) / 1000);
	return ISM330BX_ERROR_OK;
}

/* ── 4. Temperature ───────────────────────────────────────────────────────── */

ISM330BXError ism330bxReadTempRaw(const ISM330BX* const dev, int16_t* const raw)
{
	if (dev == NULL || raw == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	uint8_t             buf[2];
	const ISM330BXError err = ism330bxReadRegisters(dev, ISM330BX_REG_OUT_TEMP_L, buf, 2U);
	if (err != ISM330BX_ERROR_OK)
		return err;

	*raw = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
	return ISM330BX_ERROR_OK;
}

ISM330BXError ism330bxReadTempMilliCelsius(const ISM330BX* const dev,
                                           int32_t* const              milli_celsius)
{
	if (dev == NULL || milli_celsius == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	int16_t             raw = 0;
	const ISM330BXError err = ism330bxReadTempRaw(dev, &raw);
	if (err != ISM330BX_ERROR_OK)
		return err;

	/* Datasheet: T(°C) = raw / 256 + 25. Returning milli-°C: (raw * 1000)/256 + 25000. */
	*milli_celsius = ((int32_t)raw * 1000) / 256 + 25000;
	return ISM330BX_ERROR_OK;
}

/* ── 5. Status ────────────────────────────────────────────────────────────── */

ISM330BXError ism330bxGetStatus(const ISM330BX* const dev,
                                ISM330BXStatus* const       status)
{
	if (dev == NULL || status == NULL)
		return ISM330BX_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return ISM330BX_ERROR_NOT_INIT;

	uint8_t             reg = 0U;
	const ISM330BXError err = ism330bxReadByte(dev, ISM330BX_REG_STATUS_REG, &reg);
	if (err != ISM330BX_ERROR_OK)
		return err;

	status->accel_data_ready = (reg & ISM330BX_STATUS_XLDA_BIT) != 0U;
	status->gyro_data_ready  = (reg & ISM330BX_STATUS_GDA_BIT) != 0U;
	status->temp_data_ready  = (reg & ISM330BX_STATUS_TDA_BIT) != 0U;
	return ISM330BX_ERROR_OK;
}

/* ── Internal helpers ─────────────────────────────────────────────────────── */

static ISM330BXError ism330bxReadRegisters(const ISM330BX* const dev,
                                           const uint8_t               reg,
                                           void* const                 data,
                                           const uint16_t              length)
{
	return (dev->hal.readReg(reg, data, length) == 0) ? ISM330BX_ERROR_OK : ISM330BX_ERROR_COMM_FAIL;
}

static ISM330BXError ism330bxWriteRegisters(const ISM330BX* const dev,
                                            const uint8_t               reg,
                                            const void* const           data,
                                            const uint16_t              length)
{
	return (dev->hal.writeReg(reg, data, length) == 0) ? ISM330BX_ERROR_OK : ISM330BX_ERROR_COMM_FAIL;
}

static ISM330BXError ism330bxReadByte(const ISM330BX* const dev,
                                      const uint8_t               reg,
                                      uint8_t* const              value)
{
	return ism330bxReadRegisters(dev, reg, value, 1U);
}

static ISM330BXError ism330bxWriteByte(const ISM330BX* const dev,
                                       const uint8_t               reg,
                                       const uint8_t               value)
{
	return ism330bxWriteRegisters(dev, reg, &value, 1U);
}

static ISM330BXError ism330bxModifyByte(const ISM330BX* const dev,
                                        const uint8_t               reg,
                                        const uint8_t               mask,
                                        const uint8_t               value)
{
	uint8_t             current  = 0U;
	const ISM330BXError read_err = ism330bxReadByte(dev, reg, &current);
	if (read_err != ISM330BX_ERROR_OK)
		return read_err;
	const uint8_t updated = (uint8_t)((current & ~mask) | (value & mask));
	return ism330bxWriteByte(dev, reg, updated);
}

static ISM330BXError ism330bxApplyAccelConfig(ISM330BX* const            dev,
                                              const ISM330BXAccelConfig* const cfg)
{
	/* CTRL1 = (OP_MODE_XL << 4) | ODR_XL. ODR is the LOW nibble on BX
	 * (opposite of DHCx, which puts ODR in the high nibble). */
	const uint8_t ctrl1 = (uint8_t)(((uint8_t)cfg->odr << ISM330BX_CTRL1_ODR_XL_SHIFT) |
	                                ((uint8_t)cfg->performance_mode << ISM330BX_CTRL1_OP_MODE_XL_SHIFT));
	if (ism330bxWriteByte(dev, ISM330BX_REG_CTRL1, ctrl1) != ISM330BX_ERROR_OK)
		return ISM330BX_ERROR_COMM_FAIL;

	/* CTRL8 holds FS_XL in bits[1:0]; preserve LPF / dual-channel / QVAR-HPF
	 * fields via read-modify-write. */
	if (ism330bxModifyByte(dev, ISM330BX_REG_CTRL8,
	                       ISM330BX_CTRL8_FS_XL_MASK,
	                       (uint8_t)((uint8_t)cfg->full_scale << ISM330BX_CTRL8_FS_XL_SHIFT))
	    != ISM330BX_ERROR_OK)
		return ISM330BX_ERROR_COMM_FAIL;

	dev->accel_full_scale = cfg->full_scale;
	return ISM330BX_ERROR_OK;
}

static ISM330BXError ism330bxApplyGyroConfig(ISM330BX* const           dev,
                                             const ISM330BXGyroConfig* const cfg)
{
	/* CTRL2 = (OP_MODE_G << 4) | ODR_G. */
	const uint8_t ctrl2 = (uint8_t)(((uint8_t)cfg->odr << ISM330BX_CTRL2_ODR_G_SHIFT) |
	                                ((uint8_t)cfg->performance_mode << ISM330BX_CTRL2_OP_MODE_G_SHIFT));
	if (ism330bxWriteByte(dev, ISM330BX_REG_CTRL2, ctrl2) != ISM330BX_ERROR_OK)
		return ISM330BX_ERROR_COMM_FAIL;

	/* CTRL6 holds FS_G in bits[3:0]; preserve LPF1_G_BW in bits[6:4]. */
	if (ism330bxModifyByte(dev, ISM330BX_REG_CTRL6,
	                       ISM330BX_CTRL6_FS_G_MASK,
	                       (uint8_t)((uint8_t)cfg->full_scale << ISM330BX_CTRL6_FS_G_SHIFT))
	    != ISM330BX_ERROR_OK)
		return ISM330BX_ERROR_COMM_FAIL;

	dev->gyro_full_scale = cfg->full_scale;
	return ISM330BX_ERROR_OK;
}

static uint32_t ism330bxAccelSensNanoG(const ISM330BXAccelFullScale fs)
{
	switch (fs)
	{
	case ISM330BX_ACCEL_FS_2G:
		return ism330bx_accel_sens_nano_2g;
	case ISM330BX_ACCEL_FS_4G:
		return ism330bx_accel_sens_nano_4g;
	case ISM330BX_ACCEL_FS_8G:
		return ism330bx_accel_sens_nano_8g;
	default:
		return ism330bx_accel_sens_nano_2g;
	}
}

static uint32_t ism330bxGyroSensMicroDps(const ISM330BXGyroFullScale fs)
{
	switch (fs)
	{
	case ISM330BX_GYRO_FS_125_DPS:
		return ism330bx_gyro_sens_micro_125_dps;
	case ISM330BX_GYRO_FS_250_DPS:
		return ism330bx_gyro_sens_micro_250_dps;
	case ISM330BX_GYRO_FS_500_DPS:
		return ism330bx_gyro_sens_micro_500_dps;
	case ISM330BX_GYRO_FS_1000_DPS:
		return ism330bx_gyro_sens_micro_1000_dps;
	case ISM330BX_GYRO_FS_2000_DPS:
		return ism330bx_gyro_sens_micro_2000_dps;
	case ISM330BX_GYRO_FS_4000_DPS:
		return ism330bx_gyro_sens_micro_4000_dps;
	default:
		return ism330bx_gyro_sens_micro_250_dps;
	}
}
