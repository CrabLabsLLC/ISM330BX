/**
 * @file ism330bx_types.h
 * @brief Types, enums, HAL, and config structs for the ISM330BX driver.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef ISM330BX_TYPES_H
#define ISM330BX_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Error codes ──────────────────────────────────────────────────────────── */

typedef enum
{
	ISM330BX_ERROR_OK            = 0,  ///< Success
	ISM330BX_ERROR_INVALID_PARAM = -1, ///< NULL pointer or out-of-range argument
	ISM330BX_ERROR_COMM_FAIL     = -2, ///< Bus transaction reported failure
	ISM330BX_ERROR_NOT_INIT      = -3, ///< Driver not initialized
	ISM330BX_ERROR_BAD_ID        = -4, ///< WHO_AM_I returned an unexpected value
	ISM330BX_ERROR_NOT_SUPPORTED = -5, ///< Operation requires a HAL pin not provided
	ISM330BX_ERROR_TIMEOUT       = -6, ///< Polling loop timed out
} ISM330BXError;

/* ── Accelerometer output data rate (CTRL1.ODR_XL[3:0]) ───────────────────── */
/* ISM330BX has its own native frequency ladder; codes are NOT compatible with DHCx. */

typedef enum
{
	ISM330BX_ACCEL_ODR_OFF      = 0x0U, ///< Power-down
	ISM330BX_ACCEL_ODR_1_875_HZ = 0x1U, ///< 1.875 Hz (XL only)
	ISM330BX_ACCEL_ODR_7_5_HZ   = 0x2U,
	ISM330BX_ACCEL_ODR_15_HZ    = 0x3U,
	ISM330BX_ACCEL_ODR_30_HZ    = 0x4U,
	ISM330BX_ACCEL_ODR_60_HZ    = 0x5U,
	ISM330BX_ACCEL_ODR_120_HZ   = 0x6U,
	ISM330BX_ACCEL_ODR_240_HZ   = 0x7U,
	ISM330BX_ACCEL_ODR_480_HZ   = 0x8U,
	ISM330BX_ACCEL_ODR_960_HZ   = 0x9U,
	ISM330BX_ACCEL_ODR_1920_HZ  = 0xAU,
	ISM330BX_ACCEL_ODR_3840_HZ  = 0xBU,
} ISM330BXAccelODR;

/* ── Accelerometer full-scale (CTRL8.FS_XL[1:0]) ──────────────────────────── */
/* Note: BX has no ±16 g range. */

typedef enum
{
	ISM330BX_ACCEL_FS_2G = 0x0U,
	ISM330BX_ACCEL_FS_4G = 0x1U,
	ISM330BX_ACCEL_FS_8G = 0x2U,
} ISM330BXAccelFullScale;

/* ── Gyroscope output data rate (CTRL2.ODR_G[3:0]) ────────────────────────── */
/* No 1.875 Hz code on the gyro side. */

typedef enum
{
	ISM330BX_GYRO_ODR_OFF     = 0x0U, ///< Power-down
	ISM330BX_GYRO_ODR_7_5_HZ  = 0x2U,
	ISM330BX_GYRO_ODR_15_HZ   = 0x3U,
	ISM330BX_GYRO_ODR_30_HZ   = 0x4U,
	ISM330BX_GYRO_ODR_60_HZ   = 0x5U,
	ISM330BX_GYRO_ODR_120_HZ  = 0x6U,
	ISM330BX_GYRO_ODR_240_HZ  = 0x7U,
	ISM330BX_GYRO_ODR_480_HZ  = 0x8U,
	ISM330BX_GYRO_ODR_960_HZ  = 0x9U,
	ISM330BX_GYRO_ODR_1920_HZ = 0xAU,
	ISM330BX_GYRO_ODR_3840_HZ = 0xBU,
} ISM330BXGyroODR;

/* ── Gyroscope full-scale (CTRL6.FS_G[3:0]) ───────────────────────────────── */
/* Note: ±4000 dps uses code 0xC, not a sequential value. */

typedef enum
{
	ISM330BX_GYRO_FS_125_DPS  = 0x0U,
	ISM330BX_GYRO_FS_250_DPS  = 0x1U,
	ISM330BX_GYRO_FS_500_DPS  = 0x2U,
	ISM330BX_GYRO_FS_1000_DPS = 0x3U,
	ISM330BX_GYRO_FS_2000_DPS = 0x4U,
	ISM330BX_GYRO_FS_4000_DPS = 0xCU,
} ISM330BXGyroFullScale;

/* ── Operating mode (CTRL1.OP_MODE_XL[6:4] / CTRL2.OP_MODE_G[6:4]) ────────── */
/* Default 0 = high-performance for both axes; specific low-power codes vary
 * per datasheet. Callers that want non-default modes can pass the raw 3-bit
 * code directly and it'll be programmed verbatim. */

typedef enum
{
	ISM330BX_PERF_MODE_HIGH_PERFORMANCE = 0x0U, ///< Default
	ISM330BX_PERF_MODE_LOW_POWER_1      = 0x4U,
	ISM330BX_PERF_MODE_LOW_POWER_2      = 0x5U,
	ISM330BX_PERF_MODE_LOW_POWER_3      = 0x6U,
} ISM330BXPerformanceMode;

/* ── Interrupt pin polarity & drive type ──────────────────────────────────── */

typedef enum
{
	ISM330BX_INT_POL_ACTIVE_HIGH = 0U,
	ISM330BX_INT_POL_ACTIVE_LOW  = 1U,
} ISM330BXIntPolarity;

typedef enum
{
	ISM330BX_INT_DRIVE_PUSH_PULL  = 0U,
	ISM330BX_INT_DRIVE_OPEN_DRAIN = 1U,
} ISM330BXIntDrive;

/* ── 3-axis sample (raw int16_t signed counts) ────────────────────────────── */

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} ISM330BXAxesRaw;

/* ── 3-axis sample, milli-units (g for accel, dps for gyro) ───────────────── */

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
} ISM330BXAxesMilli;

/* ── Status (decoded STATUS_REG) ──────────────────────────────────────────── */

typedef struct
{
	bool accel_data_ready; ///< STATUS_REG.XLDA
	bool gyro_data_ready;  ///< STATUS_REG.GDA
	bool temp_data_ready;  ///< STATUS_REG.TDA
} ISM330BXStatus;

/* ── Accelerometer config bundle ──────────────────────────────────────────── */

typedef struct
{
	ISM330BXAccelODR        odr;
	ISM330BXAccelFullScale  full_scale;
	ISM330BXPerformanceMode performance_mode;
} ISM330BXAccelConfig;

/* ── Gyroscope config bundle ──────────────────────────────────────────────── */

typedef struct
{
	ISM330BXGyroODR         odr;
	ISM330BXGyroFullScale   full_scale;
	ISM330BXPerformanceMode performance_mode;
} ISM330BXGyroConfig;

/* ── HAL ──────────────────────────────────────────────────────────────────── */

/**
 * @brief Platform-provided register-access primitives.
 *
 * The driver is bus-agnostic: the platform code wraps either SPI or I²C
 * transactions to satisfy `readReg` and `writeReg`. The chip auto-increments
 * the register pointer for multi-byte transfers when CTRL3.IF_INC = 1
 * (which the driver sets at init), so the platform just needs a faithful
 * "address + N data bytes" implementation on whichever bus.
 */
typedef struct
{
	/**
	 * @brief Read `length` bytes starting from register `reg`. Returns 0 on success.
	 *
	 * For SPI: assert ~CS, send `reg | 0x80` (read bit), clock in N bytes, release ~CS.
	 * For I²C: write the device address + reg, restart, read N bytes.
	 */
	int (*readReg)(uint8_t reg, void* const data, uint16_t length);

	/**
	 * @brief Write `length` bytes starting at register `reg`. Returns 0 on success.
	 */
	int (*writeReg)(uint8_t reg, const void* const data, uint16_t length);

	/** @brief Optional: spin/sleep at least `delay_ms` milliseconds. */
	void (*delayMs)(uint32_t delay_ms);

	/** @brief Optional: drive the active-low ~RST pin. NULL = no hardware reset support. */
	void (*resetSet)(bool level);

	/** @brief Optional: read the INT1 pin level. NULL = polled-only. */
	bool (*int1Get)(void);

	/** @brief Optional: read the INT2 pin level. */
	bool (*int2Get)(void);
} ISM330BXHAL;

/* ── Init configuration ───────────────────────────────────────────────────── */

typedef struct
{
	ISM330BXAccelConfig accel;
	ISM330BXGyroConfig  gyro;
	bool                block_data_update; ///< CTRL3.BDU
	ISM330BXIntPolarity interrupt_polarity;
	ISM330BXIntDrive    interrupt_drive;
} ISM330BXConfig;

/* ── Device handle ────────────────────────────────────────────────────────── */

typedef struct
{
	ISM330BXHAL            hal;
	ISM330BXAccelFullScale accel_full_scale; ///< Cached for unit conversion
	ISM330BXGyroFullScale  gyro_full_scale;  ///< Cached for unit conversion
	bool                   is_initialized;
} ISM330BXDevice;

#ifdef __cplusplus
}
#endif

#endif /* ISM330BX_TYPES_H */
