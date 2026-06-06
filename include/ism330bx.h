/**
 * @file ism330bx.h
 * @brief Public API for the ST ISM330BX 6-axis IMU driver (minimal scope).
 *
 * The ISM330BX is a 3-axis accelerometer + 3-axis gyroscope + temperature
 * sensor with on-chip FIFO, machine-learning core, finite state machine,
 * sensor-hub master, ISPU, and rich activity-detection interrupts. This
 * driver implements only the minimum surface required by the gravitometer
 * firmware: identity, reset, accel/gyro config, raw + milli-unit reads,
 * temperature, and STATUS_REG decode. FIFO/MLC/FSM/ISPU/sensor-hub/OIS/
 * self-test/wake-up/tap are all out of scope for this build.
 *
 * The chip is bus-agnostic from the driver's perspective: register access is
 * done via two callbacks (`readReg`, `writeReg`) which the application wraps
 * around either SPI (mode 0 or 3) or I²C.
 *
 * **Thread safety:** the driver is *not* thread-safe. Callers must serialize
 * access to a given ::ISM330BXDevice handle.
 *
 * API layering:
 *   1. Initialization / Lifecycle / Identity
 *   2. Accelerometer configuration & data
 *   3. Gyroscope configuration & data
 *   4. Temperature
 *   5. Status / data-ready
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef ISM330BX_H
#define ISM330BX_H

#include "ism330bx_registers.h"
#include "ism330bx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 1. Initialization / Lifecycle / Identity ─────────────────────────────── */

/**
 * @brief Initialize the driver: connect HAL, verify chip ID, apply config.
 *
 * Sequence:
 *   1. Validate args.
 *   2. Copy HAL into device handle.
 *   3. Read WHO_AM_I; reject if it doesn't match ::ISM330BX_WHO_AM_I_VALUE.
 *   4. Issue a software reset and wait for it to clear.
 *   5. Set CTRL3.IF_INC and BDU per config; configure INT pin polarity / drive.
 *   6. Apply accel + gyro config (cache full-scale for unit conversion).
 *
 * @param[in,out] dev     Device handle. Must not be NULL.
 * @param[in]     config  Initialization configuration. Must not be NULL.
 * @param[in]     hal     HAL function pointers. `readReg` and `writeReg` must be non-NULL.
 * @return ::ISM330BX_ERROR_OK on success.
 */
ISM330BXError ism330bxInit(ISM330BXDevice* const       dev,
                           const ISM330BXConfig* const config,
                           const ISM330BXHAL* const    hal);

/**
 * @brief Issue a software reset (CTRL3.SW_RESET) and poll until it clears.
 */
ISM330BXError ism330bxReset(ISM330BXDevice* const dev);

/**
 * @brief Read the WHO_AM_I register. Should be ::ISM330BX_WHO_AM_I_VALUE.
 *
 * @param[in]  dev      Initialized device handle.
 * @param[out] who_am_i Caller-allocated byte to fill in. Must not be NULL.
 */
ISM330BXError ism330bxGetWHOAMI(const ISM330BXDevice* const dev, uint8_t* const who_am_i);

/* ── 2. Accelerometer configuration & data ────────────────────────────────── */

/**
 * @brief Apply accel ODR + full-scale + op-mode in one shot.
 *
 * Writes CTRL1 (ODR + OP_MODE) and updates CTRL8 bits[1:0] (FS_XL) via
 * read-modify-write so other CTRL8 fields (HP/LPF2 cutoff, dual-channel,
 * QVAR HPF) are preserved.
 */
ISM330BXError ism330bxSetAccelConfig(ISM330BXDevice* const            dev,
                                     const ISM330BXAccelConfig* const config);

/**
 * @brief Read the accelerometer's last conversion (raw 16-bit signed counts).
 *
 * The chip block at 0x28..0x2D is in Z,Y,X order on the BX; this function
 * does a single auto-incremented bus read and re-orders into the standard
 * {x,y,z} struct for caller convenience.
 */
ISM330BXError ism330bxReadAccelRaw(const ISM330BXDevice* const dev,
                                   ISM330BXAxesRaw* const      out);

/**
 * @brief Read the accelerometer in milli-g, applying the cached full-scale.
 */
ISM330BXError ism330bxReadAccelMilliG(const ISM330BXDevice* const dev,
                                      ISM330BXAxesMilli* const    out);

/* ── 3. Gyroscope configuration & data ────────────────────────────────────── */

/**
 * @brief Apply gyro ODR + full-scale + op-mode in one shot.
 *
 * Writes CTRL2 (ODR + OP_MODE) and updates CTRL6 bits[3:0] (FS_G) via
 * read-modify-write to preserve LPF1 bandwidth in the same register.
 */
ISM330BXError ism330bxSetGyroConfig(ISM330BXDevice* const           dev,
                                    const ISM330BXGyroConfig* const config);

/**
 * @brief Read the gyroscope's last conversion (raw 16-bit signed counts).
 */
ISM330BXError ism330bxReadGyroRaw(const ISM330BXDevice* const dev,
                                  ISM330BXAxesRaw* const      out);

/**
 * @brief Read the gyroscope in milli-degrees per second, using the cached FS.
 */
ISM330BXError ism330bxReadGyroMilliDPS(const ISM330BXDevice* const dev,
                                       ISM330BXAxesMilli* const    out);

/* ── 4. Temperature ───────────────────────────────────────────────────────── */

/**
 * @brief Read the on-chip temperature sensor in raw 16-bit signed counts.
 *
 * Conversion (datasheet): degC = raw / 256 + 25.
 */
ISM330BXError ism330bxReadTempRaw(const ISM330BXDevice* const dev, int16_t* const raw);

/**
 * @brief Read the on-chip temperature in milli-degrees Celsius.
 */
ISM330BXError ism330bxReadTempMilliCelsius(const ISM330BXDevice* const dev,
                                           int32_t* const              milli_celsius);

/* ── 5. Status / data-ready ───────────────────────────────────────────────── */

/**
 * @brief Read STATUS_REG and decode XLDA / GDA / TDA bits.
 */
ISM330BXError ism330bxGetStatus(const ISM330BXDevice* const dev,
                                ISM330BXStatus* const       status);

#ifdef __cplusplus
}
#endif

#endif /* ISM330BX_H */
