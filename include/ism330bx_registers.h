/**
 * @file ism330bx_registers.h
 * @brief ISM330BX register addresses, bit-field shifts/masks, and command constants.
 *
 * Register addresses come from the ST datasheet cross-referenced against the
 * official `ism330bx-pid` driver (https://github.com/STMicroelectronics/ism330bx-pid).
 * Field shifts and masks are derived from the per-register bit layouts; named-value
 * enumerations live in `ism330bx_types.h` so callers never see raw bit patterns.
 *
 * NOTE: the BX register map differs from the DHCx in several places. In particular:
 *   - FS_XL lives in CTRL8[1:0] (not CTRL1) on BX.
 *   - FS_G lives in CTRL6[3:0] (not CTRL2) on BX.
 *   - CTRL1.ODR_XL is in bits [3:0] (not [7:4] like DHCx); OP_MODE_XL is in [6:4].
 *   - CTRL2.ODR_G  is in bits [3:0]; OP_MODE_G  is in [6:4].
 *   - Accel output block at 0x28..0x2D is in Z,Y,X order on BX (not X,Y,Z).
 *   - FIFO_STATUS1/2 are at 0x1B/0x1C (not 0x3A/0x3B). FIFO is out of scope here.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef ISM330BX_REGISTERS_H
#define ISM330BX_REGISTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Identity ─────────────────────────────────────────────────────────────── */

#define ISM330BX_WHO_AM_I_VALUE 0x71U ///< Expected ID returned by ::ISM330BX_REG_WHO_AM_I

/* ── User-bank register addresses ─────────────────────────────────────────── */

#define ISM330BX_REG_FUNC_CFG_ACCESS 0x01U ///< Bank-select gate
#define ISM330BX_REG_PIN_CTRL        0x02U
#define ISM330BX_REG_IF_CFG          0x03U
#define ISM330BX_REG_FIFO_CTRL1      0x07U
#define ISM330BX_REG_FIFO_CTRL2      0x08U
#define ISM330BX_REG_FIFO_CTRL3      0x09U
#define ISM330BX_REG_FIFO_CTRL4      0x0AU
#define ISM330BX_REG_INT1_CTRL       0x0DU
#define ISM330BX_REG_INT2_CTRL       0x0EU
#define ISM330BX_REG_WHO_AM_I        0x0FU
#define ISM330BX_REG_CTRL1           0x10U ///< [3:0]=ODR_XL, [6:4]=OP_MODE_XL
#define ISM330BX_REG_CTRL2           0x11U ///< [3:0]=ODR_G,  [6:4]=OP_MODE_G
#define ISM330BX_REG_CTRL3           0x12U
#define ISM330BX_REG_CTRL4           0x13U
#define ISM330BX_REG_CTRL5           0x14U
#define ISM330BX_REG_CTRL6           0x15U ///< [3:0]=FS_G, [6:4]=LPF1_G_BW
#define ISM330BX_REG_CTRL7           0x16U
#define ISM330BX_REG_CTRL8           0x17U ///< [1:0]=FS_XL, [7:5]=HP_LPF2_XL_BW
#define ISM330BX_REG_CTRL9           0x18U
#define ISM330BX_REG_CTRL10          0x19U
#define ISM330BX_REG_FIFO_STATUS1    0x1BU ///< (FIFO out of scope for this driver)
#define ISM330BX_REG_FIFO_STATUS2    0x1CU
#define ISM330BX_REG_ALL_INT_SRC     0x1DU
#define ISM330BX_REG_STATUS_REG      0x1EU
#define ISM330BX_REG_OUT_TEMP_L      0x20U
#define ISM330BX_REG_OUT_TEMP_H      0x21U
#define ISM330BX_REG_OUTX_L_G        0x22U ///< Gyro output: X,Y,Z in 0x22..0x27
#define ISM330BX_REG_OUTX_H_G        0x23U
#define ISM330BX_REG_OUTY_L_G        0x24U
#define ISM330BX_REG_OUTY_H_G        0x25U
#define ISM330BX_REG_OUTZ_L_G        0x26U
#define ISM330BX_REG_OUTZ_H_G        0x27U
#define ISM330BX_REG_OUTZ_L_A        0x28U ///< Accel output: Z,Y,X in 0x28..0x2D (BX-specific!)
#define ISM330BX_REG_OUTZ_H_A        0x29U
#define ISM330BX_REG_OUTY_L_A        0x2AU
#define ISM330BX_REG_OUTY_H_A        0x2BU
#define ISM330BX_REG_OUTX_L_A        0x2CU
#define ISM330BX_REG_OUTX_H_A        0x2DU
#define ISM330BX_REG_TIMESTAMP0      0x40U
#define ISM330BX_REG_TIMESTAMP1      0x41U
#define ISM330BX_REG_TIMESTAMP2      0x42U
#define ISM330BX_REG_TIMESTAMP3      0x43U

/* ── CTRL1 (accel control) bit fields ─────────────────────────────────────── */

#define ISM330BX_CTRL1_ODR_XL_SHIFT     0U
#define ISM330BX_CTRL1_ODR_XL_MASK      (0xFU << ISM330BX_CTRL1_ODR_XL_SHIFT)
#define ISM330BX_CTRL1_OP_MODE_XL_SHIFT 4U
#define ISM330BX_CTRL1_OP_MODE_XL_MASK  (0x7U << ISM330BX_CTRL1_OP_MODE_XL_SHIFT)

/* ── CTRL2 (gyro control) bit fields ──────────────────────────────────────── */

#define ISM330BX_CTRL2_ODR_G_SHIFT     0U
#define ISM330BX_CTRL2_ODR_G_MASK      (0xFU << ISM330BX_CTRL2_ODR_G_SHIFT)
#define ISM330BX_CTRL2_OP_MODE_G_SHIFT 4U
#define ISM330BX_CTRL2_OP_MODE_G_MASK  (0x7U << ISM330BX_CTRL2_OP_MODE_G_SHIFT)

/* ── CTRL3 (master control) bit fields ────────────────────────────────────── */

#define ISM330BX_CTRL3_SW_RESET_BIT (1U << 0)
#define ISM330BX_CTRL3_IF_INC_BIT   (1U << 2) ///< Auto-increment register address
#define ISM330BX_CTRL3_BDU_BIT      (1U << 6) ///< Block data update
#define ISM330BX_CTRL3_BOOT_BIT     (1U << 7) ///< Reload calibration (auto-clears)

/* ── CTRL6 (gyro filter / FS) bit fields ──────────────────────────────────── */

#define ISM330BX_CTRL6_FS_G_SHIFT     0U
#define ISM330BX_CTRL6_FS_G_MASK      (0xFU << ISM330BX_CTRL6_FS_G_SHIFT)
#define ISM330BX_CTRL6_LPF1_G_BW_SHIFT 4U
#define ISM330BX_CTRL6_LPF1_G_BW_MASK  (0x7U << ISM330BX_CTRL6_LPF1_G_BW_SHIFT)

/* ── CTRL8 (accel filter / FS) bit fields ─────────────────────────────────── */

#define ISM330BX_CTRL8_FS_XL_SHIFT         0U
#define ISM330BX_CTRL8_FS_XL_MASK          (0x3U << ISM330BX_CTRL8_FS_XL_SHIFT)
#define ISM330BX_CTRL8_XL_DUALC_EN_BIT     (1U << 3)
#define ISM330BX_CTRL8_AH_QVAR_HPF_BIT     (1U << 4)
#define ISM330BX_CTRL8_HP_LPF2_XL_BW_SHIFT 5U
#define ISM330BX_CTRL8_HP_LPF2_XL_BW_MASK  (0x7U << ISM330BX_CTRL8_HP_LPF2_XL_BW_SHIFT)

/* ── IF_CFG bit fields (interrupt pad polarity / drive) ───────────────────── */

#define ISM330BX_IF_CFG_PP_OD_BIT     (1U << 3) ///< 0 = push-pull, 1 = open-drain
#define ISM330BX_IF_CFG_H_LACTIVE_BIT (1U << 4) ///< 0 = INT active-high, 1 = active-low

/* ── STATUS_REG bit fields ────────────────────────────────────────────────── */

#define ISM330BX_STATUS_XLDA_BIT (1U << 0) ///< Accel data ready
#define ISM330BX_STATUS_GDA_BIT  (1U << 1) ///< Gyro data ready
#define ISM330BX_STATUS_TDA_BIT  (1U << 2) ///< Temp data ready

#ifdef __cplusplus
}
#endif

#endif /* ISM330BX_REGISTERS_H */
