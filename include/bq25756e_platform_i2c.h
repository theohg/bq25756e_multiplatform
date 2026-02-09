/**
 * @file bq25756e_platform_i2c.h
 * @brief Platform-agnostic I2C function declarations for the BQ25756E library.
 *
 * Provides a common I2C API used by the BQ25756E driver class. The
 * implementation is selected at compile time (Arduino Wire or STM32 HAL).
 *
 * @copyright Copyright (c) 2026 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#ifndef BQ25756E_PLATFORM_I2C_H
#define BQ25756E_PLATFORM_I2C_H

#include "bq25756e_platform_config.h"

#ifdef BQ25756E_PLATFORM_ARDUINO
    #include <Wire.h>
#endif

#ifdef BQ25756E_PLATFORM_STM32
    /**
     * @brief Sets the I2C handle for STM32 platforms.
     * This function must be called once during initialization (e.g., in main.c after MX_I2Cx_Init())
     * before any other I2C operations from this library are used.
     * @param hi2c Pointer to the I2C_HandleTypeDef structure configured for the BQ25756E.
     */
    void bq25756e_i2c_set_handle(I2C_HandleTypeDef* hi2c);
#endif

/* ──────────────────── Common I2C API ──────────────────── */

/**
 * @brief Write an 8-bit value to a specific BQ25756E register.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to write to.
 * @param value 8-bit value to write.
 */
void bq25756e_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value);

/**
 * @brief Write a 16-bit value to a specific BQ25756E register (LSB first).
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to write to.
 * @param value 16-bit value to write.
 */
void bq25756e_i2c_write_register16(uint8_t device_address, uint8_t reg, uint16_t value);

/**
 * @brief Read an 8-bit value from a specific BQ25756E register.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to read from.
 * @return 8-bit value read, or 0 on error.
 */
uint8_t bq25756e_i2c_read_register(uint8_t device_address, uint8_t reg);

/**
 * @brief Read a 16-bit value from a specific BQ25756E register (LSB first).
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to read from.
 * @return 16-bit value read, or 0 on error.
 */
uint16_t bq25756e_i2c_read_register16(uint8_t device_address, uint8_t reg);

/**
 * @brief Set or clear specific bits in an 8-bit BQ25756E register.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address.
 * @param mask Bitmask indicating which bits to affect.
 * @param enable true to set the masked bits, false to clear them.
 */
void bq25756e_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, bool enable);

/**
 * @brief Write a new value into specific bits of an 8-bit BQ25756E register.
 * Other bits in the register are preserved.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address.
 * @param mask Bitmask indicating which bits to modify.
 * @param new_value_for_bits New value for the masked bits.
 */
void bq25756e_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value_for_bits);

#endif /* BQ25756E_PLATFORM_I2C_H */
