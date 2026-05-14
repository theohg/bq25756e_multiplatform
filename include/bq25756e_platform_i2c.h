/**
 * @file bq25756e_platform_i2c.h
 * @brief Platform-agnostic I2C function declarations for the BQ25756E library.
 *
 * Provides a common I2C API used by the BQ25756E driver class. The
 * implementation is selected at compile time (Arduino Wire, STM32 HAL, or Pico SDK).
 *
 * @note Device addresses are always passed as 7-bit addresses.
 *       STM32 HAL shifts them internally; Arduino Wire and Pico SDK expect 7-bit.
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

#if defined(PLATFORM_ARDUINO)
    typedef TwoWire* bus_handle_t;
#elif defined(PLATFORM_STM32)
    typedef I2C_HandleTypeDef* bus_handle_t;
#elif defined(PLATFORM_RP2040)
    typedef i2c_inst_t* bus_handle_t;
#else
    typedef void* bus_handle_t;
#endif

/* ──────────────────── Common I2C API ──────────────────── */

#define BQ25756E_OK         0
#define BQ25756E_ERR_I2C    1
#define BQ25756E_ERR_HANDLE 2

/**
 * @brief Write an 8-bit value to a specific BQ25756E register.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to write to.
 * @param value 8-bit value to write.
 * @return true on success, false on failure.
 */
bool bq25756e_i2c_write_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t value);

/**
 * @brief Write a 16-bit value to a specific BQ25756E register (LSB first).
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to write to.
 * @param value 16-bit value to write.
 * @return true on success, false on failure.
 */
bool bq25756e_i2c_write_register16(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint16_t value);

/**
 * @brief Read an 8-bit value from a specific BQ25756E register.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to read from.
 * @param out Destination for the register value.
 * @return true on success, false on failure.
 */
bool bq25756e_i2c_read_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t* out);

/**
 * @brief Read a 16-bit value from a specific BQ25756E register (LSB first).
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address to read from.
 * @param out Destination for the register value.
 * @return true on success, false on failure.
 */
bool bq25756e_i2c_read_register16(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint16_t* out);

/**
 * @brief Set or clear specific bits in an 8-bit BQ25756E register.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address.
 * @param mask Bitmask indicating which bits to affect.
 * @param enable true to set the masked bits, false to clear them.
 * @return true on success, false on failure.
 */
bool bq25756e_i2c_modify_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, bool enable);

/**
 * @brief Write a new value into specific bits of an 8-bit BQ25756E register.
 * Other bits in the register are preserved.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address of the BQ25756E.
 * @param reg Register address.
 * @param mask Bitmask indicating which bits to modify.
 * @param new_value_for_bits New value for the masked bits.
 * @return true on success, false on failure.
 */
bool bq25756e_i2c_modify_register_bits(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value_for_bits);

#endif /* BQ25756E_PLATFORM_I2C_H */
