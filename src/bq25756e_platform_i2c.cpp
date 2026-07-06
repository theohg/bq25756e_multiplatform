/**
 * @file bq25756e_platform_i2c.cpp
 * @brief Platform-specific I2C implementations for the BQ25756E library.
 *
 * Arduino/ESP32 path uses Wire; STM32 path uses HAL_I2C_Mem_Read/Write.
 *
 * @copyright Copyright (c) 2026 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#include "bq25756e_platform_i2c.h"

#ifdef BQ25756E_PLATFORM_RP2040
// Per-transaction I2C timeout for the RP2040 path. A blocking transfer can hang
// forever if a slave clock-stretches indefinitely or the bus locks up; bounding
// it lets the failure propagate through the existing bool returns instead of
// freezing the caller's loop. 5 ms is generous at 400 kHz (each byte ~22.5 us).
#define BQ25756E_RP2040_I2C_TIMEOUT_US 5000u
#endif

#ifdef BQ25756E_PLATFORM_STM32
// Per-transaction I2C timeout (ms) for the STM32 HAL path. HAL_MAX_DELAY blocks
// forever on the same clock-stretch/bus-lockup failures the RP2040 bound above
// guards against; 5 ms mirrors that bound so the failure propagates instead.
#define BQ25756E_STM32_I2C_TIMEOUT_MS 5u
#endif

/**
 * @brief Write an 8-bit value to a specific BQ25756E register.
 * @param bus The bus handle to use.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to write to.
 * @param value The 8-bit value to write.
 */
bool bq25756e_i2c_write_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t value) {
    if (bus == NULL) {
        return false;
    }

#ifdef BQ25756E_PLATFORM_ARDUINO
    bus->beginTransmission(device_address);
    bus->write(reg);
    bus->write(value);
    return bus->endTransmission() == 0;
#elif defined(BQ25756E_PLATFORM_STM32)
    return HAL_I2C_Mem_Write(bus, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, BQ25756E_STM32_I2C_TIMEOUT_MS) == HAL_OK;
#elif defined(BQ25756E_PLATFORM_RP2040)
    uint8_t data[2] = { reg, value };
    return i2c_write_timeout_us(bus, device_address, data, 2, false, BQ25756E_RP2040_I2C_TIMEOUT_US) == 2;
#else
    (void)device_address;
    (void)reg;
    (void)value;
    return false;
#endif
}

/**
 * @brief Write a 16-bit value to a specific BQ25756 register.
 * BQ25756 expects LSB first, then MSB for the data payload.
 * @param device_address The I2C address of the BQ25756.
 * @param reg The register address to write to.
 * @param value The 16-bit value to write.
 */
bool bq25756e_i2c_write_register16(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint16_t value) {
    if (bus == NULL) {
        return false;
    }

#ifdef BQ25756E_PLATFORM_ARDUINO
    bus->beginTransmission(device_address);
    bus->write(reg);
    bus->write(value & 0xFF);
    bus->write((value >> 8) & 0xFF);
    return bus->endTransmission() == 0;
#elif defined(BQ25756E_PLATFORM_STM32)
    uint8_t data_payload[2];
    data_payload[0] = value & 0xFF;
    data_payload[1] = (value >> 8) & 0xFF;
    return HAL_I2C_Mem_Write(bus, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, data_payload, 2, BQ25756E_STM32_I2C_TIMEOUT_MS) == HAL_OK;
#elif defined(BQ25756E_PLATFORM_RP2040)
    uint8_t data_payload[3];
    data_payload[0] = reg;
    data_payload[1] = value & 0xFF;
    data_payload[2] = (value >> 8) & 0xFF;
    return i2c_write_timeout_us(bus, device_address, data_payload, 3, false, BQ25756E_RP2040_I2C_TIMEOUT_US) == 3;
#else
    (void)device_address;
    (void)reg;
    (void)value;
    return false;
#endif
}

/**
 * @brief Read an 8-bit value from a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to read from.
 * @return The 8-bit value read from the register, or 0 on error.
 */
bool bq25756e_i2c_read_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t* out) {
    if (bus == NULL || out == NULL) {
        return false;
    }

#ifdef BQ25756E_PLATFORM_ARDUINO
    bus->beginTransmission(device_address);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) {
        return false;
    }

    if (bus->requestFrom(device_address, static_cast<uint8_t>(1)) != 1) {
        return false;
    }

    if (!bus->available()) {
        return false;
    }
    *out = bus->read();
    return true;
#elif defined(BQ25756E_PLATFORM_STM32)
    return HAL_I2C_Mem_Read(bus, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, out, 1, BQ25756E_STM32_I2C_TIMEOUT_MS) == HAL_OK;
#elif defined(BQ25756E_PLATFORM_RP2040)
    if (i2c_write_timeout_us(bus, device_address, &reg, 1, true, BQ25756E_RP2040_I2C_TIMEOUT_US) != 1) {
        return false;
    }
    return i2c_read_timeout_us(bus, device_address, out, 1, false, BQ25756E_RP2040_I2C_TIMEOUT_US) == 1;
#else
    (void)device_address;
    (void)reg;
    return false;
#endif
}

/**
 * @brief Read a 16-bit value from a specific BQ25756E register.
 * BQ25756 returns LSB first, then MSB.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to read from.
 * @return The 16-bit value read from the register, or 0 on error.
 */
bool bq25756e_i2c_read_register16(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint16_t* out) {
    if (bus == NULL || out == NULL) {
        return false;
    }

#ifdef BQ25756E_PLATFORM_ARDUINO
    bus->beginTransmission(device_address);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) {
        return false;
    }

    if (bus->requestFrom(device_address, static_cast<uint8_t>(2)) != 2) {
        return false;
    }

    if (bus->available() < 2) {
        return false;
    }
    uint8_t lsb = bus->read();
    uint8_t msb = bus->read();
    *out = (static_cast<uint16_t>(msb) << 8) | lsb;
    return true;
#elif defined(BQ25756E_PLATFORM_STM32)
    uint8_t data_buffer[2];
    if (HAL_I2C_Mem_Read(bus, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, data_buffer, 2, BQ25756E_STM32_I2C_TIMEOUT_MS) == HAL_OK) {
        *out = (static_cast<uint16_t>(data_buffer[1]) << 8) | data_buffer[0];
        return true;
    }
    return false;
#elif defined(BQ25756E_PLATFORM_RP2040)
    uint8_t data_buffer[2] = {0, 0};
    if (i2c_write_timeout_us(bus, device_address, &reg, 1, true, BQ25756E_RP2040_I2C_TIMEOUT_US) != 1) {
        return false;
    }
    if (i2c_read_timeout_us(bus, device_address, data_buffer, 2, false, BQ25756E_RP2040_I2C_TIMEOUT_US) != 2) {
        return false;
    }
    *out = (static_cast<uint16_t>(data_buffer[1]) << 8) | data_buffer[0];
    return true;
#else
    (void)device_address;
    (void)reg;
    return false;
#endif
}

/**
 * @brief Modify specific bits in a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address.
 * @param mask The bitmask to apply.
 * @param enable True to set bits, false to clear bits.
 */
bool bq25756e_i2c_modify_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, bool enable) {
    uint8_t current_value = 0;
    if (!bq25756e_i2c_read_register(bus, device_address, reg, &current_value)) {
        return false;
    }
    
    uint8_t new_value;
    if (enable) {
        new_value = current_value | mask;  // Set bits
    } else {
        new_value = current_value & ~mask; // Clear bits
    }
    // Only write if the value has changed
    if (new_value != current_value) {
        return bq25756e_i2c_write_register(bus, device_address, reg, new_value);
    }
    return true;
}

/**
 * @brief Modify specific bits in a specific BQ25756E register using a new value for those bits.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address.
 * @param mask The bitmask indicating which bits to modify.
 * @param new_value_for_bits The new value for the bits defined by the mask. Other bits are preserved.
 */
bool bq25756e_i2c_modify_register_bits(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value_for_bits) {
    uint8_t current_value = 0;
    if (!bq25756e_i2c_read_register(bus, device_address, reg, &current_value)) {
        return false;
    }
    
    // Clear the bits defined by the mask in the current value, then OR with the new value (also masked)
    uint8_t new_value = (current_value & ~mask) | (new_value_for_bits & mask);
    
    if (new_value != current_value) {
        return bq25756e_i2c_write_register(bus, device_address, reg, new_value);
    }
    return true;
}