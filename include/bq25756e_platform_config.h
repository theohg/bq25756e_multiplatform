/**
 * @file bq25756e_platform_config.h
 * @brief Compile-time platform detection for the BQ25756E library.
 *
 * Automatically detects the target platform (Arduino/ESP32 or STM32) via
 * preprocessor macros and includes the appropriate system headers.
 *
 * @copyright Copyright (c) 2026 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#ifndef BQ25756E_PLATFORM_CONFIG_H
#define BQ25756E_PLATFORM_CONFIG_H

#if defined(ESP32) || defined(ESP_PLATFORM) || defined(ARDUINO)
    #define BQ25756E_PLATFORM_ARDUINO
    #ifndef PLATFORM_ARDUINO
        #define PLATFORM_ARDUINO
    #endif
    #include <Arduino.h>

#elif defined(USE_HAL_DRIVER) || defined(USE_STM32_HAL_DRIVER) || defined(BQ25756E_PLATFORM_STM32)
    #ifndef BQ25756E_PLATFORM_STM32
        #define BQ25756E_PLATFORM_STM32
    #endif
    #ifndef PLATFORM_STM32
        #define PLATFORM_STM32
    #endif
    #include <stdio.h>
    #include <math.h>
    #include <string.h>

    /* Auto-detect STM32 family HAL header, or override via build flag:
       -DBQ25756E_STM32_HAL_HEADER='"stm32f4xx_hal.h"'                    */
    #if defined(BQ25756E_STM32_HAL_HEADER)
        #include BQ25756E_STM32_HAL_HEADER
    #elif defined(STM32C0xx)
        #include "stm32c0xx_hal.h"
    #elif defined(STM32F0xx)
        #include "stm32f0xx_hal.h"
    #elif defined(STM32F1xx)
        #include "stm32f1xx_hal.h"
    #elif defined(STM32F2xx)
        #include "stm32f2xx_hal.h"
    #elif defined(STM32F3xx)
        #include "stm32f3xx_hal.h"
    #elif defined(STM32F4xx)
        #include "stm32f4xx_hal.h"
    #elif defined(STM32F7xx)
        #include "stm32f7xx_hal.h"
    #elif defined(STM32G0xx)
        #include "stm32g0xx_hal.h"
    #elif defined(STM32G4xx)
        #include "stm32g4xx_hal.h"
    #elif defined(STM32H5xx)
        #include "stm32h5xx_hal.h"
    #elif defined(STM32H7xx)
        #include "stm32h7xx_hal.h"
    #elif defined(STM32L0xx)
        #include "stm32l0xx_hal.h"
    #elif defined(STM32L1xx)
        #include "stm32l1xx_hal.h"
    #elif defined(STM32L4xx)
        #include "stm32l4xx_hal.h"
    #elif defined(STM32L5xx)
        #include "stm32l5xx_hal.h"
    #elif defined(STM32U0xx)
        #include "stm32u0xx_hal.h"
    #elif defined(STM32U5xx)
        #include "stm32u5xx_hal.h"
    #elif defined(STM32WBxx) || defined(STM32WB5Mxx) || defined(STM32WB55xx)
        #include "stm32wbxx_hal.h"
    #elif defined(STM32WBAxx)
        #include "stm32wbaxx_hal.h"
    #elif defined(STM32WLxx)
        #include "stm32wlxx_hal.h"
    #else
        #error "STM32 family not detected. Define BQ25756E_STM32_HAL_HEADER in build flags."
    #endif

#elif defined(PICO_BOARD) || defined(PICO_RP2040) || defined(PICO_SDK_VERSION_MAJOR)
    #define BQ25756E_PLATFORM_RP2040
    #ifndef PLATFORM_RP2040
        #define PLATFORM_RP2040
    #endif
    #include <stdio.h>
    #include <math.h>
    #include <string.h>
    #include "hardware/i2c.h"

#elif defined(BQ25756E_PLATFORM_NATIVE) || defined(BQ25756E_PLATFORM_HOST)
    /* Native/host build (x86/ARM desktop) for unit testing. No MCU peripherals:
       the I2C implementation is not linked, only the register logic and data
       types are exercised. bus_handle_t falls back to void* in platform_i2c.h. */
    #ifndef BQ25756E_PLATFORM_NATIVE
        #define BQ25756E_PLATFORM_NATIVE
    #endif
    #ifndef PLATFORM_NATIVE
        #define PLATFORM_NATIVE
    #endif
    #include <stdio.h>
    #include <math.h>
    #include <string.h>

#else
    #error "Unsupported platform. Define BQ25756E_PLATFORM_ARDUINO, BQ25756E_PLATFORM_STM32, BQ25756E_PLATFORM_RP2040, or BQ25756E_PLATFORM_NATIVE manually."
#endif

#endif /* BQ25756E_PLATFORM_CONFIG_H */
