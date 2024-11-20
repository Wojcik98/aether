/**
 *
 * Copyright (c) 2023 STMicroelectronics.
 * Copyright (c) 2024 Michał Wójcik.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef VL53L4CD_I2C_HPP_
#define VL53L4CD_I2C_HPP_

#include <stdint.h>
#include <string.h>

/**
 * @brief Error instance.
 */
using VL53L4CD_Error = uint8_t;

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */
// #define VL53L4CD_I2C_FAST_MODE_PLUS

class VL53L4CD_I2C {
public:
    static constexpr uint8_t VL53L4CD_I2C_ADDR_DEFAULT = 0x29;

    virtual ~VL53L4CD_I2C() = default;

    /**
     * @brief Read 32 bits through I2C.
     */
    virtual uint8_t read_32bit(uint16_t register_address, uint32_t *value) = 0;

    /**
     * @brief Write 32 bits through I2C.
     */
    virtual uint8_t write_32bit(uint16_t register_address, uint32_t value) = 0;

    /**
     * @brief Read 16 bits through I2C.
     */
    virtual uint8_t read_16bit(uint16_t register_address, uint16_t *value) = 0;

    /**
     * @brief Write 16 bits through I2C.
     */
    virtual uint8_t write_16bit(uint16_t register_address, uint16_t value) = 0;

    /**
     * @brief Read 8 bits through I2C.
     */
    virtual uint8_t read_8bit(uint16_t register_address, uint8_t *value) = 0;

    /**
     * @brief Write 8 bits through I2C.
     */
    virtual uint8_t write_8bit(uint16_t register_address, uint8_t value) = 0;

    /**
     * @brief Wait during N milliseconds.
     */
    virtual void wait_ms(uint32_t time_ms) = 0;
};

#endif // VL53L4CD_I2C_HPP_
