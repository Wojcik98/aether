#ifndef _VL53L4CD_STM_HPP_
#define _VL53L4CD_STM_HPP_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"

#include "vl53l4cd_api.hpp"
#include "vl53l4cd_i2c.hpp"

class VL53L4CD_STM : public VL53L4CD_I2C {
public:
    ~VL53L4CD_STM() = default;

    void set_i2c(I2C_HandleTypeDef *dev_handle);

    void set_address(uint8_t address);

    uint8_t get_address();

    /**
     * @brief Read 32 bits through I2C.
     */
    uint8_t read_32bit(uint16_t register_address, uint32_t *value) override;

    /**
     * @brief Write 32 bits through I2C.
     */
    uint8_t write_32bit(uint16_t register_address, uint32_t value) override;

    /**
     * @brief Read 16 bits through I2C.
     */
    uint8_t read_16bit(uint16_t register_address, uint16_t *value) override;

    /**
     * @brief Write 16 bits through I2C.
     */
    uint8_t write_16bit(uint16_t register_address, uint16_t value) override;

    /**
     * @brief Read 8 bits through I2C.
     */
    uint8_t read_8bit(uint16_t register_address, uint8_t *value) override;

    /**
     * @brief Write 8 bits through I2C.
     */
    uint8_t write_8bit(uint16_t register_address, uint8_t value) override;

    /**
     * @brief Wait during N milliseconds.
     */
    void wait_ms(uint32_t time_ms) override;

private:
    I2C_HandleTypeDef *dev_handle;
    uint8_t address = VL53L4CD_I2C_ADDR_DEFAULT;

    uint8_t hal_addr() { return address << 1; }
};

#endif // _VL53L4CD_STM_HPP_
