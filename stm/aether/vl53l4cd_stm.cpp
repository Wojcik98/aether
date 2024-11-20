#include "vl53l4cd_stm.hpp"

void VL53L4CD_STM::set_i2c(I2C_HandleTypeDef *dev_handle) {
    this->dev_handle = dev_handle;
}

void VL53L4CD_STM::set_address(uint8_t address) { this->address = address; }

uint8_t VL53L4CD_STM::get_address() { return address; }

uint8_t VL53L4CD_STM::read_32bit(uint16_t register_address, uint32_t *value) {
    uint8_t data[4];
    uint8_t status = HAL_I2C_Mem_Read(dev_handle, hal_addr(), register_address,
                                      I2C_MEMADD_SIZE_16BIT, data, 4, 1000);
    if (status == HAL_OK) {
        *value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    }
    return status;
}

uint8_t VL53L4CD_STM::write_32bit(uint16_t register_address, uint32_t value) {
    uint8_t data[4] = {
        (uint8_t)((value >> 24) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF),
    };
    return HAL_I2C_Mem_Write(dev_handle, hal_addr(), register_address,
                             I2C_MEMADD_SIZE_16BIT, data, 4, 1000);
}

uint8_t VL53L4CD_STM::read_16bit(uint16_t register_address, uint16_t *value) {
    uint8_t data[2];
    uint8_t status = HAL_I2C_Mem_Read(dev_handle, hal_addr(), register_address,
                                      I2C_MEMADD_SIZE_16BIT, data, 2, 1000);
    if (status == HAL_OK) {
        *value = (data[0] << 8) | data[1];
    }
    return status;
}

uint8_t VL53L4CD_STM::write_16bit(uint16_t register_address, uint16_t value) {
    uint8_t data[2] = {
        (uint8_t)((value >> 8) & 0xFF),
        (uint8_t)(value & 0xFF),
    };
    return HAL_I2C_Mem_Write(dev_handle, hal_addr(), register_address,
                             I2C_MEMADD_SIZE_16BIT, data, 2, 1000);
}

uint8_t VL53L4CD_STM::read_8bit(uint16_t register_address, uint8_t *value) {
    return HAL_I2C_Mem_Read(dev_handle, hal_addr(), register_address,
                            I2C_MEMADD_SIZE_16BIT, value, 1, 1000);
}

uint8_t VL53L4CD_STM::write_8bit(uint16_t register_address, uint8_t value) {
    return HAL_I2C_Mem_Write(dev_handle, hal_addr(), register_address,
                             I2C_MEMADD_SIZE_16BIT, &value, 1, 1000);
}

void VL53L4CD_STM::wait_ms(uint32_t time_ms) { HAL_Delay(time_ms); }
