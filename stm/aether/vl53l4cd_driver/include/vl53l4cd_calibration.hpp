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

/**
 * @file  vl53l4cd_calibration.h
 * @brief Calibration Functions definition
 */

#ifndef VL53L4CD_CALIBRATION_HPP_
#define VL53L4CD_CALIBRATION_HPP_

#include "vl53l4cd_api.hpp"
#include "vl53l4cd_i2c.hpp"

class VL53L4CD_Calibration {
public:
    VL53L4CD_Calibration(VL53L4CD_API *api, VL53L4CD_I2C *i2c)
        : api(api), i2c(i2c) {}
    /**
     * @brief This function can be used to perform an offset calibration. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance. ST recommend to perform offset at 100m, on a grey17%
     * reflective target, but any other distance and reflectance can be used.
     * The function returns the offset value found and programs the offset
     * compensation into the device.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (int16_t) target_dist_mm : Real distance between the sensor and
     * the target in millimeters. ST recommend 100mm. Min distance is 10mm and
     * max is 1000mm.
     * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A
     * higher number of samples increases the accuracy, but it also takes more
     * time. ST recommend to use at least 10 samples.
     * @return (VL53L4CD_ERROR) status : 0 if OK, or 255 if something occurred
     * (e.g invalid nb of samples).
     */
    VL53L4CD_Error calibrate_offset(int16_t target_dist_mm,
                                    int16_t *p_measured_offset_mm,
                                    int16_t nb_samples);

    /**
     * @brief This function can be used to perform a Xtalk calibration. Xtalk
     * represents the correction to apply to the sensor when a protective
     * coverglass is placed at the top of the sensor. The distance for
     * calibration depends of the coverglass, it needs to be characterized.
     * Please refer to the User Manual for more information. The function
     * returns the Xtalk value found and programs the Xtalk compensation into
     * the device.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param uint16_t) target_dist_mm : Real distance between the sensor and
     * the target in millimeters. This distance needs to be characterized, as
     * described into the User Manual.
     * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A
     * higher number of samples increases the accuracy, but it also takes more
     * time. ST recommend to use at least 10 samples.
     * @return (VL53L4CD_ERROR) status : 0 if OK, or 255 if something occurred
     * (e.g invalid nb of samples).
     */
    VL53L4CD_Error calibrate_xtalk(int16_t target_dist_mm,
                                   uint16_t *p_measured_xtalk_kcps,
                                   int16_t nb_samples);

private:
    VL53L4CD_API *api;
    VL53L4CD_I2C *i2c;
};
#endif // VL53L4CD_CALIBRATION_HPP_
