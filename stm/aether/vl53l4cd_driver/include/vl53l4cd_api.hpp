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

#ifndef VL53L4CD_API_HPP_
#define VL53L4CD_API_HPP_

#include "vl53l4cd_i2c.hpp"

/**
 *  @brief Driver version
 */
constexpr uint8_t VL53L4CD_IMPLEMENTATION_VER_MAJOR = 3;
constexpr uint8_t VL53L4CD_IMPLEMENTATION_VER_MINOR = 0;
constexpr uint8_t VL53L4CD_IMPLEMENTATION_VER_BUILD = 1;
constexpr uint32_t VL53L4CD_IMPLEMENTATION_VER_REVISION = 0;

/**
 *  @brief Driver error type
 */
using VL53L4CD_Error = uint8_t;

constexpr uint8_t VL53L4CD_ERROR_NONE = 0U;
constexpr uint8_t VL53L4CD_ERROR_XTALK_FAILED = 253U;
constexpr uint8_t VL53L4CD_ERROR_INVALID_ARGUMENT = 254U;
constexpr uint8_t VL53L4CD_ERROR_TIMEOUT = 255U;

/**
 *  @brief Inner Macro for API. Not for user, only for development.
 */
constexpr uint16_t VL53L4CD_SOFT_RESET = 0x0000;
constexpr uint16_t VL53L4CD_I2C_SLAVE__DEVICE_ADDRESS = 0x0001;
constexpr uint16_t VL53L4CD_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008;
constexpr uint16_t VL53L4CD_XTALK_PLANE_OFFSET_KCPS = 0x0016;
constexpr uint16_t VL53L4CD_XTALK_X_PLANE_GRADIENT_KCPS = 0x0018;
constexpr uint16_t VL53L4CD_XTALK_Y_PLANE_GRADIENT_KCPS = 0x001A;
constexpr uint16_t VL53L4CD_RANGE_OFFSET_MM = 0x001E;
constexpr uint16_t VL53L4CD_INNER_OFFSET_MM = 0x0020;
constexpr uint16_t VL53L4CD_OUTER_OFFSET_MM = 0x0022;
constexpr uint16_t VL53L4CD_GPIO_HV_MUX__CTRL = 0x0030;
constexpr uint16_t VL53L4CD_GPIO__TIO_HV_STATUS = 0x0031;
constexpr uint16_t VL53L4CD_SYSTEM__INTERRUPT = 0x0046;
constexpr uint16_t VL53L4CD_RANGE_CONFIG_A = 0x005E;
constexpr uint16_t VL53L4CD_RANGE_CONFIG_B = 0x0061;
constexpr uint16_t VL53L4CD_RANGE_CONFIG__SIGMA_THRESH = 0x0064;
constexpr uint16_t VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066;
constexpr uint16_t VL53L4CD_INTERMEASUREMENT_MS = 0x006C;
constexpr uint16_t VL53L4CD_THRESH_HIGH = 0x0072;
constexpr uint16_t VL53L4CD_THRESH_LOW = 0x0074;
constexpr uint16_t VL53L4CD_SYSTEM__INTERRUPT_CLEAR = 0x0086;
constexpr uint16_t VL53L4CD_SYSTEM_START = 0x0087;
constexpr uint16_t VL53L4CD_RESULT__RANGE_STATUS = 0x0089;
constexpr uint16_t VL53L4CD_RESULT__SPAD_NB = 0x008C;
constexpr uint16_t VL53L4CD_RESULT__SIGNAL_RATE = 0x008E;
constexpr uint16_t VL53L4CD_RESULT__AMBIENT_RATE = 0x0090;
constexpr uint16_t VL53L4CD_RESULT__SIGMA = 0x0092;
constexpr uint16_t VL53L4CD_RESULT__DISTANCE = 0x0096;

constexpr uint16_t VL53L4CD_RESULT__OSC_CALIBRATE_VAL = 0x00DE;
constexpr uint16_t VL53L4CD_FIRMWARE__SYSTEM_STATUS = 0x00E5;
constexpr uint16_t VL53L4CD_IDENTIFICATION__MODEL_ID = 0x010F;

/**
 *  @brief defines Software Version
 */
struct VL53L4CD_Version_t {
    uint8_t major;     /*!< major number */
    uint8_t minor;     /*!< minor number */
    uint8_t build;     /*!< build number */
    uint32_t revision; /*!< revision number */
};

/**
 *  @brief Packed reading results type
 */
struct VL53L4CD_ResultsData_t {
    /* Status of measurements. If the status is equal to 0, the data are valid*/
    uint8_t range_status;
    /* Measured distance in millimeters */
    uint16_t distance_mm;
    /* Ambient noise in kcps */
    uint16_t ambient_rate_kcps;
    /* Ambient noise in kcps/SPAD */
    uint16_t ambient_per_spad_kcps;
    /* Measured signal of the target in kcps */
    uint16_t signal_rate_kcps;
    /* Measured signal of the target in kcps/SPAD */
    uint16_t signal_per_spad_kcps;
    /* Number of SPADs enabled */
    uint16_t number_of_spad;
    /* Estimated measurements std deviation in mm */
    uint16_t sigma_mm;
};

class VL53L4CD_API {
public:
    VL53L4CD_API(VL53L4CD_I2C *i2c) : i2c(i2c) {};

    /**
     * @brief This function programs the software driver version.
     * @param (VL53L4CD_Version_t) pVersion : Pointer of structure, containing
     * the software version.
     * @return (VL53L4CD_ERROR) status : 0 if SW version is OK.
     */
    VL53L4CD_Error get_sw_version(VL53L4CD_Version_t *pVersion);

    /**
     * @brief This function sets a new I2C address to a sensor. It can be used
     * for example when multiple sensors share the same I2C bus.
     * @param (Dev_t) dev : Device instance to update.
     * @param (uint8_t) new_address : New I2C address.
     * @return (VL53L4CD_ERROR) status : 0 if I2C address has been correctly
     * programmed.
     */
    VL53L4CD_Error set_i2c_address(uint8_t new_address);

    /**
     * @brief This function is used to get the sensor id of VL53L4CD. The sensor
     * id should be 0xEBAA.
     * @param (Dev_t) dev : Device instance.
     * @param (uint16_t) *p_id : Sensor id.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error get_sensor_id(uint16_t *p_id);

    /**
     * @brief This function is used to initialize the sensor.
     * @param (Dev_t) dev : Device instance to initialize.
     * @return (VL53L4CD_ERROR) status : 0 if init is OK.
     */
    VL53L4CD_Error sensor_init();

    /**
     * @brief This function clears the interrupt. It needs to be called after a
     * ranging data reading to arm the interrupt for the next data ready event.
     * @param (Dev_t) dev : Device instance.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error clear_interrupt();

    /**
     * @brief This function starts a ranging session. The ranging operation is
     * continuous. The clear interrupt has to be done after each get data to
     * allow the interrupt to raise when the next data is ready.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error start_ranging();

    /**
     * @brief This function stops the ranging in progress.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error stop_ranging();

    /**
     * @brief This function check if a new data is available by polling a
     * dedicated register.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint8_t) *p_is_data_ready : Pointer containing a flag to know if
     * a data is ready : 0 = no data ready, 1 = data ready.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error check_for_data_ready(uint8_t *p_is_data_ready);

    /**
     * @brief This function sets new range timing. Timing are composed of
     * TimingBudget and InterMeasurement. TimingBudget represents the timing
     * during VCSEL enabled, and InterMeasurement the time between two
     * measurements. The sensor can have different ranging mode depending of the
     * configuration, please refer to the user manual for more information.
     * @param (uint32_t) timing_budget_ms :  New timing budget in ms. Value can
     * be between 10ms and 200ms. Default is 50ms.
     * @param (uint32_t) inter_measurement_ms :  New inter-measurement in ms. If
     * the value is equal to 0, the ranging period is defined by the timing
     * budget. Otherwise, inter-measurement must be > timing budget. When all
     * the timing budget is consumed, the device goes in low power mode until
     * inter-measurement is done.
     * @return (uint8_t) status : 0 if OK.
     */
    VL53L4CD_Error set_range_timing(uint32_t timing_budget_ms,
                                    uint32_t inter_measurement_ms);

    /**
     * @brief This function gets the current range timing. Timing are composed
     * of TimingBudget and InterMeasurement. TimingBudget represents the timing
     * during VCSEL enabled, and InterMeasurement the time between two
     * measurements. The sensor can have different ranging mode depending of the
     * configuration, please refer to the user manual for more information.
     * @param (uint32_t) *p_timing_budget_ms :  Pointer containing the current
     * timing budget in ms.
     * @param (uint32_t) *p_inter_measurement_ms :  Pointer containing the
     * current inter-measurement in ms.
     * @return (uint8_t) status : 0 if OK.
     */
    VL53L4CD_Error get_range_timing(uint32_t *p_timing_budget_ms,
                                    uint32_t *p_inter_measurement_ms);

    /**
     * @brief This function gets the results reported by the sensor.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (VL53L4CD_ResultsData_t) *pResult :  Pointer of structure, filled
     * with the ranging results.
     * @return (uint8_t) status : 0 if OK.
     */
    VL53L4CD_Error get_result(VL53L4CD_ResultsData_t *pResult);

    /**
     * @brief This function sets a new offset correction in mm. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance.
     * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The
     * minimum value is -1024mm and maximum is 1023mm.
     * @return (uint8_t) status : 0 if OK.
     */
    VL53L4CD_Error set_offset(int16_t OffsetValueInMm);

    /**
     * @brief This function gets the current offset correction in mm. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The
     * minimum value is -1024mm and maximum is 1023mm.
     * @return (uint8_t) status : 0 if OK.
     */
    VL53L4CD_Error get_offset(int16_t *Offset);

    /**
     * @brief This function sets a new Xtalk value in kcps. Xtalk represents the
     * correction to apply to the sensor when a protective coverglass is placed
     * at the top of the sensor.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) XtalkValueKcps : New xtalk value in kcps. The default
     * value is 0 kcps (no coverglass). Minimum is 0 kcps , and maximum is 128
     * kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error set_xtalk(uint16_t XtalkValueKcps);

    /**
     * @brief This function gets the current Xtalk value in kcps. Xtalk
     * represents the correction to apply to the sensor when a protective
     * coverglass is placed at the top of the sensor.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) p_xtalk_kcps : Pointer of current xtalk value in kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error get_xtalk(uint16_t *p_xtalk_kcps);

    /**
     * @brief This function sets new detection thresholds. The detection
     * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1),
     * only when a condition on distance is reach. Example:
     * VL53L4CD_SetDistanceThreshold(dev,100,300,0): Below 100 mm
     * VL53L4CD_SetDistanceThreshold(dev,100,300,1): Above 300 mm
     * VL53L4CD_SetDistanceThreshold(dev,100,300,2): Below 100mm or above 300mm
     * VL53L4CD_SetDistanceThreshold(dev,100,300,3): Above 100mm or below 300mm
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) distance_low_mm : Low distance threshold in
     * millimeters.
     * @param (uint16_t) distance_high_mm : High distance threshold in
     * millimeters.
     * @param (uint8_t) window : Interrupt windows (0=below low threshold;
     * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error set_detection_thresholds(uint16_t distance_low_mm,
                                            uint16_t distance_high_mm,
                                            uint8_t window);

    /**
     * @brief This function gets the current detection thresholds. The detection
     * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1),
     * only when a condition on distance is reach.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) *p_distance_low_mm : Pointer of low distance threshold
     * in millimeters.
     * @param (uint16_t) *p_distance_high_mm : Pointer of high distance
     * threshold in millimeters.
     * @param (uint8_t) *p_window : Interrupt windows (0=below low threshold;
     * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error get_detection_thresholds(uint16_t *p_distance_low_mm,
                                            uint16_t *p_distance_high_mm,
                                            uint8_t *p_window);

    /**
     * @brief This function sets a new signal threshold in kcps. If a
     * target has a lower signal as the programmed value, the result status in
     * structure 'VL53L4CD_ResultsData_t' will be equal to 2.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) signal_kcps : New signal threshold in kcps. The default
     * value is 1024 kcps. Minimum is 0 kcps (no threshold), and maximum is
     * 16384 kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error set_signal_threshold(uint16_t signal_kcps);

    /**
     * @brief This function returns the current signal threshold in kcps. If a
     * target has a lower signal as the programmed value, the result status in
     * structure 'VL53L4CD_ResultsData_t' will be equal to 2.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) *p_signal_kcps : Pointer of signal threshold in kcps.
     * @return (VL53L4CD_ERROR) status : 0 if OK.
     */
    VL53L4CD_Error get_signal_threshold(uint16_t *p_signal_kcps);

    /**
     * @brief This function programs a new sigma threshold. The sigma
     * corresponds to the standard deviation of the returned pulse. If the
     * computed sigma is above the programmed value, the result status in
     * structure 'VL53L4CD_ResultsData_t' will be equal to 1.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) sigma_mm : New sigma threshold in mm. The default value
     * is 15mm. Minimum is 0mm (not threshold), and maximum is 16383mm.
     * @return (VL53L4CD_ERROR) status : 0 if programming is or 255 if value is
     * too high.
     */
    VL53L4CD_Error set_sigma_threshold(uint16_t sigma_mm);

    /**
     * @brief This function gets the current sigma threshold. The sigma
     * corresponds to the standard deviation of the returned pulse. If the
     * computed sigma is above the programmed value, the result status in
     * structure 'VL53L4CD_ResultsData_t' will be equal to 1.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @param (uint16_t) *p_sigma_mm : Current sigma threshold in mm.
     * @return (VL53L4CD_ERROR) status : 0 if programming is OK.
     */
    VL53L4CD_Error get_sigma_threshold(uint16_t *p_sigma_mm);

    /**
     * @brief This function can be called when the temperature might have
     * changed by more than 8 degrees Celsius. The function can only be used if
     * the sensor is not ranging, otherwise, the ranging needs to be stopped
     * using function 'VL53L4CD_StopRanging()'. After calling this function, the
     * ranging can restart normally.
     * @param (Dev_t) dev : instance of selected VL53L4CD sensor.
     * @return (VL53L4CD_ERROR) status : 0 if update is OK.
     */
    VL53L4CD_Error start_temperature_update();

private:
    VL53L4CD_I2C *i2c;
};

#endif // VL53L4CD_API_HPP_
