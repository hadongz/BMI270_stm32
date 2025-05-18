/*
* BMI270 IMU Sensor Driver
* 
* Copyright (c) 2025 Muhammad Hadi
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* This driver provides functions to interface with the BMI270 IMU sensor
* on STM32 microcontrollers using the HAL library. It supports accelerometer,
* gyroscope, and temperature measurements with calibration support.
*
* The BMI270 is a high-performance Inertial Measurement Unit (IMU) with 
* a 6-axis accelerometer and gyroscope. This driver simplifies configuration
* and data collection from the sensor over I2C interface.
*/

#ifndef BMI270_HAL_H
#define BMP270_HAL_H
 
#if defined(STM32F4xx)
    #include "stm32f4xx_hal.h"
#elif defined(STM32F1xx)
    #include "stm32f1xx_hal.h"
#elif defined(STM32F0xx)
    #include "stm32f0xx_hal.h"
#elif defined(STM32F2xx)
    #include "stm32f2xx_hal.h"
#elif defined(STM32F3xx)
    #include "stm32f3xx_hal.h"
#elif defined(STM32L0xx)
    #include "stm32l0xx_hal.h"
#elif defined(STM32L1xx)
    #include "stm32l1xx_hal.h"
#elif defined(STM32L4xx)
    #include "stm32l4xx_hal.h"
#else
    #error "STM32 family not detected. Please include appropriate HAL header before including BMP180.h"
#endif
#include <math.h>
#include <stdio.h>

#include "bmi2_defs.h"
#include "bmi2.h"
#include "bmi270.h"
#include "madgwick.h"

/**
 * @brief BMI270 device structure containing all sensor state
 * 
 * This structure holds all information related to the BMI270 sensor instance,
 * including hardware interface, configuration settings, and latest sensor readings.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;                /**< HAL I2C handle for communication */
    struct bmi2_dev bmi2_dev;               /**< BMI2 device structure for vendor API */
    struct bmi2_sens_data raw_sens_data;    /**< Raw sensor data from last reading */
    float accel_x;                          /**< Acceleration in m/s² along X-axis */
    float accel_y;                          /**< Acceleration in m/s² along Y-axis */
    float accel_z;                          /**< Acceleration in m/s² along Z-axis */
    float gyro_x;                           /**< Angular rate in degrees/second around X-axis */
    float gyro_y;                           /**< Angular rate in degrees/second around Y-axis */
    float gyro_z;                           /**< Angular rate in degrees/second around Z-axis */
    uint8_t device_ready;                   /**< Flag indicating if device is initialized (1) or not (0) */
    uint8_t accel_range;                    /**< Current accelerometer range setting */
    uint8_t gyro_range;                     /**< Current gyroscope range setting */
    int16_t accel_offsets[3];               /**< Acceleration offsets for x, y, z */
    int16_t gyro_offsets[3];                /**< Gyroscope offsets for x, y, z */
    int8_t calibrated;                      /**< Calibration flag */
} BMI270_DEV;

/**
 * @brief Initialize the BMI270 sensor
 * 
 * This function initializes the BMI270 sensor with default settings for
 * accelerometer and gyroscope. It sets up communication interface and
 * checks that the sensor is responsive and properly configured.
 * 
 * @param[out] dev Pointer to BMI270_DEV structure to be initialized
 * @param[in] hi2c Pointer to HAL I2C handle for sensor communication
 * 
 * @return Result of initialization
 * @retval BMI2_OK (0) Success
 * @retval <0 Error code (see BMI2 error codes in bmi2_defs.h)
 */
int8_t BMI270_init(BMI270_DEV *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief Configure accelerometer settings
 * 
 * This function configures the accelerometer with the specified range and
 * output data rate (ODR).
 * 
 * @param[in] dev Pointer to BMI270_DEV structure
 * @param[in] range Accelerometer range setting (BMI2_ACC_RANGE_2G, BMI2_ACC_RANGE_4G, etc.)
 * @param[in] odr Output data rate setting (BMI2_ACC_ODR_100HZ, BMI2_ACC_ODR_200HZ, etc.)
 * 
 * @return Result of configuration
 * @retval BMI2_OK (0) Success
 * @retval <0 Error code
 */
int8_t BMI270_set_accel_config(BMI270_DEV *dev, uint8_t range, uint8_t odr);

/**
 * @brief Configure gyroscope settings
 * 
 * This function configures the gyroscope with the specified range and
 * output data rate (ODR).
 * 
 * @param[in] dev Pointer to BMI270_DEV structure
 * @param[in] range Gyroscope range setting (BMI2_GYR_RANGE_2000, BMI2_GYR_RANGE_1000, etc.)
 * @param[in] odr Output data rate setting (BMI2_GYR_ODR_100HZ, BMI2_GYR_ODR_200HZ, etc.)
 * 
 * @return Result of configuration
 * @retval BMI2_OK (0) Success
 * @retval <0 Error code
 */
int8_t BMI270_set_gyro_config(BMI270_DEV *dev, uint8_t range, uint8_t odr);

/**
 * @brief Get raw accelerometer and gyroscope data from sensor
 * 
 * This function reads the latest sensor data from the BMI270 and stores
 * the raw values in the dev structure.
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval <0 Error code
 */
int8_t BMI270_get_raw_accel_gyro(BMI270_DEV *dev);

/**
 * @brief Get processed accelerometer, gyroscope, and temperature data
 * 
 * This function reads the latest sensor data, applies scaling based on
 * the current range settings, and stores the values in physical units
 * in the dev structure (m/s² for accelerometer and degrees/s for gyroscope
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval <0 Error code
 */
int8_t BMI270_get_accel_gyro(BMI270_DEV *dev);

/**
 * @brief Records calibration values for the BMI270 sensor on a flat surface
 * 
 * This function calibrates both accelerometer and gyroscope with the sensor 
 * placed on a flat, level surface. It calculates calibration offsets by
 * collecting multiple samples and computing the average deviation from the
 * expected values (0,0,1g for accelerometer, 0,0,0 for gyroscope).
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure to store calibration values
 * @param[in] samples Number of samples to collect for averaging (100+ recommended)
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval BMI2_E_DEV_NOT_FOUND Device not ready
 * @retval BMI2_E_INVALID_STATUS No valid samples collected
 */
int8_t BMI270_record_calibration(BMI270_DEV *dev, uint16_t samples);

/**
 * @brief Sets calibration offset values manually for the BMI270 sensor
 * 
 * This function allows setting pre-determined calibration offsets without
 * performing the actual calibration procedure. This is useful for loading
 * saved calibration values from non-volatile memory or for applying known
 * calibration values from previous sessions.
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure to update
 * @param[in] ax Accelerometer X-axis offset value
 * @param[in] ay Accelerometer Y-axis offset value
 * @param[in] az Accelerometer Z-axis offset value (relative to 1g)
 * @param[in] gx Gyroscope X-axis offset value
 * @param[in] gy Gyroscope Y-axis offset value
 * @param[in] gz Gyroscope Z-axis offset value
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval BMI2_E_DEV_NOT_FOUND Device not ready
 * 
 * @note The calibration offsets are subtracted from raw readings. For example,
 *       if raw accelerometer readings at rest are (50,20,8200) and the expected
 *       values are (0,0,8192), then the offsets should be (50,20,8).
 */
int8_t BMI270_offset_calibration(BMI270_DEV *dev, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);

int8_t BMI270_madgwick_fusion(BMI270_DEV *dev);

#endif /* BMI270_HAL_H */