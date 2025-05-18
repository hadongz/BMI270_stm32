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

#include "BMI270_stm32.h"

/* Private function prototypes */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void bmi2_delay_us(uint32_t period, void *intf_ptr);
static void enable_dwt_cycle_counter(void);

/**
 * @brief I2C read function for BMI270 sensor
 * 
 * This function implements the vendor-defined I2C read interface for the BMI270 sensor.
 * It converts between STM32 HAL I2C functions and the BMI2 driver API requirements.
 * 
 * @param[in] reg_addr Register address to read from
 * @param[out] reg_data Pointer to buffer where read data will be stored
 * @param[in] len Number of bytes to read
 * @param[in] intf_ptr Pointer to interface (I2C_HandleTypeDef in this case)
 * 
 * @return Result of operation
 * @retval BMI2_OK Success
 * @retval BMI2_E_NULL_PTR Null pointer error
 * @retval BMI2_E_COM_FAIL Communication failure
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) 
{
  if (intf_ptr == NULL) {
    return BMI2_E_NULL_PTR;
  }

  I2C_HandleTypeDef *h12c = (I2C_HandleTypeDef *) intf_ptr;
  uint8_t shifted_addr = (BMI2_I2C_SEC_ADDR << 1);

  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(h12c, shifted_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 1000);

  if (status == HAL_OK) {
    return BMI2_OK;
  } else {
    return BMI2_E_COM_FAIL;
  }
}

/**
 * @brief I2C write function for BMI270 sensor
 * 
 * This function implements the vendor-defined I2C write interface for the BMI270 sensor.
 * It converts between STM32 HAL I2C functions and the BMI2 driver API requirements.
 * 
 * @param[in] reg_addr Register address to write to
 * @param[in] reg_data Pointer to data to be written
 * @param[in] len Number of bytes to write
 * @param[in] intf_ptr Pointer to interface (I2C_HandleTypeDef in this case)
 * 
 * @return Result of operation
 * @retval BMI2_OK Success
 * @retval BMI2_E_NULL_PTR Null pointer error
 * @retval BMI2_E_COM_FAIL Communication failure
 */
static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) 
{
  if (intf_ptr == NULL) {
    return BMI2_E_NULL_PTR;
  }

  I2C_HandleTypeDef *h12c = (I2C_HandleTypeDef *) intf_ptr;
  uint8_t shifted_addr = (BMI2_I2C_SEC_ADDR << 1);

  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Write(h12c, shifted_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *) reg_data, len, 1000);

  if (status == HAL_OK) {
    return BMI2_OK;
  } else {
    return BMI2_E_COM_FAIL;
  }
}

/**
 * @brief Microsecond delay function for BMI270 sensor
 * 
 * This function implements a microsecond delay using the DWT cycle counter
 * for Cortex-M3 and higher, or a software delay loop for other cores.
 * 
 * @param[in] period Delay duration in microseconds
 * @param[in] intf_ptr Interface pointer (not used in this implementation)
 */
static void bmi2_delay_us(uint32_t period, void *intf_ptr) 
{
    (void)intf_ptr; // Unused parameter
    
    #if (__CORTEX_M >= 0x03U)
    // Use DWT cycle counter for precise timing on Cortex-M3 and higher
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = period * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start_tick) < delay_ticks);
    #else
    // Software delay loop for other cores
    volatile uint32_t count = period * (SystemCoreClock / 1000000U / 4U); // Adjust divisor
    while (count > 0) {
        count--;
        __NOP(); // No operation, helps prevent optimization out
    }
    if (period == 0) { // Ensure at least a minimal delay if 0 is passed
        __NOP();
    }
    #endif
}

/**
 * @brief Enable DWT cycle counter for precise timing
 * 
 * This function enables the Data Watchpoint and Trace (DWT) cycle counter
 * for Cortex-M3 and higher cores. This allows for precise microsecond timing.
 */
static void enable_dwt_cycle_counter(void) 
{
    #if (__CORTEX_M >= 0x03U)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  #endif
}

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
int8_t BMI270_init(BMI270_DEV *dev, I2C_HandleTypeDef *hi2c) 
{
    // Enable DWT cycle counter for microsecond delay function
    enable_dwt_cycle_counter();

    int8_t rslt;
    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    
    // Store I2C handle
    dev->hi2c = hi2c;
    dev->device_ready = 0;
    
    // Set up the bmi2_dev structure
    dev->bmi2_dev.intf_ptr = hi2c;
    dev->bmi2_dev.chip_id = BMI270_CHIP_ID;
    dev->bmi2_dev.intf = BMI2_I2C_INTF;
    dev->bmi2_dev.read = bmi2_i2c_read;
    dev->bmi2_dev.write = bmi2_i2c_write;
    dev->bmi2_dev.delay_us = bmi2_delay_us;
    dev->bmi2_dev.read_write_len = 64; // Maximum buffer length
    dev->bmi2_dev.config_file_ptr = NULL; // No custom config file
    
    // Initialize BMI270
    rslt = bmi270_init(&dev->bmi2_dev);
    if (rslt != BMI2_OK) return rslt;
    
    // Enable accelerometer and gyroscope
    rslt = bmi270_sensor_enable(sens_list, 2, &dev->bmi2_dev);
    if (rslt != BMI2_OK) return rslt;
    
    // Default configurations for accelerometer (±4G, 100Hz)
    rslt = BMI270_set_accel_config(dev, BMI2_ACC_RANGE_4G, BMI2_ACC_ODR_100HZ);
    if (rslt != BMI2_OK) return rslt;
    
    // Default configurations for gyroscope (±2000dps, 100Hz)
    rslt = BMI270_set_gyro_config(dev, BMI2_GYR_RANGE_2000, BMI2_GYR_ODR_100HZ);
    if (rslt != BMI2_OK) return rslt;
    
    // Mark device as ready
    dev->device_ready = 1;
    
    return BMI2_OK;
}

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
int8_t BMI270_set_accel_config(BMI270_DEV *dev, uint8_t range, uint8_t odr) 
{
    int8_t rslt;
    struct bmi2_sens_config config;
    
    // Prepare configuration structure
    config.type = BMI2_ACCEL;
    config.cfg.acc.odr = odr;
    config.cfg.acc.range = range;
    config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;      // Normal bandwidth with average of 4 samples
    config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // Performance optimized mode
    
    // Apply configuration to sensor
    rslt = bmi2_set_sensor_config(&config, 1, &dev->bmi2_dev);
    
    // Store current range setting
    if (rslt == BMI2_OK) {
        dev->accel_range = range;
    }
    
    return rslt;
}

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
int8_t BMI270_set_gyro_config(BMI270_DEV *dev, uint8_t range, uint8_t odr) 
{
    int8_t rslt;
    struct bmi2_sens_config config;
    
    // Prepare configuration structure
    config.type = BMI2_GYRO;
    config.cfg.gyr.odr = odr;
    config.cfg.gyr.range = range;
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;       // Normal bandwidth
    config.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;  // Performance optimized for noise
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // Performance optimized filtering
    
    // Apply configuration to sensor
    rslt = bmi2_set_sensor_config(&config, 1, &dev->bmi2_dev);
    
    // Store current range setting
    if (rslt == BMI2_OK) {
        dev->gyro_range = range;
    }
    
    return rslt;
}

/**
 * @brief Get accelerometer and gyroscope data from sensor without calibration or normalization
 * 
 * This function reads the latest sensor data from the BMI270 and stores
 * the values in the dev structure.
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval BMI2_E_DEV_NOT_FOUND Device not ready
 * @retval <0 Other error code
 */
int8_t BMI270_get_raw_accel_gyro(BMI270_DEV *dev) 
{
    int8_t rslt;
    struct bmi2_sens_data sensor_data;
    
    // Check if device is initialized
    if (!dev->device_ready) return BMI2_E_DEV_NOT_FOUND;
    
    // Read sensor data
    rslt = bmi2_get_sensor_data(&sensor_data, &dev->bmi2_dev);
    if (rslt != BMI2_OK) return rslt;

    // Store the data in the device structure
    dev->raw_sens_data = sensor_data;

    return BMI2_OK;
}

/**
 * @brief Get processed accelerometer, gyroscope
 * 
 * This function reads the latest sensor data, applies scaling based on
 * the current range settings, and stores the values in physical units
 * in the dev structure (m/s² for accelerometer and degrees/s for gyroscope.
 * if dev->calibrated is 1 it will apply calibration by subtracting the 
 * stored offset values
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval <0 Error code
 */
int8_t BMI270_get_accel_gyro(BMI270_DEV *dev) 
{
    if (!dev->device_ready) return BMI2_E_DEV_NOT_FOUND;

    // First get the raw sensor data
    int8_t rslt;
    rslt = BMI270_get_raw_accel_gyro(dev);
    if (rslt != BMI2_OK) return rslt;

    // Calibrate by subtracting with offset
    if (dev->calibrated)
    {
        dev->raw_sens_data.acc.x -= dev->accel_offsets[0];
        dev->raw_sens_data.acc.y -= dev->accel_offsets[1];
        dev->raw_sens_data.acc.z -= dev->accel_offsets[2];

        dev->raw_sens_data.gyr.x -= dev->gyro_offsets[0];
        dev->raw_sens_data.gyr.y -= dev->gyro_offsets[1];
        dev->raw_sens_data.gyr.z -= dev->gyro_offsets[2];
    }

    float acc_scale, gyro_scale;
    
    // Calculate scale factor based on current range setting
    switch(dev->accel_range) {
        case BMI2_ACC_RANGE_2G:
            acc_scale = 2.0f * 9.81f / 32768.0f;
            break;
        case BMI2_ACC_RANGE_4G:
            acc_scale = 4.0f * 9.81f / 32768.0f;
            break;
        case BMI2_ACC_RANGE_8G:
            acc_scale = 8.0f * 9.81f / 32768.0f;
            break;
        case BMI2_ACC_RANGE_16G:
            acc_scale = 16.0f * 9.81f / 32768.0f;
            break;
        default:
            acc_scale = 4.0f * 9.81f / 32768.0f; // Default to 4G
    }

    // Calculate scale factor based on current range setting
    switch(dev->gyro_range) {
        case BMI2_GYR_RANGE_125:
            gyro_scale = 125.0f / 32768.0f;
            break;
        case BMI2_GYR_RANGE_250:
            gyro_scale = 250.0f / 32768.0f;
            break;
        case BMI2_GYR_RANGE_500:
            gyro_scale = 500.0f / 32768.0f;
            break;
        case BMI2_GYR_RANGE_1000:
            gyro_scale = 1000.0f / 32768.0f;
            break;
        case BMI2_GYR_RANGE_2000:
            gyro_scale = 2000.0f / 32768.0f;
            break;
        default:
            gyro_scale = 2000.0f / 32768.0f; 
    }
    
    // Apply scaling to convert raw values to physical units
    dev->accel_x = dev->raw_sens_data.acc.x * acc_scale;
    dev->accel_y = dev->raw_sens_data.acc.y * acc_scale;
    dev->accel_z = dev->raw_sens_data.acc.z * acc_scale;
    
    dev->gyro_x = dev->raw_sens_data.gyr.x * gyro_scale;
    dev->gyro_y = dev->raw_sens_data.gyr.y * gyro_scale;
    dev->gyro_z = dev->raw_sens_data.gyr.z * gyro_scale;
    
    return BMI2_OK;
}

/**
 * @brief Records calibration values for the BMI270 sensor on a flat surface
 * 
 * This function calibrates both accelerometer and gyroscope with the sensor 
 * placed on a flat, level surface. It calculates calibration offsets by
 * collecting multiple samples and computing the average deviation from the
 * expected values.
 * 
 * For the accelerometer:
 * - X and Y axes should read 0 when flat
 * - Z axis should read 1g (value depends on configured range)
 * 
 * For the gyroscope:
 * - All axes should read 0 when stationary
 * 
 * The function automatically determines the proper Z-axis target value
 * based on the accelerometer range configuration stored in the device structure.
 * 
 * @param[in,out] dev Pointer to BMI270_DEV structure to store calibration values
 * @param[in] samples Number of samples to collect for averaging (100+ recommended)
 * 
 * @return Result of operation
 * @retval BMI2_OK (0) Success
 * @retval BMI2_E_DEV_NOT_FOUND Device not ready
 * @retval BMI2_E_INVALID_STATUS No valid samples collected
 */
int8_t BMI270_record_calibration(BMI270_DEV *dev, uint16_t samples) 
{
    if (!dev->device_ready) return BMI2_E_DEV_NOT_FOUND;
    if (samples < 10) samples = 100; // Default to 100 samples
    
    int32_t acc_sum[3] = {0};
    int32_t gyr_sum[3] = {0};
    uint16_t valid_samples = 0;
    
    printf("CALIBRATING, DON'T MOVE THE SENSOR\n");

    int16_t accel_z_target;
    switch(dev->accel_range) {
        case BMI2_ACC_RANGE_2G:
            accel_z_target = 32767 / 2; // 1g at ±2G range
            break;
        case BMI2_ACC_RANGE_4G:
            accel_z_target = 32767 / 4; // 1g at ±4G range
            break;
        case BMI2_ACC_RANGE_8G:
            accel_z_target = 32767 / 8; // 1g at ±8G range
            break;
        case BMI2_ACC_RANGE_16G:
            accel_z_target = 32767 / 16; // 1g at ±16G range
            break;
        default:
            accel_z_target = 32767 / 4; // Default to 2G
    }

    // Collect samples
    for (uint16_t i = 0; i < samples; i++) {
        if (BMI270_get_raw_accel_gyro(dev) == BMI2_OK) {
            acc_sum[0] += dev->raw_sens_data.acc.x;
            acc_sum[1] += dev->raw_sens_data.acc.y;
            acc_sum[2] += dev->raw_sens_data.acc.z;
            
            gyr_sum[0] += dev->raw_sens_data.gyr.x;
            gyr_sum[1] += dev->raw_sens_data.gyr.y;
            gyr_sum[2] += dev->raw_sens_data.gyr.z;
            
            valid_samples++;
        }
        
        // Show progress every 20 samples
        if (i % 20 == 0) {
            printf(".");
        }
        
        HAL_Delay(5); 
    }
    
    if (valid_samples == 0) return BMI2_E_INVALID_STATUS;
    
    // Calculate average values
    for (int i = 0; i < 3; i++) {
        // Calculate the offset as the difference between the average and the target
        if (i == 2) {
            // For Z-axis, we want 1g (8192 for ±4G range)
            dev->accel_offsets[i] = (acc_sum[i] / valid_samples) - accel_z_target;
        } else {
            // For X and Y axes, we want 0
            dev->accel_offsets[i] = acc_sum[i] / valid_samples;
        }
        
        // For gyroscope, we want all axes to be 0
        dev->gyro_offsets[i] = gyr_sum[i] / valid_samples;
    }
    
    printf("\nCALIBRATION DONE\n");
    printf("ACCEL OFFSET: X: %d, Y: %d, Z: %d | GYRO OFFSET X: %d, Y: %d, Z: %d\n", 
           dev->accel_offsets[0], dev->accel_offsets[1], dev->accel_offsets[2],
           dev->gyro_offsets[0], dev->gyro_offsets[1], dev->gyro_offsets[2]);
    
    dev->calibrated = 1;

    return BMI2_OK;
}

/**
 * @brief Sets calibration offset values manually for the BMI270 sensor
 * 
 * This function allows setting pre-determined calibration offsets without
 * performing the actual calibration procedure. This is useful for:
 * - Loading saved calibration values from non-volatile memory
 * - Applying known calibration values from previous sessions
 * - Setting calibration parameters without requiring a level surface
 * - Restoring factory calibration values
 * 
 * The offsets are stored in the device structure and will be applied to
 * raw sensor readings when the device is marked as calibrated.
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
 */
int8_t BMI270_offset_calibration(BMI270_DEV *dev, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) 
{
    if (!dev->device_ready) return BMI2_E_DEV_NOT_FOUND;

    dev->accel_offsets[0] = ax;
    dev->accel_offsets[1] = ay;
    dev->accel_offsets[2] = az;

    dev->gyro_offsets[0] = gx;
    dev->gyro_offsets[1] = gy;
    dev->gyro_offsets[2] = gz;

    dev->calibrated = 1;

    return BMI2_OK;
}