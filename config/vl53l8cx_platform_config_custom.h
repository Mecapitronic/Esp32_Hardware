/**
 ******************************************************************************
 * @file    vl53l8cx_platform_config_custom.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 November 2021
 * @brief   Header file with the custom platform settings.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_
#define _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_

/*
 * @brief If you want to customize these defines you can add in the application
 * code the file platform_config_custom.h file where you can override some of
 * these defines.
 */

/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#ifndef VL53L8CX_NB_TARGET_PER_ZONE
  #define   VL53L8CX_NB_TARGET_PER_ZONE   1U
#endif

/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this macro
 * allows to use the firmware format instead of user format. The firmware format allows
 * an increased precision.
 */

// #define  VL53L8CX_USE_RAW_FORMAT

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

  #define VL53L8CX_DISABLE_AMBIENT_PER_SPAD
  #define VL53L8CX_DISABLE_NB_SPADS_ENABLED
  #define VL53L8CX_DISABLE_NB_TARGET_DETECTED
  #define VL53L8CX_DISABLE_SIGNAL_PER_SPAD
  #define VL53L8CX_DISABLE_RANGE_SIGMA_MM
//  #define VL53L8CX_DISABLE_DISTANCE_MM
  #define VL53L8CX_DISABLE_REFLECTANCE_PERCENT
//  #define VL53L8CX_DISABLE_TARGET_STATUS
  #define VL53L8CX_DISABLE_MOTION_INDICATOR

#endif  // _VL53L8CX_PLATFORM_CONFIG_CUSTOM_H_


/*
To ensure data consistency, ST always recommends keeping the ‘number of targets detected’ and ‘target status’
enabled. This filters the measurements depending on the target status (refer to Section 5.5: Results
interpretation).

5.5 Results interpretation
The data returned by the VL53L8CX can be filtered to take into account the target status. The status indicates the
measurement validity. The full status list is described in the following table.

Table 4. List of available target status
Target status Description
0 Ranging data are not updated
1 Signal rate too low on SPAD array
2 Target phase
3 Sigma estimator too high
4 Target consistency failed
5 Range valid
6 Wrap around not performed (typically the first range)
7 Rate consistency failed
8 Signal rate too low for the current target
9 Range valid with large pulse (may be due to a merged target)
10 Range valid, but no target detected at previous range
11 Measurement consistency failed
12 Target blurred by another one, due to sharpener
13 Target detected but inconsistent data. Frequently happens for secondary targets.
255 No target detected (only if number of targets detected is enabled)

To have consistent data, the user needs to filter invalid target status. To give a confidence rating, a target with
status 5 is considered as 100% valid. A status of 6 or 9 can be considered with a confidence value of 50%. All
other statuses are below the 50% confidence level.

*/