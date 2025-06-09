/*
 *  @file as7341.h
 *  @brief AS7341 11-Channel Spectral Sensor driver header for nRF52 SDK
 *
 *  Adapted for nRF52 SDK from Adafruit_AS7341 Arduino library.
 *  All Arduino/C++ dependencies removed. For use with nRF52833 and compatible MCUs.
 *
 *  Original Author: Bryan Siepert for Adafruit Industries
 *  Adaptation: [Your Name/Company]
 *  License: BSD (see license.txt)
 */

#ifndef AS7341_H__
#define AS7341_H__

#include <stdint.h>
#include <stdbool.h>
#include "as7341_defines.h"

// Default I2C address for AS7341
#define AS7341_I2C_ADDR 0x39

// Register address for channel data low byte (adjust if needed)
#define AS7341_CH0_DATA_L 0x95

// Sensor state struct (should be defined elsewhere if not here)
// extern as7341_sensor_t sensor;

/**
 * @brief Test connection to AS7341 sensor.
 * @return true if successful, false otherwise.
 */
bool test_as7341_connection(void);

/**
 * @brief Configure the AS7341 sensor (gain, atime, astep).
 * @param gain Gain setting.
 * @param atime Integration time.
 * @param astep Step count.
 * @return true if successful, false otherwise.
 */
bool as7341_configure(as7341_gain_t gain, uint8_t atime, uint16_t astep);

/**
 * @brief Read mode 1 channels from AS7341.
 * @param buffer Buffer to store channel readings.
 * @param len Number of channels to read.
 * @return true if successful, false otherwise.
 */
bool as7341_read_mode1_channels(uint16_t *buffer, uint8_t len);

/**
 * @brief Read all channels (F1-F8, Clear, NIR) from the AS7341.
 * @param readings_buffer Buffer to store 12 uint16_t readings (6 low + 6 high channels).
 * @return true if successful, false otherwise.
 */
bool as7341_read_all_channels(uint16_t *readings_buffer);

/**
 * @brief Enable or disable sensor power.
 * @param enable true to enable, false to disable.
 */
void as7341_power_enable(bool enable);

/**
 * @brief Enable or disable spectral measurement.
 * @param enable true to enable, false to disable.
 * @return true if successful, false otherwise.
 */
bool as7341_enable_spectral_measurement(bool enable);

/**
 * @brief Delay for data readiness/acquisition.
 * @param wait_time Time to wait in ms, or 0 to wait until data ready.
 */
void as7341_delay_for_data(int wait_time_ms);

/**
 * @brief Configure SMUX for low or high channels.
 * @param low true for low channels (F1-F4, Clear, NIR), false for high channels (F5-F8, Clear, NIR).
 */
void as7341_set_smux_low_channels(bool f1_f4);

/**
 * @brief Check if data is ready.
 * @return true if data is ready, false otherwise.
 */
bool as7341_get_is_data_ready(void);

/**
 * @brief Enable SMUX.
 * @return true if successful, false otherwise.
 */
bool as7341_enable_smux(void);

/**
 * @brief SMUX configurations for F1-F4, Clear, NIR.
 */
void as7341_setup_f1f4_clear_nir(void);

/**
 * @brief SMUX configurations for F5-F8, Clear, NIR.
 */
void as7341_setup_f5f8_clear_nir(void);

/**
 * @brief Set SMUX command.
 * @param command SMUX command to set.
 * @return true if successful, false otherwise.
 */
bool as7341_set_smux_command(as7341_smux_cmd_t command);

/**
 * @brief Sample and print sensor data.
 */
void as7341_sample_and_print(void);

/**
 * @brief Calculate Photosynthetically Active Radiation (PAR) value.
 * @return Calculated PAR value in micromoles per square meter per second (µmol/m²/s).
 */
int32_t as7341_calculate_par(const uint16_t *channel_readings); // deprecated, use _from_channels

/**
 * @brief Calculate Lux value from channel readings.
 * @return Calculated Lux value.
 */
int32_t as7341_calculate_lux(const uint16_t *channel_readings); // deprecated, use _from_channels

/**
 * @brief Calculate Photosynthetically Active Radiation (PAR) value from channels.
 * @param channel_readings Pointer to an array of channel readings.
 * @return Calculated PAR value in micromoles per square meter per second (µmol/m²/s).
 */
int32_t as7341_calculate_par_from_channels(const uint16_t *channel_readings);

/**
 * @brief Calculate Lux value from channels.
 * @param channel_readings Pointer to an array of channel readings.
 * @return Calculated Lux value.
 */
int32_t as7341_calculate_lux_from_channels(const uint16_t *channel_readings);

/**
 * @brief Apply IIR filter to current sensor reading.
 * @param previous Previous filtered value.
 * @param current Current sensor reading.
 * @param alpha Filter coefficient (0-255, higher is smoother).
 * @return Filtered value.
 */
int32_t iir_filter(int32_t previous, int32_t current, uint16_t alpha);

/**
 * @brief Initialize the AS7341 sensor (wrapper for as7341_init).
 */
void as7341_init_sensor(void);

/**
 * @brief Get the current gain setting of the AS7341 sensor.
 * @return Current gain setting.
 */
as7341_gain_t as7341_get_current_gain(void);

#endif // AS7341_H__
