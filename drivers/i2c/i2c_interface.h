/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/**
 * @file i2c_interface.h
 * @brief I2C (TWI) interface header for nRF52 SDK.
 *
 * Provides initialization, event handler, and data transmission functions for TWI/I2C communication.
 */

#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include "nrf_drv_twi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TWI transfer completion flag.
 * Set to true when a transfer is finished.
 */
extern volatile bool m_xfer_done;

/**
 * @brief Buffer for received sample data.
 */
extern uint8_t m_sample;

/**
 * @brief TWI instance structure.
 */
extern const nrf_drv_twi_t m_twi;

/**
 * @brief Enable internal pull-ups for I2C lines.
 */
void i2c_enable_internal_pullups(void);

/**
 * @brief Disable internal pull-ups for I2C lines.
 */
void i2c_disable_internal_pullups(void);

/**
 * @brief Initialize the TWI (I2C) interface.
 *
 * Configures and enables the TWI peripheral for communication.
 */
void i2c_init(void);

/**
 * @brief Transmit a 32-bit unsigned integer over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_uint(uint32_t value, uint8_t address);

/**
 * @brief Transmit a 32-bit unsigned long over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_ulong(uint32_t value, uint8_t address);

/**
 * @brief Transmit a 16-bit unsigned short over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_ushort(uint16_t value, uint8_t address);

/**
 * @brief Transmit an 8-bit unsigned char over I2C and optionally receive a response.
 *
 * @param value   Value to transmit.
 * @param address I2C address of the target device.
 * @return Received value if applicable, 0 otherwise.
 */
uint8_t i2c_transmit_uchar(uint8_t value, uint8_t address);

/**
 * @brief Read multiple bytes from a register of an I2C device.
 *
 * Sends the register address, then reads 'len' bytes into 'buf'.
 * @param address I2C address of the device.
 * @param reg Register address to read from.
 * @param buf Buffer to store the read data.
 * @param len Number of bytes to read.
 * @return true if successful, false otherwise.
 */
bool i2c_read_bytes(uint8_t address, uint8_t reg, uint8_t *buf, size_t len);

/**
 * @brief Write a byte to a register of the AS7341 sensor.
 * @param reg Register address.
 * @param value Value to write.
 * @return true if successful, false otherwise.
 */
bool as7341_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief Read a byte from a register of the AS7341 sensor.
 * @param reg Register address.
 * @param value Pointer to store the read value.
 * @return true if successful, false otherwise.
 */
bool as7341_read_reg(uint8_t reg, uint8_t *value);

/**
 * @brief Read a 16-bit value from a register (LSB first) of the AS7341 sensor.
 * @param reg Register address.
 * @param value Pointer to store the read 16-bit value.
 * @return true if successful, false otherwise.
 */
bool as7341_read_reg16(uint8_t reg, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif /* I2C_INTERFACE_H */
// End of file