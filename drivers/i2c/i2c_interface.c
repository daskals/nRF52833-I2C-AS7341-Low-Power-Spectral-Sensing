/**
 * @file i2c_interface.c
 * @brief I2C (TWI) interface implementation for nRF52 SDK.
 *
 * Provides initialization, event handler, and data transmission functions for TWI/I2C communication.
 */

#include "i2c_interface.h"
#include <stdio.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <string.h>
#include <errno.h>
#include "as7341_defines.h"

#define PIN_I2C_SDA   17 
#define PIN_I2C_SCL   19 


// Transfer completion flag
volatile bool m_xfer_done = false;

// Buffer for received sample data
uint8_t m_sample = 0;

// TWI instance structure
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);



#define HANDLE_ERROR(err_code) \
    if ((err_code) != NRF_SUCCESS) { \
        NRF_LOG_ERROR("I2C Error: %d", err_code); \
    }

/**
 * @brief Enable internal pull-ups on I2C lines.
 */
void i2c_enable_internal_pullups(void)
{
    nrf_gpio_cfg(
        PIN_I2C_SCL,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        PIN_I2C_SDA,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    NRF_LOG_INFO("Internal pull-ups enabled on I2C lines.");
    NRF_LOG_FLUSH();
}

/**
 * @brief Disable internal pull-ups on I2C lines.
 */
void i2c_disable_internal_pullups(void)
{
    nrf_gpio_cfg(
        PIN_I2C_SCL,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        PIN_I2C_SDA,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0D1,
        NRF_GPIO_PIN_NOSENSE);

    NRF_LOG_INFO("Internal pull-ups disabled (lines in high-Z).");
    NRF_LOG_FLUSH();
}

/**
 * @brief Initialize the TWI (I2C) interface.
 *
 * Configures the TWI peripheral with the specified pins and frequency,
 * registers the event handler, and enables the TWI driver.
 */
void i2c_init(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("I2C INIT");

    const nrf_drv_twi_config_t twi_sensor_config = {
       .scl                = PIN_I2C_SCL,
       .sda                = PIN_I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    //i2c_enable_internal_pullups();
    err_code = nrf_drv_twi_init(&m_twi, &twi_sensor_config, NULL, NULL);
    HANDLE_ERROR(err_code);

    nrf_drv_twi_enable(&m_twi);
    NRF_LOG_INFO("I2C INIT COMPLETE");
    NRF_LOG_FLUSH();
}





// Read multiple bytes from a register of an I2C device
bool i2c_read_bytes(uint8_t address, uint8_t reg, uint8_t *buf, size_t len) {
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, address, &reg, 1, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to set register 0x%02X for multi-byte read: %d", reg, err_code);
        return false;
    }
    err_code = nrf_drv_twi_rx(&m_twi, address, buf, len);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to read %d bytes from register 0x%02X: %d", len, reg, err_code);
        return false;
    }
    return true;
}

// Helper function to write a byte to a register (AS7341)
bool as7341_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, AS7341_I2CADDR_DEFAULT, buf, 2, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to write to register 0x%02X: %d", reg, err_code);
        return false;
    }
    return true;
}

// Helper function to read a byte from a register (AS7341)
bool as7341_read_reg(uint8_t reg, uint8_t *value) {
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, AS7341_I2CADDR_DEFAULT, &reg, 1, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to set register 0x%02X: %d", reg, err_code);
        return false;
    }
    err_code = nrf_drv_twi_rx(&m_twi, AS7341_I2CADDR_DEFAULT, value, 1);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to read from register 0x%02X: %d", reg, err_code);
        return false;
    }
    return true;
}

// Helper function to read a 16-bit value from a register (LSB first, AS7341)
bool as7341_read_reg16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    ret_code_t err_code = nrf_drv_twi_tx(&m_twi, AS7341_I2CADDR_DEFAULT, &reg, 1, false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to set register 0x%02X: %d", reg, err_code);
        return false;
    }
    err_code = nrf_drv_twi_rx(&m_twi, AS7341_I2CADDR_DEFAULT, buf, 2);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to read 16-bit from register 0x%02X: %d", reg, err_code);
        return false;
    }
    *value = (uint16_t)(buf[0] | (buf[1] << 8)); // LSB first
    return true;
}