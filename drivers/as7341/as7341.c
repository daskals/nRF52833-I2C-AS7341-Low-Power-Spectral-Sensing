// as7341.h
#ifndef AS7341_H
#define AS7341_H

#include <stdint.h>
#include <stdbool.h>
#include "as7341_defines.h"


#endif // AS7341_H

// as7341.c
#include "as7341.h"
#include "i2c_interface.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static as7341_gain_t s_as7341_gain = AS7341_GAIN_256X;


// Test connection to AS7341 sensor
bool test_as7341_connection(void) {
    NRF_LOG_DEBUG("test_as7341_connection() called");
    NRF_LOG_INFO("TEST AS7341 CONNECTION");
    NRF_LOG_FLUSH();

    uint8_t chip_id = 0;
    if (!as7341_read_reg(AS7341_WHOAMI, &chip_id)) {
        NRF_LOG_ERROR("Failed to read WHOAMI register");
        NRF_LOG_FLUSH();
        return false;
    }

    NRF_LOG_INFO("AS7341 WHOAMI: 0x%x", chip_id);
    NRF_LOG_FLUSH();

    if ((chip_id & 0xFC) == (AS7341_CHIP_ID << 2)) {
        NRF_LOG_INFO("AS7341 successfully detected", AS7341_I2C_ADDR);
        NRF_LOG_FLUSH();
        return true;
    } else {
        NRF_LOG_WARNING("Unexpected WHOAMI response. Expected 0x%x, got 0x%x", AS7341_CHIP_ID << 2, chip_id & 0xFC);
        NRF_LOG_FLUSH();
        return false;
    }
}


// Enable or disable sensor power
void as7341_power_enable(bool enable) {
    NRF_LOG_DEBUG("as7341_power_enable() called: enable=%d", enable);
    uint8_t enable_val;
    as7341_read_reg(AS7341_ENABLE, &enable_val);
    enable_val = (enable_val & ~0x01) | (enable ? 0x01 : 0x00);
    as7341_write_reg(AS7341_ENABLE, enable_val);
}

/**
 * @brief Initialize the AS7341 sensor with default configuration and remember gain for later use.
 */
void as7341_init_sensor(void)
{
    NRF_LOG_INFO("as7341 Init");
    // Example: Configure AS7341 with gain = 256X, atime = 100, astep = 999
    s_as7341_gain = AS7341_GAIN_1X; // Store gain for later normalization
    uint8_t atime = 35;
    uint16_t astep = 999;
    // Total time = (ATIME+1)*(ASTEP+1)*2.78us
    as7341_configure(s_as7341_gain, atime, astep);
}



/**
 * @brief Configures the AS7341 for spectral measurement.
 *
 * This function sets the ADC integration parameters, gain, and initial configuration registers.
 * It does not perform SMUX channel routing — that should be handled by set_smux_low_channels() or related logic.
 *
 * @param gain  Gain multiplier (use `as7341_gain_t` enum)
 * @param atime Integration time (0–255) — controls ADC conversion time
 * @param astep Integration step size (0–65535) — affects total integration window
 * @return true if all register writes succeed, false otherwise
 */
bool as7341_configure(as7341_gain_t gain, uint8_t atime, uint16_t astep) {
    NRF_LOG_DEBUG("as7341_configure() called: gain=%d, atime=%u, astep=%u", gain, atime, astep);
    bool ok = true;

    // Set integration time: (ATIME + 1) × (ASTEP + 1) × 2.78 µs
    ok &= as7341_write_reg(AS7341_ATIME, atime);
    ok &= as7341_write_reg(AS7341_ASTEP_L, (uint8_t)(astep & 0xFF));
    ok &= as7341_write_reg(AS7341_ASTEP_H, (uint8_t)(astep >> 8));

    // CFG0: Bit 0 = Low power (0 = normal), Bits 4-5 = register bank (00 = default)
    //       Bit 6 = Spectral trigger mode lengthening (0 = disable)
    ok &= as7341_write_reg(AS7341_CFG0, 0x00); // Normal power mode, bank 0

    // CFG1: Gain control — lower 4 bits define gain multiplier
    ok &= as7341_write_reg(AS7341_CFG1, (uint8_t)gain);

    // CFG6: Sets SMUX configuration (used with SMUX_CMD_WRITE); here 0x30 is the Adafruit default
    // for enabling F1-F4 and Clear/NIR SMUX channels (manual override)
    ok &= as7341_write_reg(AS7341_CFG6, 0x30);

    return ok;
}



// Enable spectral measurement
bool as7341_enable_spectral_measurement(bool enable) {
    NRF_LOG_DEBUG("as7341_enable_spectral_measurement() called: enable=%d", enable);
    uint8_t enable_val;
    as7341_read_reg(AS7341_ENABLE, &enable_val);
    enable_val = (enable_val & ~0x02) | (enable ? 0x02 : 0x00);
    return as7341_write_reg(AS7341_ENABLE, enable_val);
}

// Read all channels (F1-F8, Clear, NIR)
bool as7341_read_all_channels(uint16_t *readings_buffer) {
    NRF_LOG_DEBUG("as7341_read_all_channels() called");
    as7341_power_enable(true);
    // Configure SMUX for low channels (F1-F4, Clear, NIR)
    as7341_set_smux_low_channels(true);
    as7341_enable_spectral_measurement(true);
    as7341_delay_for_data(0);

    // Read 12 bytes (6 channels, 2 bytes each) from the data registers using i2c_read_bytes
    uint8_t buf[12] = {0};
    if (!i2c_read_bytes(AS7341_I2CADDR_DEFAULT, AS7341_CH0_DATA_L, buf, 12)) {
        NRF_LOG_ERROR("Failed to read channel data from AS7341");
        return false;
    }
    // Buffer mapping for low channels:
    // readings_buffer[0] = F1
    // readings_buffer[1] = F2
    // readings_buffer[2] = F3
    // readings_buffer[3] = F4
    // readings_buffer[4] = CLEAR_L
    // readings_buffer[5] = NIR_L
    for (int i = 0; i < 6; i++) {
        readings_buffer[i] = (uint16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));
    }
    // Configure SMUX for high channels (F5-F8, Clear, NIR)
    as7341_set_smux_low_channels(false);
    as7341_enable_spectral_measurement(true);
    as7341_delay_for_data(0);

    if (!i2c_read_bytes(AS7341_I2CADDR_DEFAULT, AS7341_CH0_DATA_L, buf, 12)) {
        NRF_LOG_ERROR("Failed to read high channel data from AS7341");
        return false;
    }
    // Buffer mapping for high channels:
    // readings_buffer[6] = F5
    // readings_buffer[7] = F6
    // readings_buffer[8] = F7
    // readings_buffer[9] = F8
    // readings_buffer[10] = CLEAR_H
    // readings_buffer[11] = NIR_H
    for (int i = 0; i < 6; i++) {
        readings_buffer[6 + i] = (uint16_t)(buf[i * 2] | (buf[i * 2 + 1] << 8));
    }


    as7341_enable_spectral_measurement(false);
    as7341_power_enable(false);
    return true;
}


// Delay for data readiness (nRF52 style, no POSIX timespec)
void as7341_delay_for_data(int wait_time_ms) {
    NRF_LOG_DEBUG("as7341_delay_for_data() called: wait_time_ms=%d", wait_time_ms);
    if (wait_time_ms == 0) {
        // Wait until data ready
        while (!as7341_get_is_data_ready()) {
            //nrf_delay_ms(1);
        }
    } else if (wait_time_ms > 0) {
        uint32_t elapsed = 0;
        while (!as7341_get_is_data_ready() && elapsed < (uint32_t)wait_time_ms) {
            nrf_delay_ms(1);
            elapsed++;
        }
    }
}

// Configure SMUX for low or high channels
void as7341_set_smux_low_channels(bool f1_f4) {
    NRF_LOG_DEBUG("as7341_set_smux_low_channels() called: f1_f4=%d", f1_f4);
    as7341_enable_spectral_measurement(false);
    as7341_set_smux_command(AS7341_SMUX_CMD_WRITE);
    if (f1_f4) {
        as7341_setup_f1f4_clear_nir();
    } else {
        as7341_setup_f5f8_clear_nir();
    }
    as7341_enable_smux();
}

// Check if data is ready
bool as7341_get_is_data_ready(void) {
    NRF_LOG_DEBUG("as7341_get_is_data_ready() called");
    uint8_t status = 0;
    as7341_read_reg(AS7341_STATUS2, &status);
    return (status & 0x40) != 0; // AVALID bit
}

// Enable SMUX
bool as7341_enable_smux(void) {
    NRF_LOG_DEBUG("as7341_enable_smux() called");
    uint8_t enable_val = 0;
    as7341_read_reg(AS7341_ENABLE, &enable_val);
    enable_val |= 0x10; // Set SMUX enable bit
    if (!as7341_write_reg(AS7341_ENABLE, enable_val)) return false;

    for (uint16_t count = 0; count < 1000; count++) {
        as7341_read_reg(AS7341_ENABLE, &enable_val);
        if (!(enable_val & 0x10)) return true;
        //nrf_delay_ms(1);
    }
    return false;
}

/**
 * @brief Configure SMUX for F1-F4, Clear, and NIR channels.
 *
 * This function sets the SMUX registers to route F1-F4, Clear, and NIR channels to ADCs.
 */
void as7341_setup_f1f4_clear_nir(void) {
    NRF_LOG_DEBUG("as7341_setup_f1f4_clear_nir() called");
    as7341_write_reg(0x00, 0x30); // F3 left to ADC2
    as7341_write_reg(0x01, 0x01); // F1 left to ADC0
    as7341_write_reg(0x02, 0x00); // Reserved
    as7341_write_reg(0x03, 0x00); // F8 left disabled
    as7341_write_reg(0x04, 0x00); // F6 left disabled
    as7341_write_reg(0x05, 0x42); // F4 left to ADC3, F2 left to ADC1
    as7341_write_reg(0x06, 0x00); // F5 left disabled
    as7341_write_reg(0x07, 0x00); // F7 left disabled
    as7341_write_reg(0x08, 0x50); // CLEAR to ADC4
    as7341_write_reg(0x09, 0x00); // F5 right disabled
    as7341_write_reg(0x0A, 0x00); // F7 right disabled
    as7341_write_reg(0x0B, 0x00); // Reserved
    as7341_write_reg(0x0C, 0x20); // F2 right to ADC1
    as7341_write_reg(0x0D, 0x04); // F4 right to ADC3
    as7341_write_reg(0x0E, 0x00); // F6/F8 right disabled
    as7341_write_reg(0x0F, 0x30); // F3 right to ADC2
    as7341_write_reg(0x10, 0x01); // F1 right to ADC0
    as7341_write_reg(0x11, 0x50); // CLEAR right to ADC4
    as7341_write_reg(0x12, 0x00); // Reserved
    as7341_write_reg(0x13, 0x06); // NIR to ADC5
}

/**
 * @brief Configure SMUX for F5-F8, Clear, and NIR channels.
 *
 * This function sets the SMUX registers to route F5-F8, Clear, and NIR channels to ADCs.
 */
void as7341_setup_f5f8_clear_nir(void) {
    NRF_LOG_DEBUG("as7341_setup_f5f8_clear_nir() called");
    as7341_write_reg(0x00, 0x00); // F3 left disabled
    as7341_write_reg(0x01, 0x00); // F1 left disabled
    as7341_write_reg(0x02, 0x00); // Reserved
    as7341_write_reg(0x03, 0x40); // F8 left to ADC3
    as7341_write_reg(0x04, 0x02); // F6 left to ADC1
    as7341_write_reg(0x05, 0x00); // F4/F2 disabled
    as7341_write_reg(0x06, 0x10); // F5 left to ADC0
    as7341_write_reg(0x07, 0x03); // F7 left to ADC2
    as7341_write_reg(0x08, 0x50); // CLEAR to ADC4
    as7341_write_reg(0x09, 0x10); // F5 right to ADC0
    as7341_write_reg(0x0A, 0x03); // F7 right to ADC2
    as7341_write_reg(0x0B, 0x00); // Reserved
    as7341_write_reg(0x0C, 0x00); // F2 right disabled
    as7341_write_reg(0x0D, 0x00); // F4 right disabled
    as7341_write_reg(0x0E, 0x24); // F8 right to ADC2, F6 right to ADC1
    as7341_write_reg(0x0F, 0x00); // F3 right disabled
    as7341_write_reg(0x10, 0x00); // F1 right disabled
    as7341_write_reg(0x11, 0x50); // CLEAR right to ADC4
    as7341_write_reg(0x12, 0x00); // Reserved
    as7341_write_reg(0x13, 0x06); // NIR to ADC5
}

/**
 * @brief Set the SMUX command register.
 *
 * @param[in] command SMUX command to set (see as7341_smux_cmd_t).
 * @return true if register write succeeds, false otherwise.
 */
bool as7341_set_smux_command(as7341_smux_cmd_t command) {
    NRF_LOG_DEBUG("as7341_set_smux_command() called: command=%d", command);
    uint8_t cfg6_val = 0;
    as7341_read_reg(AS7341_CFG6, &cfg6_val);
    cfg6_val = (cfg6_val & ~0x18) | ((uint8_t)command << 3);
    return as7341_write_reg(AS7341_CFG6, cfg6_val);
}


/**
 * @brief Print the AS7341 channel readings.
 */
void as7341_sample_and_print(void)
{
    NRF_LOG_DEBUG("as7341_sample_and_print() called");
    uint16_t channel_readings[12] = {0};
    int32_t lux_value;
    int32_t par_value;
    /*
     * AS7341 Channel Mapping (Wavelengths):
     *  F1  (channel_readings[0])  : 415 nm
     *  F2  (channel_readings[1])  : 445 nm
     *  F3  (channel_readings[2])  : 480 nm
     *  F4  (channel_readings[3])  : 515 nm
     *  CLEAR_L (channel_readings[4]) : Clear (low) channel
     *  NIR_L   (channel_readings[5]) : Near-IR (low) channel
     *  F5  (channel_readings[6])  : 555 nm
     *  F6  (channel_readings[7])  : 590 nm
     *  F7  (channel_readings[8])  : 630 nm
     *  F8  (channel_readings[9])  : 680 nm
     *  CLEAR_H (channel_readings[10]): Clear (high) channel
     *  NIR_H   (channel_readings[11]): Near-IR (high) channel
     */
    if (as7341_read_all_channels(channel_readings)) {
        NRF_LOG_INFO("AS7341 Channel F1 (415nm, Violet): %u", channel_readings[0]);
        NRF_LOG_INFO("AS7341 Channel F2 (445nm, Blue): %u", channel_readings[1]);
        NRF_LOG_INFO("AS7341 Channel F3 (480nm, Blue-Green): %u", channel_readings[2]);
        NRF_LOG_INFO("AS7341 Channel F4 (515nm, Green): %u", channel_readings[3]);
        NRF_LOG_INFO("AS7341 Channel F5 (555nm, Green-Yellow): %u", channel_readings[6]);
        NRF_LOG_INFO("AS7341 Channel F6 (590nm, Yellow-Orange): %u", channel_readings[7]);
        NRF_LOG_INFO("AS7341 Channel F7 (630nm, Orange): %u", channel_readings[8]);
        NRF_LOG_INFO("AS7341 Channel F8 (680nm, Red): %u", channel_readings[9]);
        NRF_LOG_INFO("AS7341 Channel CLEAR_L (380-700 nm): %u", channel_readings[4]);
        NRF_LOG_INFO("AS7341 Channel NIR_L (910 nm): %u", channel_readings[5]);
        NRF_LOG_INFO("AS7341 Channel CLEAR_H (380-700 nm): %u", channel_readings[10]);
        NRF_LOG_INFO("AS7341 Channel NIR_H (910 nm): %u", channel_readings[11]);

      
        par_value = as7341_calculate_par_from_channels(channel_readings);
        NRF_LOG_INFO("AS7341 Par: %d", par_value);
    } else {
        NRF_LOG_WARNING("AS7341 channel read failed.");
    }
}

/**
 * @brief Regression-calibrated coefficients for PAR (μmol/m²/s).
 * Order: F1 (415nm), F2 (445nm), F3 (480nm), F4 (515nm), F5 (555nm), F6 (590nm), F7 (630nm), F8 (690nm)
 */
static const float par_regression_coeffs[8] = {
    4.7280568f,   // F1 - 415 nm
    -0.6033910f,  // F2 - 445 nm
    -1.5187001f,  // F3 - 480 nm
    0.4630669f,   // F4 - 515 nm
    0.6610031f,   // F5 - 555 nm
    -1.6748574f,  // F6 - 590 nm
    0.7903176f,   // F7 - 630 nm
    -0.2266746f   // F8 - 690 nm
};

/**
 * @brief Gain factors for each supported AS7341 gain setting.
 * Index matches as7341_gain_t enum.
 */
static const float gain_factors[] = {
    0.5f,   // AS7341_GAIN_0_5X
    1.0f,   // AS7341_GAIN_1X
    2.0f,   // AS7341_GAIN_2X
    4.0f,   // AS7341_GAIN_4X
    8.0f,   // AS7341_GAIN_8X
    16.0f,  // AS7341_GAIN_16X
    32.0f,  // AS7341_GAIN_32X
    64.0f,  // AS7341_GAIN_64X
    128.0f, // AS7341_GAIN_128X
    256.0f, // AS7341_GAIN_256X
    512.0f  // AS7341_GAIN_512X
};

/**
 * @brief Calculate PAR index (relative μmol/m²/s) from AS7341 channel readings using regression coefficients.
 *
 * Uses only F1-F8 channels and applies regression-calibrated coefficients.
 *
 * @param[in] channel_readings Pointer to array of 12 channel readings.
 * @return int32_t Calculated PAR index, or -1 on error.
 */
int32_t as7341_calculate_par_from_channels(const uint16_t *channel_readings)
{
    // Validate input pointer
    if (channel_readings == NULL) {
        NRF_LOG_ERROR("as7341_calculate_par_from_channels: NULL channel_readings");
        return -1;
    }
    // Validate gain index
    if (s_as7341_gain >= (sizeof(gain_factors) / sizeof(gain_factors[0]))) {
        NRF_LOG_ERROR("as7341_calculate_par_from_channels: Invalid gain index");
        return -1;
    }

    float gain = gain_factors[s_as7341_gain];
    float par = -1.9196374f; // Intercept

    // Apply regression coefficients to F1-F8 channels
    par += par_regression_coeffs[0] * channel_readings[0];  // F1
    par += par_regression_coeffs[1] * channel_readings[1];  // F2
    par += par_regression_coeffs[2] * channel_readings[2];  // F3
    par += par_regression_coeffs[3] * channel_readings[3];  // F4
    par += par_regression_coeffs[4] * channel_readings[6];  // F5
    par += par_regression_coeffs[5] * channel_readings[7];  // F6
    par += par_regression_coeffs[6] * channel_readings[8];  // F7
    par += par_regression_coeffs[7] * channel_readings[9];  // F8

    return (int32_t)(par / gain);
}

