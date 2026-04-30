#include "as7341.h"
#include "i2c_interface.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static as7341_gain_t s_as7341_gain = AS7341_GAIN_1X;

/* Gain multiplier lookup — index matches as7341_gain_t enum (DS Fig 49) */
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
    512.0f, // AS7341_GAIN_512X
};

#define GAIN_TABLE_SIZE (sizeof(gain_factors) / sizeof(gain_factors[0]))

/* ---------------------------------------------------------------------------
 * Connection test
 * ---------------------------------------------------------------------------*/

bool test_as7341_connection(void)
{
    NRF_LOG_INFO("TEST AS7341 CONNECTION");
    NRF_LOG_FLUSH();

    uint8_t chip_id = 0;
    if (!as7341_read_reg(AS7341_WHOAMI, &chip_id))
    {
        NRF_LOG_ERROR("AS7341: Failed to read WHOAMI register");
        NRF_LOG_FLUSH();
        return false;
    }
    NRF_LOG_INFO("AS7341 WHOAMI: 0x%x", chip_id);
    NRF_LOG_FLUSH();

    if ((chip_id & 0xFCu) == (AS7341_CHIP_ID << 2))
    {
        NRF_LOG_INFO("AS7341 successfully detected");
        NRF_LOG_FLUSH();
        return true;
    }
    NRF_LOG_WARNING("AS7341: unexpected WHOAMI 0x%x (expected 0x%x)",
                    chip_id & 0xFCu, AS7341_CHIP_ID << 2);
    NRF_LOG_FLUSH();
    return false;
}

/* ---------------------------------------------------------------------------
 * Power and measurement enable
 * ---------------------------------------------------------------------------*/

void as7341_power_enable(bool enable)
{
    uint8_t val = 0;
    as7341_read_reg(AS7341_ENABLE, &val);
    val = (val & ~AS7341_ENABLE_PON) | (enable ? AS7341_ENABLE_PON : 0x00u);
    as7341_write_reg(AS7341_ENABLE, val);
}

bool as7341_enable_spectral_measurement(bool enable)
{
    uint8_t val = 0;
    as7341_read_reg(AS7341_ENABLE, &val);
    val = (val & ~AS7341_ENABLE_SP_EN) | (enable ? AS7341_ENABLE_SP_EN : 0x00u);
    return as7341_write_reg(AS7341_ENABLE, val);
}

/* ---------------------------------------------------------------------------
 * Initialisation and configuration
 * ---------------------------------------------------------------------------*/

bool as7341_init_sensor(void)
{
    NRF_LOG_INFO("AS7341 Init");

    /* DS §10.2.1: set PON='1' first, then configure registers */
    as7341_power_enable(true);
    nrf_delay_ms(5); /* allow oscillator to stabilise in IDLE state */

    s_as7341_gain = AS7341_GAIN_1X;
    /* Integration time = (35+1)*(999+1)*2.78 µs ≈ 100 ms */
    return as7341_configure(AS7341_GAIN_1X, 35, 999);
}

bool as7341_configure(as7341_gain_t gain, uint8_t atime, uint16_t astep)
{
    bool ok = true;

    /* Track gain for PAR/lux normalisation — must be updated before any cal */
    s_as7341_gain = gain;

    /* Integration time: (ATIME+1) × (ASTEP+1) × 2.78 µs (DS §10.2.2) */
    ok &= as7341_write_reg(AS7341_ATIME, atime);
    ok &= as7341_write_reg(AS7341_ASTEP_L, (uint8_t)(astep & 0xFFu));
    ok &= as7341_write_reg(AS7341_ASTEP_H, (uint8_t)(astep >> 8));

    /* CFG0: normal power mode, bank 0, no trigger lengthening */
    ok &= as7341_write_reg(AS7341_CFG0, 0x00u);

    /* CFG1: gain — lower 5 bits (DS Fig 49) */
    ok &= as7341_write_reg(AS7341_CFG1, (uint8_t)gain);

    /* NOTE: CFG6 (SMUX_CMD) is written by as7341_set_smux_command() immediately
     * before each measurement pass. Do NOT write it here — a stray value
     * would corrupt the reserved bits [7:5] (DS §10, "gray fields must not
     * be changed at any time"). */

    return ok;
}

/* ---------------------------------------------------------------------------
 * SMUX helpers
 * ---------------------------------------------------------------------------*/

bool as7341_set_smux_command(as7341_smux_cmd_t command)
{
    uint8_t val = 0;
    as7341_read_reg(AS7341_CFG6, &val);
    val = (val & ~AS7341_CFG6_SMUX_CMD_MASK) |
          ((uint8_t)command << AS7341_CFG6_SMUX_CMD_SHIFT);
    return as7341_write_reg(AS7341_CFG6, val);
}

bool as7341_enable_smux(void)
{
    uint8_t val = 0;
    as7341_read_reg(AS7341_ENABLE, &val);
    val |= AS7341_ENABLE_SMUXEN;
    if (!as7341_write_reg(AS7341_ENABLE, val)) return false;

    /* Hardware clears SMUXEN automatically when the transfer completes */
    for (uint16_t i = 0; i < 1000; i++)
    {
        nrf_delay_ms(1);
        as7341_read_reg(AS7341_ENABLE, &val);
        if (!(val & AS7341_ENABLE_SMUXEN)) return true;
    }
    NRF_LOG_WARNING("AS7341: SMUX enable timeout");
    return false;
}

void as7341_set_smux_low_channels(bool f1_f4)
{
    as7341_enable_spectral_measurement(false);
    as7341_set_smux_command(AS7341_SMUX_CMD_WRITE);
    if (f1_f4)
    {
        as7341_setup_f1f4_clear_nir();
    }
    else
    {
        as7341_setup_f5f8_clear_nir();
    }
    as7341_enable_smux();
}

void as7341_setup_f1f4_clear_nir(void)
{
    /* Route F1-F4, Clear, NIR to ADC0-ADC5
     * Register format: upper nibble = right PD → ADC, lower nibble = left PD → ADC */
    as7341_write_reg(0x00, 0x30); // F3  left  → ADC2
    as7341_write_reg(0x01, 0x01); // F1  left  → ADC0
    as7341_write_reg(0x02, 0x00);
    as7341_write_reg(0x03, 0x00);
    as7341_write_reg(0x04, 0x00);
    as7341_write_reg(0x05, 0x42); // F4  left  → ADC3, F2 left → ADC1
    as7341_write_reg(0x06, 0x00);
    as7341_write_reg(0x07, 0x00);
    as7341_write_reg(0x08, 0x50); // Clear     → ADC4
    as7341_write_reg(0x09, 0x00);
    as7341_write_reg(0x0A, 0x00);
    as7341_write_reg(0x0B, 0x00);
    as7341_write_reg(0x0C, 0x20); // F2  right → ADC1
    as7341_write_reg(0x0D, 0x04); // F4  right → ADC3
    as7341_write_reg(0x0E, 0x00);
    as7341_write_reg(0x0F, 0x30); // F3  right → ADC2
    as7341_write_reg(0x10, 0x01); // F1  right → ADC0
    as7341_write_reg(0x11, 0x50); // Clear     → ADC4
    as7341_write_reg(0x12, 0x00);
    as7341_write_reg(0x13, 0x06); // NIR       → ADC5
}

void as7341_setup_f5f8_clear_nir(void)
{
    /* Route F5-F8, Clear, NIR to ADC0-ADC5 */
    as7341_write_reg(0x00, 0x00);
    as7341_write_reg(0x01, 0x00);
    as7341_write_reg(0x02, 0x00);
    as7341_write_reg(0x03, 0x40); // F8  left  → ADC3
    as7341_write_reg(0x04, 0x02); // F6  left  → ADC1
    as7341_write_reg(0x05, 0x00);
    as7341_write_reg(0x06, 0x10); // F5  left  → ADC0
    as7341_write_reg(0x07, 0x03); // F7  left  → ADC2
    as7341_write_reg(0x08, 0x50); // Clear     → ADC4
    as7341_write_reg(0x09, 0x10); // F5  right → ADC0
    as7341_write_reg(0x0A, 0x03); // F7  right → ADC2
    as7341_write_reg(0x0B, 0x00);
    as7341_write_reg(0x0C, 0x00);
    as7341_write_reg(0x0D, 0x00);
    as7341_write_reg(0x0E, 0x24); // F8  right → ADC3, F6 right → ADC1
    as7341_write_reg(0x0F, 0x00);
    as7341_write_reg(0x10, 0x00);
    as7341_write_reg(0x11, 0x50); // Clear     → ADC4
    as7341_write_reg(0x12, 0x00);
    as7341_write_reg(0x13, 0x06); // NIR       → ADC5
}

/* ---------------------------------------------------------------------------
 * Data ready and delay
 * ---------------------------------------------------------------------------*/

bool as7341_get_is_data_ready(void)
{
    uint8_t status = 0;
    as7341_read_reg(AS7341_STATUS2, &status);
    return (status & AS7341_STATUS2_AVALID) != 0;
}

void as7341_delay_for_data(int wait_time_ms)
{
    if (wait_time_ms == 0)
    {
        while (!as7341_get_is_data_ready())
        {
            nrf_delay_ms(1);
        }
    }
    else
    {
        uint32_t elapsed = 0;
        while (!as7341_get_is_data_ready() && elapsed < (uint32_t)wait_time_ms)
        {
            nrf_delay_ms(1);
            elapsed++;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Channel reads
 * ---------------------------------------------------------------------------*/

bool as7341_read_mode1_channels(uint16_t *buffer, uint8_t len)
{
    if (buffer == NULL || len == 0) return false;
    uint8_t count = (len > 6u) ? 6u : len;
    uint8_t buf[12] = {0};
    if (!i2c_read_bytes(AS7341_I2CADDR_DEFAULT, AS7341_CH0_DATA_L, buf, count * 2u))
    {
        NRF_LOG_ERROR("AS7341: mode1 channel read failed");
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i] = (uint16_t)(buf[i * 2u] | ((uint16_t)buf[i * 2u + 1u] << 8));
    }
    return true;
}

bool as7341_read_all_channels(uint16_t *readings_buffer)
{
    /*
     * Buffer layout (12 entries, DS Fig 7):
     *  [0]=F1(415nm) [1]=F2(445nm) [2]=F3(480nm) [3]=F4(515nm)
     *  [4]=Clear_L   [5]=NIR_L
     *  [6]=F5(555nm) [7]=F6(590nm) [8]=F7(630nm) [9]=F8(680nm)
     *  [10]=Clear_H  [11]=NIR_H
     */
    as7341_power_enable(true);

    /* --- Pass 1: low channels F1-F4, Clear, NIR --- */
    as7341_set_smux_low_channels(true);
    as7341_enable_spectral_measurement(true);
    as7341_delay_for_data(0);

    uint8_t buf[12] = {0};
    if (!i2c_read_bytes(AS7341_I2CADDR_DEFAULT, AS7341_CH0_DATA_L, buf, 12))
    {
        NRF_LOG_ERROR("AS7341: low channel read failed");
        as7341_power_enable(false);
        return false;
    }
    for (int i = 0; i < 6; i++)
    {
        readings_buffer[i] = (uint16_t)(buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8));
    }

    /* --- Pass 2: high channels F5-F8, Clear, NIR --- */
    as7341_set_smux_low_channels(false);
    as7341_enable_spectral_measurement(true);
    as7341_delay_for_data(0);

    if (!i2c_read_bytes(AS7341_I2CADDR_DEFAULT, AS7341_CH0_DATA_L, buf, 12))
    {
        NRF_LOG_ERROR("AS7341: high channel read failed");
        as7341_power_enable(false);
        return false;
    }
    for (int i = 0; i < 6; i++)
    {
        readings_buffer[6 + i] = (uint16_t)(buf[i * 2] | ((uint16_t)buf[i * 2 + 1] << 8));
    }

    as7341_enable_spectral_measurement(false);
    as7341_power_enable(false);
    return true;
}

/* ---------------------------------------------------------------------------
 * Sample and log
 * ---------------------------------------------------------------------------*/

void as7341_sample_and_print(void)
{
    uint16_t ch[12] = {0};
    if (!as7341_read_all_channels(ch))
    {
        NRF_LOG_WARNING("AS7341: channel read failed");
        return;
    }

    /* Warn if saturation occurred — readings may be clipped (DS §10, STATUS2) */
    uint8_t status2 = 0;
    as7341_read_reg(AS7341_STATUS2, &status2);
    if (status2 & (AS7341_STATUS2_ASAT_DIG | AS7341_STATUS2_ASAT_ANA))
    {
        NRF_LOG_WARNING("AS7341: saturation detected — readings may be clipped");
    }

    NRF_LOG_INFO("-----------------------------");
    NRF_LOG_INFO("AS7341 F1  415nm: %u", ch[AS7341_CHANNEL_415nm_F1]);
    NRF_LOG_INFO("AS7341 F2  445nm: %u", ch[AS7341_CHANNEL_445nm_F2]);
    NRF_LOG_INFO("AS7341 F3  480nm: %u", ch[AS7341_CHANNEL_480nm_F3]);
    NRF_LOG_INFO("AS7341 F4  515nm: %u", ch[AS7341_CHANNEL_515nm_F4]);
    NRF_LOG_INFO("AS7341 F5  555nm: %u", ch[AS7341_CHANNEL_555nm_F5]);
    NRF_LOG_INFO("AS7341 F6  590nm: %u", ch[AS7341_CHANNEL_590nm_F6]);
    NRF_LOG_INFO("AS7341 F7  630nm: %u", ch[AS7341_CHANNEL_630nm_F7]);
    NRF_LOG_INFO("AS7341 F8  680nm: %u", ch[AS7341_CHANNEL_680nm_F8]);
    NRF_LOG_INFO("AS7341 Clear_L  : %u", ch[AS7341_CHANNEL_CLEAR_0]);
    NRF_LOG_INFO("AS7341 NIR_L    : %u", ch[AS7341_CHANNEL_NIR_0]);
    NRF_LOG_INFO("AS7341 Clear_H  : %u", ch[AS7341_CHANNEL_CLEAR]);
    NRF_LOG_INFO("AS7341 NIR_H    : %u", ch[AS7341_CHANNEL_NIR]);
    NRF_LOG_INFO("AS7341 PAR      : %d", as7341_calculate_par_from_channels(ch));
    NRF_LOG_INFO("AS7341 Lux      : %d", as7341_calculate_lux_from_channels(ch));
}

/* ---------------------------------------------------------------------------
 * Gain accessor
 * ---------------------------------------------------------------------------*/

as7341_gain_t as7341_get_current_gain(void)
{
    return s_as7341_gain;
}

/* ---------------------------------------------------------------------------
 * IIR filter
 * ---------------------------------------------------------------------------*/

int32_t iir_filter(int32_t previous, int32_t current, uint16_t alpha)
{
    if (alpha > 255u) alpha = 255u;
    /* alpha=255 → heavily weighted to previous; alpha=0 → pass current through */
    return (int32_t)(((uint32_t)alpha * (uint32_t)previous +
                      (uint32_t)(255u - alpha) * (uint32_t)current) / 255u);
}

/* ---------------------------------------------------------------------------
 * PAR calculation
 *
 * Regression coefficients calibrated at GAIN_1X, ATIME=35, ASTEP=999.
 * Raw counts are normalised to 1x-equivalent by dividing by the gain factor.
 * Channel order matches readings_buffer layout from as7341_read_all_channels().
 * ---------------------------------------------------------------------------*/

static const float par_regression_coeffs[8] = {
    4.7280568f,   // F1 415 nm
   -0.6033910f,   // F2 445 nm
   -1.5187001f,   // F3 480 nm
    0.4630669f,   // F4 515 nm
    0.6610031f,   // F5 555 nm
   -1.6748574f,   // F6 590 nm
    0.7903176f,   // F7 630 nm
   -0.2266746f,   // F8 680 nm
};

int32_t as7341_calculate_par_from_channels(const uint16_t *channel_readings)
{
    if (channel_readings == NULL) return -1;
    if (s_as7341_gain >= GAIN_TABLE_SIZE) return -1;

    float gain = gain_factors[s_as7341_gain];
    float par  = -1.9196374f; // regression intercept

    par += par_regression_coeffs[0] * channel_readings[AS7341_CHANNEL_415nm_F1];
    par += par_regression_coeffs[1] * channel_readings[AS7341_CHANNEL_445nm_F2];
    par += par_regression_coeffs[2] * channel_readings[AS7341_CHANNEL_480nm_F3];
    par += par_regression_coeffs[3] * channel_readings[AS7341_CHANNEL_515nm_F4];
    par += par_regression_coeffs[4] * channel_readings[AS7341_CHANNEL_555nm_F5];
    par += par_regression_coeffs[5] * channel_readings[AS7341_CHANNEL_590nm_F6];
    par += par_regression_coeffs[6] * channel_readings[AS7341_CHANNEL_630nm_F7];
    par += par_regression_coeffs[7] * channel_readings[AS7341_CHANNEL_680nm_F8];

    return (int32_t)(par / gain);
}

int32_t as7341_calculate_par(const uint16_t *channel_readings)
{
    return as7341_calculate_par_from_channels(channel_readings);
}

/* ---------------------------------------------------------------------------
 * Lux calculation
 *
 * Weights from CIE photopic luminosity function sampled at each channel centre
 * wavelength (relative, not calibrated in absolute lux).
 * ---------------------------------------------------------------------------*/

static const float lux_coeffs[8] = {
    0.001f,  // F1 415 nm — negligible photopic response
    0.015f,  // F2 445 nm
    0.076f,  // F3 480 nm
    0.243f,  // F4 515 nm
    0.503f,  // F5 555 nm — near photopic peak
    0.450f,  // F6 590 nm
    0.265f,  // F7 630 nm
    0.066f,  // F8 680 nm
};

int32_t as7341_calculate_lux_from_channels(const uint16_t *channel_readings)
{
    if (channel_readings == NULL) return -1;
    if (s_as7341_gain >= GAIN_TABLE_SIZE) return -1;

    float gain = gain_factors[s_as7341_gain];
    float lux  = 0.0f;

    lux += lux_coeffs[0] * channel_readings[AS7341_CHANNEL_415nm_F1];
    lux += lux_coeffs[1] * channel_readings[AS7341_CHANNEL_445nm_F2];
    lux += lux_coeffs[2] * channel_readings[AS7341_CHANNEL_480nm_F3];
    lux += lux_coeffs[3] * channel_readings[AS7341_CHANNEL_515nm_F4];
    lux += lux_coeffs[4] * channel_readings[AS7341_CHANNEL_555nm_F5];
    lux += lux_coeffs[5] * channel_readings[AS7341_CHANNEL_590nm_F6];
    lux += lux_coeffs[6] * channel_readings[AS7341_CHANNEL_630nm_F7];
    lux += lux_coeffs[7] * channel_readings[AS7341_CHANNEL_680nm_F8];

    return (int32_t)(lux / gain);
}

int32_t as7341_calculate_lux(const uint16_t *channel_readings)
{
    return as7341_calculate_lux_from_channels(channel_readings);
}
