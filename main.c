// -----------------------------------------------------------------------------
// License and File Description
// -----------------------------------------------------------------------------
/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * nRF52833 low-power spectral sensing with AS7341.
 * Periodic RTC-triggered AS7341 sampling, DCDC enabled.
 * Platform: nRF52833, nRF5 SDK 17.1.0
 */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "i2c_interface.h"
#include "nrf_gpio.h"

// -----------------------------------------------------------------------------
// Defines and Constants
// -----------------------------------------------------------------------------
#define LED_FUNCTIONALITY_ENABLED   1
#define SAMPLE_INTERVAL_SEC         1
#define SAMPLE_INTERVAL_MS          (SAMPLE_INTERVAL_SEC * 1000)
#define RTC_FREQUENCY               32

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
const  nrf_drv_rtc_t  rtc       = NRF_DRV_RTC_INSTANCE(2);
static uint32_t       rtc_ticks = RTC_US_TO_TICKS(SAMPLE_INTERVAL_MS * 1000, RTC_FREQUENCY);
static bool           as7341_initialized = false;

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------
static void rtc_handler(nrf_drv_rtc_int_type_t int_type);
static void lfclk_config(void);
static void rtc_config(void);
void init_log(void);

// -----------------------------------------------------------------------------
// Function Implementations
// -----------------------------------------------------------------------------

/**
 * @brief RTC interrupt handler. Triggers AS7341 sampling and resets RTC compare.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        if (as7341_initialized)
        {
            as7341_sample_and_print();
        }

#if LED_FUNCTIONALITY_ENABLED
        LEDS_INVERT(BSP_LED_1_MASK);
#endif

        nrf_drv_rtc_counter_clear(&rtc);
        uint32_t err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief Configure low frequency clock source.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**
 * @brief Configure and initialize RTC instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    nrf_drv_rtc_config_t config;
    config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);
    APP_ERROR_CHECK(err_code);

    nrf_drv_rtc_enable(&rtc);
}

/**
 * @brief Initializes the logging module.
 */
void init_log(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_DEBUG("Logging initialized\r\n");
}

/**
 * @brief Main application entry point.
 */
int main(void)
{
#if LED_FUNCTIONALITY_ENABLED
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
#endif

    NRF_POWER->DCDCEN = 1;

    init_log();
    NRF_LOG_INFO("Main Inits.");

    lfclk_config();
    NRF_LOG_INFO("lfclk_config() done");

    ret_code_t err_code = i2c_init();
    NRF_LOG_INFO("i2c_init() returned: %d", err_code);

    // Enable sensor board power via GPIO P1.08
    //nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 8));
    //nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(1, 8));
    //NRF_LOG_INFO("Sensor board power enabled");
    //nrf_delay_ms(10); // Wait for AS7341 POR before first I2C access

    bool as7341_ok = test_as7341_connection();
    NRF_LOG_INFO("test_as7341_connection() returned: %d", as7341_ok);
    if (as7341_ok)
    {
        as7341_initialized = as7341_init_sensor();
        NRF_LOG_INFO("as7341_init_sensor() returned: %d", as7341_initialized);
    }
    else
    {
        NRF_LOG_WARNING("AS7341 sensor not detected. Continuing without sensor.");
    }

    rtc_config();
    NRF_LOG_INFO("rtc_config() done");

    while (1)
    {
        while (NRF_LOG_PROCESS() != NRF_SUCCESS);
        nrf_pwr_mgmt_run();
    }
}

// -----------------------------------------------------------------------------
// End of File
// -----------------------------------------------------------------------------
