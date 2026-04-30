/*
 *  @file as7341_defines.h
 *  @brief Register definitions and enums for the AS7341 11-Channel Spectral Sensor
 *
 *  Adapted for nRF52 SDK integration from the original Adafruit Arduino library.
 *  All Arduino dependencies removed. For use with nRF52833 and compatible MCUs.
 *
 *  Original Author: Bryan Siepert for Adafruit Industries
 *  License: BSD (see license.txt)
 */

#ifndef AS7341_DEFINES_H
#define AS7341_DEFINES_H

/* ---------------------------------------------------------------------------
 * I2C address
 * ---------------------------------------------------------------------------*/

#define AS7341_I2CADDR_DEFAULT  0x39  ///< AS7341 I2C address

/* ---------------------------------------------------------------------------
 * Chip identification
 * ---------------------------------------------------------------------------*/

#define AS7341_CHIP_ID          0x09  ///< Expected value in ID register bits [7:2]
#define AS7341_WHOAMI           0x92  ///< Chip ID register address (same as AS7341_ID)

/* ---------------------------------------------------------------------------
 * Bank 1 registers (require REG_BANK=1 in CFG0 register 0xA9)
 * DS §10: "reserved/unlisted bits must not be changed at any time"
 * ---------------------------------------------------------------------------*/

#define AS7341_ASTATUS          0x60  ///< ALS status (bank 1)
#define AS7341_CH0_DATA_L_B1    0x61  ///< CH0 data low  (bank 1)
#define AS7341_CH0_DATA_H_B1    0x62  ///< CH0 data high (bank 1)
#define AS7341_ITIME_L          0x63  ///< Integration time low  (bank 1, SYND mode)
#define AS7341_ITIME_M          0x64  ///< Integration time mid  (bank 1, SYND mode)
#define AS7341_ITIME_H          0x65  ///< Integration time high (bank 1, SYND mode)
#define AS7341_CONFIG           0x70  ///< LED control / INT_MODE select (bank 1)
#define AS7341_STAT             0x71  ///< Trigger status (bank 1)
#define AS7341_EDGE             0x72  ///< SYNC edge count (bank 1, SYND mode)
#define AS7341_GPIO             0x73  ///< GPIO / INT photo-diode connect (bank 1)
#define AS7341_LED              0x74  ///< LED enable and current limit (bank 1)

/* ---------------------------------------------------------------------------
 * Main register bank (always accessible)
 * ---------------------------------------------------------------------------*/

#define AS7341_ENABLE           0x80  ///< Main enable: PON, SP_EN, WEN, SMUXEN, FDEN
#define AS7341_ATIME            0x81  ///< ADC integration step count
#define AS7341_WTIME            0x83  ///< Wait time between measurements
#define AS7341_SP_LOW_TH_L      0x84  ///< Spectral low threshold low byte
#define AS7341_SP_LOW_TH_H      0x85  ///< Spectral low threshold high byte
#define AS7341_SP_HIGH_TH_L     0x86  ///< Spectral high threshold low byte
#define AS7341_SP_HIGH_TH_H     0x87  ///< Spectral high threshold high byte
#define AS7341_AUXID            0x90  ///< Auxiliary ID
#define AS7341_REVID            0x91  ///< Revision ID
#define AS7341_ID               0x92  ///< Chip ID register — bits [7:2] = chip ID
#define AS7341_STATUS           0x93  ///< Interrupt status (ASAT, AINT, FINT, CINT, SINT)
#define AS7341_ASTATUS_         0x94  ///< ALS status (bank 0 mirror)
#define AS7341_CH0_DATA_L       0x95  ///< ADC channel 0 data low byte
#define AS7341_CH0_DATA_H       0x96  ///< ADC channel 0 data high byte
#define AS7341_CH1_DATA_L       0x97  ///< ADC channel 1 data low byte
#define AS7341_CH1_DATA_H       0x98  ///< ADC channel 1 data high byte
#define AS7341_CH2_DATA_L       0x99  ///< ADC channel 2 data low byte
#define AS7341_CH2_DATA_H       0x9A  ///< ADC channel 2 data high byte
#define AS7341_CH3_DATA_L       0x9B  ///< ADC channel 3 data low byte
#define AS7341_CH3_DATA_H       0x9C  ///< ADC channel 3 data high byte
#define AS7341_CH4_DATA_L       0x9D  ///< ADC channel 4 data low byte
#define AS7341_CH4_DATA_H       0x9E  ///< ADC channel 4 data high byte
#define AS7341_CH5_DATA_L       0x9F  ///< ADC channel 5 data low byte
#define AS7341_CH5_DATA_H       0xA0  ///< ADC channel 5 data high byte
#define AS7341_STATUS2          0xA3  ///< Measurement status: saturation, AVALID
#define AS7341_STATUS3          0xA4  ///< Spectral interrupt source
#define AS7341_STATUS5          0xA6  ///< Flicker detection interrupt status
#define AS7341_STATUS6          0xA7  ///< SAI_ACTIVE, INT_BUSY, SP_TRIG, FD_TRIG
#define AS7341_CFG0             0xA9  ///< Low power mode, register bank select
#define AS7341_CFG1             0xAA  ///< ADC gain (AGAIN[4:0])
#define AS7341_CFG3             0xAC  ///< Sleep after interrupt (SAI)
#define AS7341_CFG6             0xAF  ///< SMUX command register (SMUX_CMD[4:3])
#define AS7341_CFG8             0xB1  ///< FIFO threshold, FD_AGC, SP_AGC
#define AS7341_CFG9             0xB2  ///< SIEN_FD, SIEN_SMUX interrupt enables
#define AS7341_CFG10            0xB3  ///< AGC_H, AGC_L, FD_PERS
#define AS7341_CFG12            0xB5  ///< Threshold channel, SP_TH_CH
#define AS7341_PERS             0xBD  ///< Interrupt persistence (APERS)
#define AS7341_GPIO2            0xBE  ///< GPIO direction, polarity, I/O
#define AS7341_ASTEP_L          0xCA  ///< Integration step size low byte
#define AS7341_ASTEP_H          0xCB  ///< Integration step size high byte
#define AS7341_AGC_GAIN_MAX     0xCF  ///< AGC maximum gain
#define AS7341_AZ_CONFIG        0xD6  ///< Auto-zero configuration
#define AS7341_FD_CFG0          0xD7  ///< Flicker detection config
#define AS7341_FD_TIME1         0xD8  ///< Flicker detection integration time low
#define AS7341_FD_TIME2         0xDA  ///< Flicker detection gain / time high nibble
#define AS7341_FD_STATUS        0xDB  ///< Flicker detection status
#define AS7341_INTENAB          0xF9  ///< Interrupt enable register
#define AS7341_CONTROL          0xFA  ///< Auto-zero, FIFO clear, clear SAI active
#define AS7341_FIFO_MAP         0xFC  ///< FIFO channel map
#define AS7341_FIFO_LVL         0xFD  ///< FIFO fill level
#define AS7341_FDATA_L          0xFE  ///< FIFO data low byte
#define AS7341_FDATA_H          0xFF  ///< FIFO data high byte

/* ---------------------------------------------------------------------------
 * ENABLE register (0x80) bit masks — DS Figure 33
 * ---------------------------------------------------------------------------*/

#define AS7341_ENABLE_PON       0x01  ///< Power ON — activates internal oscillator
#define AS7341_ENABLE_SP_EN     0x02  ///< Spectral measurement enable
#define AS7341_ENABLE_WEN       0x08  ///< Wait enable between measurements
#define AS7341_ENABLE_SMUXEN    0x10  ///< SMUX enable (self-clearing when done)
#define AS7341_ENABLE_FDEN      0x40  ///< Flicker detection enable

/* ---------------------------------------------------------------------------
 * STATUS2 register (0xA3) bit masks — DS Figure 32
 * ---------------------------------------------------------------------------*/

#define AS7341_STATUS2_AVALID   0x40  ///< Spectral measurement valid (bit 6)
#define AS7341_STATUS2_ASAT_DIG 0x10  ///< Digital saturation (bit 4)
#define AS7341_STATUS2_ASAT_ANA 0x08  ///< Analog saturation  (bit 3)

/* ---------------------------------------------------------------------------
 * CFG6 register (0xAF) SMUX_CMD field — DS Figure 32
 * ---------------------------------------------------------------------------*/

#define AS7341_CFG6_SMUX_CMD_MASK   0x18  ///< SMUX_CMD field mask (bits [4:3])
#define AS7341_CFG6_SMUX_CMD_SHIFT  3     ///< SMUX_CMD field LSB position

/* ---------------------------------------------------------------------------
 * Spectral interrupt threshold masks (STATUS3)
 * ---------------------------------------------------------------------------*/

#define AS7341_SPECTRAL_INT_HIGH_MSK  0x20  ///< High threshold interrupt flag
#define AS7341_SPECTRAL_INT_LOW_MSK   0x10  ///< Low threshold interrupt flag

/* ---------------------------------------------------------------------------
 * Enumerations
 * ---------------------------------------------------------------------------*/

/**
 * @brief Gain multipliers — values match CFG1 AGAIN[4:0] register encoding (DS Fig 49).
 */
typedef enum {
    AS7341_GAIN_0_5X  = 0,
    AS7341_GAIN_1X    = 1,
    AS7341_GAIN_2X    = 2,
    AS7341_GAIN_4X    = 3,
    AS7341_GAIN_8X    = 4,
    AS7341_GAIN_16X   = 5,
    AS7341_GAIN_32X   = 6,
    AS7341_GAIN_64X   = 7,
    AS7341_GAIN_128X  = 8,
    AS7341_GAIN_256X  = 9,
    AS7341_GAIN_512X  = 10,
} as7341_gain_t;

/**
 * @brief SMUX configuration commands — CFG6 SMUX_CMD[4:3] field (DS Fig 32).
 */
typedef enum {
    AS7341_SMUX_CMD_ROM_RESET = 0, ///< ROM code initialisation of SMUX
    AS7341_SMUX_CMD_READ      = 1, ///< Read SMUX configuration from SMUX chain to RAM
    AS7341_SMUX_CMD_WRITE     = 2, ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_cmd_t;

/**
 * @brief Internal ADC channel indices (CH0–CH5).
 */
typedef enum {
    AS7341_ADC_CHANNEL_0 = 0,
    AS7341_ADC_CHANNEL_1 = 1,
    AS7341_ADC_CHANNEL_2 = 2,
    AS7341_ADC_CHANNEL_3 = 3,
    AS7341_ADC_CHANNEL_4 = 4,
    AS7341_ADC_CHANNEL_5 = 5,
} as7341_adc_channel_t;

/**
 * @brief Spectral channel indices in the readings buffer returned by
 *        as7341_read_all_channels().
 *
 *  Pass 1 (low):  [0]=F1  [1]=F2  [2]=F3  [3]=F4  [4]=Clear  [5]=NIR
 *  Pass 2 (high): [6]=F5  [7]=F6  [8]=F7  [9]=F8  [10]=Clear [11]=NIR
 */
typedef enum {
    AS7341_CHANNEL_415nm_F1 = 0,
    AS7341_CHANNEL_445nm_F2 = 1,
    AS7341_CHANNEL_480nm_F3 = 2,
    AS7341_CHANNEL_515nm_F4 = 3,
    AS7341_CHANNEL_CLEAR_0  = 4,
    AS7341_CHANNEL_NIR_0    = 5,
    AS7341_CHANNEL_555nm_F5 = 6,
    AS7341_CHANNEL_590nm_F6 = 7,
    AS7341_CHANNEL_630nm_F7 = 8,
    AS7341_CHANNEL_680nm_F8 = 9,
    AS7341_CHANNEL_CLEAR    = 10,
    AS7341_CHANNEL_NIR      = 11,
} as7341_color_channel_t;

/**
 * @brief Interrupt persistence — measurement cycles outside threshold before interrupt.
 */
typedef enum {
    AS7341_INT_COUNT_ALL = 0,
    AS7341_INT_COUNT_1   = 1,
    AS7341_INT_COUNT_2   = 2,
    AS7341_INT_COUNT_3   = 3,
    AS7341_INT_COUNT_5   = 4,
    AS7341_INT_COUNT_10  = 5,
    AS7341_INT_COUNT_15  = 6,
    AS7341_INT_COUNT_20  = 7,
    AS7341_INT_COUNT_25  = 8,
    AS7341_INT_COUNT_30  = 9,
    AS7341_INT_COUNT_35  = 10,
    AS7341_INT_COUNT_40  = 11,
    AS7341_INT_COUNT_45  = 12,
    AS7341_INT_COUNT_50  = 13,
    AS7341_INT_COUNT_55  = 14,
    AS7341_INT_COUNT_60  = 15,
} as7341_int_cycle_count_t;

/**
 * @brief GPIO pin direction.
 */
typedef enum {
    AS7341_GPIO_OUTPUT = 0, ///< Open-drain output
    AS7341_GPIO_INPUT  = 1, ///< High-impedance input
} as7341_gpio_dir_t;

/**
 * @brief Async read wait states.
 */
typedef enum {
    AS7341_WAITING_START = 0,
    AS7341_WAITING_LOW   = 1,
    AS7341_WAITING_HIGH  = 2,
    AS7341_WAITING_DONE  = 3,
} as7341_waiting_t;

#endif /* AS7341_DEFINES_H */
