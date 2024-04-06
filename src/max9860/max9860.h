// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "main.h"

#define MAX9860_I2C_ADDRESS 0x20

typedef enum {
    MAX9860_INTRSTATUS = 0x00,
    MAX9860_MICREADBACK = 0x01,
    MAX9860_INTEN = 0x02,
    MAX9860_SYSCLK = 0x03,
    MAX9860_AUDIOCLKHIGH = 0x04,
    MAX9860_AUDIOCLKLOW = 0x05,
    MAX9860_IFC1A = 0x06,
    MAX9860_IFC1B = 0x07,
    MAX9860_VOICEFLTR = 0x08,
    MAX9860_DACATTN = 0x09,
    MAX9860_ADCLEVEL = 0x0a,
    MAX9860_DACGAIN = 0x0b,
    MAX9860_MICGAIN = 0x0c,
    MAX9860_RESERVED = 0x0d,
    MAX9860_MICADC = 0x0e,
    MAX9860_NOISEGATE = 0x0f,
    MAX9860_PWRMAN = 0x10,
    MAX9860_REVISION = 0xff,
} max9860_registers_t;

typedef struct {
    uint8_t INTRSTATUS;
    uint8_t MICREADBACK;
    uint8_t INTEN;
    uint8_t SYSCLK;
    uint8_t AUDIOCLKHIGH;
    uint8_t AUDIOCLKLOW;
    uint8_t IFC1A;
    uint8_t IFC1B;
    uint8_t VOICEFLTR;
    uint8_t DACATTN;
    uint8_t ADCLEVEL;
    uint8_t DACGAIN;
    uint8_t MICGAIN;
    uint8_t RESERVED;
    uint8_t MICADC;
    uint8_t NOISEGATE;
    uint8_t PWRMAN;
    uint8_t REVISION;
    I2C_HandleTypeDef *hi2c;
    uint32_t pclk_freq;
    volatile bool has_irq;

} max9860_device_t;

/* INTRSTATUS */
#define MAX9860_CLD 0x80
#define MAX9860_SLD 0x40
#define MAX9860_ULK 0x20

/* MICREADBACK */
#define MAX9860_NG 0xe0
#define MAX9860_AGC 0x1f

/* INTEN */
#define MAX9860_ICLD 0x80
#define MAX9860_ISLD 0x40
#define MAX9860_IULK 0x20

/* SYSCLK */
#define MAX9860_PSCLK 0x30
#define MAX9860_PSCLK_OFF 0x00
#define MAX9860_PSCLK_SHIFT 4
#define MAX9860_FREQ 0x06
#define MAX9860_FREQ_NORMAL 0x00
#define MAX9860_FREQ_12MHZ 0x02
#define MAX9860_FREQ_13MHZ 0x04
#define MAX9860_FREQ_19_2MHZ 0x06
#define MAX9860_16KHZ 0x01

/* AUDIOCLKHIGH */
#define MAX9860_PLL 0x80
#define MAX9860_NHI 0x7f

/* AUDIOCLKLOW */
#define MAX9860_NLO 0xff

/* IFC1A */
#define MAX9860_MASTER 0x80
#define MAX9860_WCI 0x40
#define MAX9860_DBCI 0x20
#define MAX9860_DDLY 0x10
#define MAX9860_HIZ 0x08
#define MAX9860_TDM 0x04

/* IFC1B */
#define MAX9860_ABCI 0x20
#define MAX9860_ADLY 0x10
#define MAX9860_ST 0x08
#define MAX9860_BSEL 0x07
#define MAX9860_BSEL_OFF 0x00
#define MAX9860_BSEL_64X 0x01
#define MAX9860_BSEL_48X 0x02
#define MAX9860_BSEL_PCLK_2 0x04
#define MAX9860_BSEL_PCLK_4 0x05
#define MAX9860_BSEL_PCLK_8 0x06
#define MAX9860_BSEL_PCLK_16 0x07

/* VOICEFLTR */
#define MAX9860_AVFLT 0xf0
#define MAX9860_AVFLT_SHIFT 4
#define MAX9860_AVFLT_COUNT 6
#define MAX9860_DVFLT 0x0f
#define MAX9860_DVFLT_SHIFT 0
#define MAX9860_DVFLT_COUNT 6

/* DACATTN */
#define MAX9860_DVA 0xfe
#define MAX9860_DVA_SHIFT 1
#define MAX9860_DVA_MUTE 0x5e

/* ADCLEVEL */
#define MAX9860_ADCRL 0xf0
#define MAX9860_ADCRL_SHIFT 4
#define MAX9860_ADCLL 0x0f
#define MAX9860_ADCLL_SHIFT 0
#define MAX9860_ADCxL_MIN 15

/* DACGAIN */
#define MAX9860_DVG 0x60
#define MAX9860_DVG_SHIFT 5
#define MAX9860_DVG_MAX 3
#define MAX9860_DVST 0x1f
#define MAX9860_DVST_SHIFT 0
#define MAX9860_DVST_MIN 31

/* MICGAIN */
#define MAX9860_PAM 0x60
#define MAX9860_PAM_SHIFT 5
#define MAX9860_PAM_MAX 3
#define MAX9860_PGAM 0x1f
#define MAX9860_PGAM_SHIFT 0
#define MAX9860_PGAM_MIN 20

/* MICADC */
#define MAX9860_AGCSRC 0x80
#define MAX9860_AGCSRC_SHIFT 7
#define MAX9860_AGCSRC_COUNT 2
#define MAX9860_AGCRLS 0x70
#define MAX9860_AGCRLS_SHIFT 4
#define MAX9860_AGCRLS_COUNT 8
#define MAX9860_AGCATK 0x0c
#define MAX9860_AGCATK_SHIFT 2
#define MAX9860_AGCATK_COUNT 4
#define MAX9860_AGCHLD 0x03
#define MAX9860_AGCHLD_OFF 0x00
#define MAX9860_AGCHLD_SHIFT 0
#define MAX9860_AGCHLD_COUNT 4

/* NOISEGATE */
#define MAX9860_ANTH 0xf0
#define MAX9860_ANTH_SHIFT 4
#define MAX9860_ANTH_MAX 15
#define MAX9860_AGCTH 0x0f
#define MAX9860_AGCTH_SHIFT 0
#define MAX9860_AGCTH_MIN 15

/* PWRMAN */
#define MAX9860_SHDN 0x80
#define MAX9860_DACEN 0x08
#define MAX9860_DACEN_SHIFT 3
#define MAX9860_ADCLEN 0x02
#define MAX9860_ADCLEN_SHIFT 1
#define MAX9860_ADCREN 0x01
#define MAX9860_ADCREN_SHIFT 0

/**
 * @brief Initializes the MAX9860 hardware.
 *
 * This function initializes the MAX9860 codec device with the given I2C handle. It configures the device
 * to a known state and prepares it for operation.
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @param hi2c Pointer to the I2C handle structure used for communication.
 * @param mclk_freq The frequency of the MCLK signal in Hz. This is used to configure the codec's PLL.
 * @return bool True if initialization was successful, False otherwise.
 */
bool max9860_init(max9860_device_t *handle, I2C_HandleTypeDef *hi2c, uint32_t mclk_freq);

/**
 * @brief Sets the speaker output gain.
 *
 * Adjusts the gain of the speaker output from -90 dB to +21 dB in 1 dB steps. This allows for precise control
 * over the output volume of the speaker connected to the MAX9860.
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @param gain Desired gain in dB. Range: -90 to +21.
 * @return bool True if the gain was successfully set, False otherwise.
 */
bool max9860_set_volume(max9860_device_t *handle, int8_t gain);

/**
 * @brief Sets the microphone input gain.
 *
 * Adjusts the gain of the microphone input from 0 to 50 dB in 1 dB steps. This function is useful for calibrating
 * the input sensitivity according to the requirements of the application.
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @param gain Desired gain in dB. Range: 0 to 50.
 * @return bool True if the mic gain was successfully set, False otherwise.
 */
bool max9860_set_mic_gain(max9860_device_t *handle, uint8_t gain);

/**
 * @brief Powers down the MAX9860.
 *
 * Puts the MAX9860 into a low-power shutdown mode. In this mode, all register settings are preserved,
 * and the I2C interface remains active, allowing for the device to be reconfigured without a full initialization.
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @return bool True if the device was successfully powered down, False otherwise.
 */
bool max9860_stop(max9860_device_t *handle);

/**
 * @brief Powers up the MAX9860.
 *
 * Exits the low-power shutdown mode and powers up the MAX9860. This function should be called after max9860_init()
 * or max9860_stop() to begin or resume operation of the device.
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @return bool True if the device was successfully powered up, False otherwise.
 */
bool max9860_start(max9860_device_t *handle);

/**
 * @brief Retrieves the current IRQ status. You can check handle->has_irq to see if an IRQ is pending
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @return bool True if successful, false otherwise
 */
bool max9860_get_irq_status(max9860_device_t *handle);

/**
 * @brief Enables the internal auto gain control algorithm for the microphone
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @return bool True if successful, false otherwise
 */
bool max9860_mic_agc_enable(max9860_device_t *handle);

/**
 * @brief Disables the internal auto gain control algorithm for the microphone and sets a manual gain
 *
 * @param handle Pointer to the MAX9860 device structure.
 * @param manual_mic_gain The gain to set the microphone to after disabling AGC (0 to 50dB)
 * @return bool True if successful, false otherwise
 */
bool max9860_mic_agc_disable(max9860_device_t *handle, uint8_t manual_mic_gain);

/**
 * @brief Set's the LRCLK Divider from PCLK (12MHz) using the codecs fractional PLL
 *
 * @param handle Pointer to the MAX9860 device structure
 * @param sample_rate The sample rate in Hz
 * @return bool True if successful, false otherwise
 */
bool max9860_set_sample_rate(max9860_device_t *handle, uint32_t sample_rate);
#ifdef __cplusplus
}
#endif