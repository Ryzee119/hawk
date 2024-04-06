// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

#include "max9860.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// The MAX9860 responds to the slave address 0x20 for all write commands and 0x21 for all read operations.

#define MAX9860_I2C_ADDRESS 0x20

#define MAX9860_MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX9860_MAX(a, b) ((a) > (b) ? (a) : (b))
#define MAX9860_CLAMP(low, val, high) (MAX9860_MAX(low, MAX9860_MIN(val, high)))

bool max9860_init(max9860_device_t *handle, I2C_HandleTypeDef *hi2c, uint32_t mclk_freq) {
    memset(handle, 0, sizeof(max9860_device_t));
    handle->hi2c = hi2c;

    // Enable all interrupts
    handle->INTEN = MAX9860_ICLD | MAX9860_IULK | MAX9860_ISLD;

    // Setup internal clock
    uint8_t ps_divider = mclk_freq / 10000000;
    ps_divider = MAX9860_CLAMP(1, ps_divider, 3);
    handle->pclk_freq = mclk_freq / ps_divider;
    handle->SYSCLK = ((ps_divider << MAX9860_PSCLK_SHIFT) & MAX9860_PSCLK) | (MAX9860_FREQ_NORMAL);

    // Set LRCLK divider. Formula is in datasheet
    uint16_t n = (65536ULL * 96ULL * DEFAULT_SAMPLE_RATE) / handle->pclk_freq;
    handle->AUDIOCLKHIGH = (n >> 8) & MAX9860_NHI;
    handle->AUDIOCLKLOW = (n >> 0) & MAX9860_NLO;

    // We want standard unaltered I2S compatible mode as slave device
    handle->IFC1A = MAX9860_MASTER | MAX9860_DDLY;    // MAS = 0, WCI = 0, DBCI = 0, DDLY = 1 HIZ = 0 TDM = 0,
    handle->IFC1B = MAX9860_ADLY | MAX9860_BSEL_48X;  // ABCI = 0, ADLY = 1, ST = 0, BSEL = 0

    // Voice Filter
    handle->VOICEFLTR = 0x00;  // FIXME, might be worthwhile for samples rates 8kHz, 16khz or 48Khz

    /* Digital Level Control */
    {
        // Adjusts the digital audio level before being converted by the DAC DVA
        // DVA[7..0] 6 = 0 dB
        handle->DACATTN = (6 << MAX9860_DVA_SHIFT) & MAX9860_DVA;

        // ADCLEVEL
        // ADCRL [7..4] Adjusts the digital audio level before being converted by the DAC. 3 = 0 dB
        // ADCLL [3..0] Adjusts the digital audio level before being converted by the DAC. 3 = 0 dB,
        // 0xF = -12 dB
        handle->ADCLEVEL = ((3 << MAX9860_ADCLL_SHIFT) & MAX9860_ADCLL) | ((3 << MAX9860_ADCRL_SHIFT) & MAX9860_ADCRL);

        // DACGAIN
        // DVG[6..5] The gain set by DVG adds to the level set by DVA. 0 = 0dB, 3 = +18dB
        // DVST[4..0] Sets the level of left ADC output mixed into the DAC. 0 = Disabled, 0x01 = 0dB, 0x1F -60dB
        handle->DACGAIN = ((0 < MAX9860_DVG_SHIFT) & MAX9860_DVG) | ((0 < MAX9860_DVST_SHIFT) & MAX9860_DVST);
    }

    // MICGAIN
    // PAM[6..5] Left and Right Microphone Preamp Gain
    // PGAM[4..0] Left and Right Microphone PGA 0 = +20dB, 1 = +19 dB, >= 0x14 = 0dB. When AGC is enabled, the AGC
    // controller overrides these settings.
    handle->MICGAIN = ((1 << MAX9860_PAM_SHIFT) & MAX9860_PAM) | ((0x14 << MAX9860_PGAM_SHIFT) & MAX9860_PGAM);

    /* Microphone Automatic Gain Control and Noise Gate */
    {
        // MICADC
        // AGCSRC[7] AGC/Noise Gate Signal Source Select 0 = left only, 1 = right and left
        // AGCRLS[6..4] Time taken by the AGC circuit to increase the gain from minimum to maximum. 0 = 78ms to 7 = 10s
        // AGCATK[3..2] The time constant of the AGC gain reduction curve. 0 = 3ms to 3 = 200ms
        // AGCHLD[1..0] Time the AGC circuit waits before beginning to increase gain when a signal below the threshold
        // is detected. 0 = disabled, 1 = 50ms to 3 = 400ms
        handle->MICADC =
            ((0 << MAX9860_AGCSRC_SHIFT) & MAX9860_AGCSRC) | ((0 << MAX9860_AGCRLS_SHIFT) & MAX9860_AGCRLS) |
            ((0 << MAX9860_AGCATK_SHIFT) & MAX9860_AGCATK) | ((0 << MAX9860_AGCHLD_SHIFT) & MAX9860_AGCHLD);

        // NOISEGATE
        // ANTH[7..4] The signal level at which the noise gate begins reducing the gain 0 = disabled, 1 = -72dB, to 0xF
        // =-16dB AGCTH[3..0] The target output signal level for AGC 0 = -3dB, 0xF = -18dB
        handle->NOISEGATE =
            ((0xF << MAX9860_ANTH_SHIFT) & MAX9860_ANTH) | ((0xF << MAX9860_AGCTH_SHIFT) & MAX9860_AGCTH);
    }

    /* Power Management */
    // PWRMAN
    // SHDN[7] 0 = shutdown, 1 = on
    // DACEN[3] 0 = DAC off, 1 = on
    // ADCLEN[1] 0 = Left ADC off, 1 = on
    // ADCREN[0] 0 = Right ADC off, 1 = on (The left ADC must be enabled when using the right ADC)
    handle->PWRMAN = ((1 << MAX9860_DACEN_SHIFT) & MAX9860_DACEN) | ((1 << MAX9860_ADCLEN_SHIFT) & MAX9860_ADCLEN) |
                     ((0 << MAX9860_ADCREN_SHIFT) & MAX9860_ADCREN);

    // Clear interrupt status
    HAL_I2C_Mem_Read(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_INTRSTATUS, 1, &handle->INTRSTATUS, 1, 1000);
    HAL_I2C_Mem_Read(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_MICREADBACK, 1, &handle->MICREADBACK, 1, 1000);

    // Write all registers in one handshake starting from 0x00 (INTEN)
    return HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_INTEN, 1, &handle->INTEN,
                             MAX9860_PWRMAN - MAX9860_INTEN + 1, 1000) == HAL_OK;
}

bool max9860_set_volume(max9860_device_t *handle, int8_t gain) {
    uint8_t DVA, DVG = 0x00;

    // -90 dB to +21 dB in 1 dB steps
    gain = MAX9860_CLAMP(-90, gain, 21);

    // DBG can add +6,+12, or +18dB to the gain
    // DBG will take the reset of the gain from +3 to -90dB

    if (gain > 3 && gain <= 9) {
        /* 6db from DVG*/
        gain -= 6;
        DVG = 0x01;
    } else if (gain <= 15) {
        /* 12db from DVG*/
        gain -= 12;
        DVG = 0x02;
    } else if (gain > 15) {
        /* 18dB from DVG */
        gain -= 18;
        DVG = 0x03;
    }

    DVA = 186 + (2 * (-90 - gain));

    handle->DACATTN &= ~MAX9860_DVA;
    handle->DACATTN |= (DVA << MAX9860_DVA_SHIFT) & MAX9860_DVA;

    handle->DACGAIN &= ~MAX9860_DVG;
    handle->DACGAIN |= ((DVG < MAX9860_DVG_SHIFT) & MAX9860_DVG);

    return HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_DACATTN, 1, &handle->DACATTN, 1, 1000) ==
               HAL_OK &&
           HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_DACGAIN, 1, &handle->DACGAIN, 1, 1000) ==
               HAL_OK;
}

// Set the gain between 0 and 50 dB in 1dB steps. This uses a combination of Preamp gain (PAM) and Programmable gain
// (PGAM). MICROPHONE --> PAM_GAIN (0, 20, 30dB) --> PGAM_GAIN (Add 0 to 20db) --> ADC
bool max9860_set_mic_gain(max9860_device_t *handle, uint8_t gain) {
    uint8_t PAM, PGA;

    gain = MAX9860_CLAMP(0, gain, 50);

    if (gain < 20) {
        PAM = 0x01;
        PGA = 0x14 - gain;
    } else if (gain < 30) {
        gain -= 20;
        PAM = 0x02;
        PGA = 0x14 - gain;
    } else {
        gain -= 30;
        PAM = 0x03;
        PGA = 0x14 - gain;
    }

    handle->MICGAIN &= ~MAX9860_PAM;
    handle->MICGAIN &= ~MAX9860_PGAM;
    handle->MICGAIN |= ((PAM << MAX9860_PAM_SHIFT) & MAX9860_PAM) | ((PGA << MAX9860_PGAM_SHIFT) & MAX9860_PGAM);
    return HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_MICGAIN, 1, &handle->MICGAIN, 1, 1000) ==
           HAL_OK;
}

bool max9860_stop(max9860_device_t *handle) {
    handle->PWRMAN &= ~MAX9860_SHDN;
    return HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_PWRMAN, 1, &handle->PWRMAN, 1, 1000) == HAL_OK;
}

bool max9860_start(max9860_device_t *handle) {
    handle->PWRMAN |= MAX9860_SHDN;
    return HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_PWRMAN, 1, &handle->PWRMAN, 1, 1000) == HAL_OK;
}

bool max9860_mic_agc_enable(max9860_device_t *handle) {
    handle->MICADC &= ~MAX9860_AGCHLD;

    // 1 = 50ms before AGC kicks in on low level
    handle->MICADC |= (1 << MAX9860_AGCHLD_SHIFT) & MAX9860_AGCHLD;
    if (max9860_set_mic_gain(handle, 0)) {
        return HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_MICADC, 1, &handle->MICADC, 1, 1000) ==
               HAL_OK;
    }
    return false;
}

bool max9860_mic_agc_disable(max9860_device_t *handle, uint8_t manual_mic_gain) {
    handle->MICADC &= ~MAX9860_AGCHLD;

    if (HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_MICADC, 1, &handle->MICADC, 1, 1000) == HAL_OK) {
        max9860_set_mic_gain(handle, manual_mic_gain);
    }
    return false;
}

bool max9860_get_irq_status(max9860_device_t *handle) {
    return HAL_I2C_Mem_Read(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_INTRSTATUS, 1, &handle->INTRSTATUS, 1, 1000) ==
           HAL_OK;
}

bool max9860_set_sample_rate(max9860_device_t *handle, uint32_t sample_rate) {
    bool res = false;
    uint8_t back = handle->SYSCLK;

    uint16_t n = (65536ULL * 96ULL * sample_rate) / handle->pclk_freq;

    handle->SYSCLK &= MAX9860_PSCLK;
    handle->AUDIOCLKHIGH = (n >> 8) & MAX9860_NHI;
    handle->AUDIOCLKLOW = (n >> 0) & MAX9860_NLO;

    // Disable output
    HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_SYSCLK, 1, &handle->SYSCLK, 1, 1000);

    // Set new LRCLK divider in the fractional PLL
    res = HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_AUDIOCLKHIGH, 1, &handle->AUDIOCLKHIGH, 2,
                            1000) == HAL_OK;

    // Restore to old state
    handle->SYSCLK = back;
    HAL_I2C_Mem_Write(handle->hi2c, MAX9860_I2C_ADDRESS, MAX9860_SYSCLK, 1, &handle->SYSCLK, 1, 1000);

    return res;
}