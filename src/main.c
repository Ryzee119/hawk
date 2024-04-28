// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

// i2S Audio codec to USB bridge.
// USB is coded to simulate an Xbox Live Communicator for the Original Xbox
// I2S is generated with STMs SAI interface as a slave device. The audio codex is a MAX9860 and is the master device.
// The STM provides a master clock to the MAX9860 using MCO and the MAX9860 provides the bit clock and frame sync back
// to the STM. The master clock is provided to the MAX9860 by STM's HSI48 which is automatically adjusted to match the
// USB clock using the clock recovery system. This eliminates the need for a crystal or other clock source and prevents
// drift between the USB clock and onboard clocks. All the I2s audio processing is circular DMA/interrupt driven and is
// pushed into a FIFO buffer for microphone data, and read from a FIFO buffer for speaker data. The USB interface reads
// from the FIFO for microphone data and writes to the FIFO for speaker data. This is all interrupt driven.

#include "main.h"
#include "eeprom_emulation/ee.h"
#include "max9860/max9860.h"
#include "usbd/usb_device.h"
#include "fifo.h"

#define RAM_FUNC __attribute__((section(".RamFunc")))
#define MCO_FREQ 12000000UL  // 48Mhz / 4 See SystemClock_Config

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SAI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void set_led_brightness(uint8_t percent);
static void start_new_audio_stream(uint32_t sample_rate);

// Peripheral handles
I2C_HandleTypeDef hi2c1;
SAI_HandleTypeDef hsai_BlockA1; // Speaker
SAI_HandleTypeDef hsai_BlockB1; // Microphone
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

// Codec
static max9860_device_t max9860;
static uint32_t pending_sample_rate_change = 0;
static uint8_t pending_agc_change = 0;
static uint8_t microphone_muted = 0;

// Non volatile storage
#define HAWK_MAGIC (0x4841574B)
typedef struct {
    uint32_t magic;
    int8_t volume;
    uint8_t mic_gain;
    uint8_t reserved[2];
} hawk_storage_t;
hawk_storage_t ee;
uint32_t ee_dirty = 0;

// Data fifos for linking USB and SAI I2S
#define FIFO_BUFF_SZ (48 * 8)
static fifo_t fifo_speaker;
static fifo_t fifo_mic;
static uint8_t fifo_speaker_buffer[FIFO_BUFF_SZ];
static uint8_t fifo_mic_buffer[FIFO_BUFF_SZ];

// UART FIFO
static uint8_t uart_fifo[256];
static uint32_t uart_fifo_head = 0, uart_fifo_tail = 0;
void io_puts(const char *s, void *ctx) {
    while (*s) {
        uart_fifo[uart_fifo_tail] = *s++;
        uart_fifo_tail = (uart_fifo_tail + 1) % sizeof(uart_fifo);
    }
}

// Circular DMA buffers for SAI TX and RX
#define SAI_DMA_BUFFER_BYTES (FIFO_BUFF_SZ)
static uint8_t tx_dma_buffer[SAI_DMA_BUFFER_BYTES] __attribute__((aligned(4)));
static uint8_t rx_dma_buffer[SAI_DMA_BUFFER_BYTES] __attribute__((aligned(4)));

// LED
static uint32_t led_rate = 0;
static uint8_t led_brightness = 0;

void (*SysMemBootJump)(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_SAI1_Init();
    MX_TIM2_Init();
    MX_USB_Device_Init();

    io_puts("HAWK!\n", NULL);

    // Emulated EEPROM
    EE_Init(&ee, sizeof(ee));
    EE_Read();
    if (ee.magic != HAWK_MAGIC) {
        io_puts("EEPROM RESET\n", NULL);
        memset(&ee, 0, sizeof(ee));
        ee.magic = HAWK_MAGIC;
        ee.volume = DEFAULT_SPEAKER_GAIN;
        ee.mic_gain = DEFAULT_MICROPHONE_GAIN;
        EE_Write(true);
    }

    if (max9860_init(&max9860, &hi2c1, MCO_FREQ) == false) {
        io_puts("MAX9860 INIT FAILED\n", NULL);
        while (1) {
            set_led_brightness(100);
            HAL_Delay(100);
            set_led_brightness(0);
            HAL_Delay(100);
        }
    }

    // Setup audio codec
    max9860_set_volume(&max9860, 20);
    max9860_mic_agc_enable(&max9860);
    max9860_start(&max9860);
    led_brightness = (((ee.volume - MIN_SPEAKER_GAIN) * 100) / (MAX_SPEAKER_GAIN - MIN_SPEAKER_GAIN));
    led_rate = 0;

    // Prepare FIFOs
    fifo_init(&fifo_speaker, fifo_speaker_buffer, sizeof(fifo_speaker_buffer));
    fifo_init(&fifo_mic, fifo_mic_buffer, sizeof(fifo_mic_buffer));

    // Start the I2s Interface DMA circular buffer.
    start_new_audio_stream(DEFAULT_SAMPLE_RATE);

    while (1) {
        uint32_t current_tick = HAL_GetTick();

        // Process and clear max9860 IRQs
        if (max9860.has_irq) {
            max9860.has_irq = false;
            // Read and clear the status
            if (max9860_get_irq_status(&max9860)) {
                io_puts("IRQ: ", NULL);
                io_puts((max9860.INTRSTATUS & MAX9860_CLD) ? "CLD " : "", NULL);
                io_puts((max9860.INTRSTATUS & MAX9860_SLD) ? "SLD " : "", NULL);
                io_puts((max9860.INTRSTATUS & MAX9860_ULK) ? "ULK " : "", NULL);
                io_puts("\n", NULL);
            }
        }

        // Handle a sample rate change
        if (pending_sample_rate_change) {
            io_puts("SR: ", NULL);
            io_puts((pending_sample_rate_change == 8000)    ? "8k"
                    : (pending_sample_rate_change == 11025) ? "11k"
                    : (pending_sample_rate_change == 16000) ? "16k"
                    : (pending_sample_rate_change == 22050) ? "22k"
                    : (pending_sample_rate_change == 24000) ? "24k"
                                                            : "???",
                    NULL);
            io_puts("\n", NULL);
            start_new_audio_stream(pending_sample_rate_change);
            pending_sample_rate_change = 0;
        }

        // Handle auto gain control toggle
        if (pending_agc_change & 0x80) {
            uint8_t agc = pending_agc_change & 0x7F;
            pending_agc_change = 0;
            io_puts("AGC: ", NULL);
            io_puts(agc ? "ON\n" : "OFF\n", NULL);
            if (agc) {
                max9860_mic_agc_enable(&max9860);
            } else {
                max9860_mic_agc_disable(&max9860, DEFAULT_MICROPHONE_GAIN);
                max9860_set_mic_gain(&max9860, ee.mic_gain);
            }
            pending_agc_change = 0;
        }

        // Flush UART fifo
        if (uart_fifo_head != uart_fifo_tail) {
            uint32_t len = (uart_fifo_head < uart_fifo_tail) ? (uart_fifo_tail - uart_fifo_head)
                                                             : (sizeof(uart_fifo) - uart_fifo_head);
            HAL_UART_Transmit(&huart2, &uart_fifo[uart_fifo_head], len, 0xFFFF);
            uart_fifo_head = (uart_fifo_head + len) % sizeof(uart_fifo);
        }

        // Prevent thrashing emulated eeprom. Only write if dirty and some period has passed
        if (ee_dirty && (HAL_GetTick() - ee_dirty) > 10000) {
            EE_Write(true);
            ee_dirty = 0;
            io_puts("EEPROM WRITTEN\n", NULL);
        }

        // Handle flashing of the LED
        static uint32_t led_tick = 0;
        static uint32_t tgl = 0;
        uint32_t led_held = current_tick - led_tick;
        if (led_rate) {
            if (led_held > led_rate) {
                led_tick = current_tick;
                set_led_brightness((tgl ^= 1) ? led_brightness : 0);
            }
        } else {
            set_led_brightness(led_brightness);
        }

        // The button has 4 modes. Normal, Mute, Mic Gain, and EEPROM Reset
        // Normal mode adjusts the volume. The LED brightness is set to the volume level.
        // Mute does not transfer any microphone information. This is indicated a slow flash of the LED.
        // Mic Gain mode makes the button adjust the microphone gain. This is indicated by a medium flash of the LED.
        // EEPROM Reset mode resets the EEPROM to defaults this is indicated by a fast flash of the LED.
        typedef enum { NORMAL_MODE, MUTE_MODE, MIC_GAIN_MODE, EEPROM_RESET_MODE } button_mode_t;
        static button_mode_t button_mode = NORMAL_MODE;
        static uint8_t last_button_state = 0;
        static uint32_t button_pressed_tick = 0;

        uint8_t button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
        if (button_state && button_state != last_button_state) {
            button_pressed_tick = current_tick;
        }

        if (!button_state && button_state != last_button_state) {
            if (current_tick - button_pressed_tick > 8000) {
                io_puts("EEPROM RESET\n", NULL);
                button_mode = EEPROM_RESET_MODE;
                led_rate = 100;
                led_brightness = 100;
                memset(&ee, 0, sizeof(ee));
                ee.magic = HAWK_MAGIC;
                ee.volume = DEFAULT_SPEAKER_GAIN;
                ee.mic_gain = DEFAULT_MICROPHONE_GAIN;
                EE_Write(true);

            } else if (current_tick - button_pressed_tick > 2000) {
                io_puts("MIC GAIN\n", NULL);
                button_mode = (button_mode == MIC_GAIN_MODE) ? NORMAL_MODE : MIC_GAIN_MODE;
                microphone_muted = 0;
                led_rate = 250;
                led_brightness =
                    (((ee.mic_gain - MIN_MICROPHONE_GAIN) * 100) / (MAX_MICROPHONE_GAIN - MIN_MICROPHONE_GAIN));

            } else if (current_tick - button_pressed_tick > 500) {
                if (button_mode != MIC_GAIN_MODE) {
                    io_puts("MUTE\n", NULL);
                    button_mode = MUTE_MODE;
                    led_rate = 1000;
                    led_brightness = 50;
                    microphone_muted = 1;
                } else {
                    button_mode = NORMAL_MODE;
                    led_rate = 0;
                    led_brightness = (((ee.volume - MIN_SPEAKER_GAIN) * 100) / (MAX_SPEAKER_GAIN - MIN_SPEAKER_GAIN));
                }

            } else if (current_tick - button_pressed_tick > 50) {
                if (button_mode == NORMAL_MODE) {
                    ee.volume += SPEAKER_INCREMEMENT;
                    // When it reaches max, one more press is no change, 2nd press resets to zero
                    if (ee.volume > (MAX_SPEAKER_GAIN + SPEAKER_INCREMEMENT)) {
                        ee.volume = MIN_SPEAKER_GAIN;
                    }
                    max9860_set_volume(&max9860, ee.volume);
                    led_brightness = (((ee.volume - MIN_SPEAKER_GAIN) * 100) / (MAX_SPEAKER_GAIN - MIN_SPEAKER_GAIN));
                    ee_dirty = HAL_GetTick();

                } else if (button_mode == MIC_GAIN_MODE) {
                    ee.mic_gain += MICROPHONE_INCREMEMENT;
                    // When it reaches max, one more press is no change, 2nd press resets to zero
                    if (ee.mic_gain > (MAX_MICROPHONE_GAIN + MICROPHONE_INCREMEMENT)) {
                        ee.mic_gain = MIN_MICROPHONE_GAIN;
                    }
                    max9860_set_mic_gain(&max9860, ee.mic_gain);
                    led_brightness =
                        (((ee.mic_gain - MIN_MICROPHONE_GAIN) * 100) / (MAX_MICROPHONE_GAIN - MIN_MICROPHONE_GAIN));
                    ee_dirty = HAL_GetTick();

                } else {
                    microphone_muted = 0;
                    led_rate = 0;
                    button_mode = NORMAL_MODE;
                }
            }
        }
        last_button_state = button_state;
    }
}

static inline void set_led_brightness(uint8_t percent) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, percent);
}

void start_new_audio_stream(uint32_t sample_rate) {
    HAL_SAI_DMAStop(&hsai_BlockA1);
    HAL_SAI_DMAStop(&hsai_BlockB1);

    max9860_set_sample_rate(&max9860, sample_rate);

    memset(tx_dma_buffer, 0, SAI_DMA_BUFFER_BYTES);
    memset(rx_dma_buffer, 0, SAI_DMA_BUFFER_BYTES);

    HAL_SAI_Transmit_DMA(&hsai_BlockA1, tx_dma_buffer, SAI_DMA_BUFFER_BYTES / 2);
    HAL_SAI_Receive_DMA(&hsai_BlockB1, rx_dma_buffer, SAI_DMA_BUFFER_BYTES / 2);
}

// SAI uses a circular DMA. This has two call backs. One when half the buffer is complete and another when fully
// complete We use these callbacks to refill the buffer with data from the FIFO or push data to the FIFO
RAM_FUNC void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    fifo_read(&fifo_speaker, &tx_dma_buffer[0], SAI_DMA_BUFFER_BYTES / 2);
}

RAM_FUNC void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    fifo_read(&fifo_speaker, &tx_dma_buffer[SAI_DMA_BUFFER_BYTES / 2], SAI_DMA_BUFFER_BYTES / 2);
}

RAM_FUNC void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (!microphone_muted)
        fifo_write(&fifo_mic, &rx_dma_buffer[0], SAI_DMA_BUFFER_BYTES / 2);
}

RAM_FUNC void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (!microphone_muted)
        fifo_write(&fifo_mic, &rx_dma_buffer[SAI_DMA_BUFFER_BYTES / 2], SAI_DMA_BUFFER_BYTES / 2);
}

RAM_FUNC void USBD_XBLC_Data_Received(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t sz) {
    fifo_write(&fifo_speaker, data, sz);
}

RAM_FUNC void USBD_XBLC_Get_Data(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t sz) {
    fifo_read(&fifo_mic, data, sz);
}

RAM_FUNC void USBD_XBLC_Sample_Rate_Changed(USBD_HandleTypeDef *pdev, uint32_t sample_rate) {
    pending_sample_rate_change = sample_rate;
}

RAM_FUNC void USBD_XBLC_AGC_Changed(USBD_HandleTypeDef *pdev, uint8_t value) {
    pending_agc_change = 0x80 | value;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MAX9860_IRQ_Pin) {
        max9860.has_irq = true;
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_CRSInitTypeDef pInit = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // PLL setup quite low only at 16Mhz. All hardware driven so we dont need much and saves power
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);

    __HAL_RCC_CRS_CLK_ENABLE();
    pInit.Prescaler = RCC_CRS_SYNC_DIV1;
    pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
    pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
    pInit.ErrorLimitValue = 34;
    pInit.HSI48CalibrationValue = 32;

    HAL_RCCEx_CRSConfig(&pInit);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00303D5B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_SAI1_Init(void) {
    hsai_BlockA1.Instance = SAI1_Block_A;
    hsai_BlockA1.Init.AudioMode = SAI_MODESLAVE_TX;
    hsai_BlockA1.Init.Synchro = SAI_SYNCHRONOUS;
    hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
    hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
    hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;
    hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    hsai_BlockB1.Instance = SAI1_Block_B;
    hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
    hsai_BlockB1.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_HF;
    hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockB1.Init.MonoStereoMode = SAI_MONOMODE;
    hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2);
    HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2);

    // Overwrite HAL Frame number with max9860 supported value
    hsai_BlockA1.Instance->FRCR &= ~SAI_xFRCR_FRL;
    hsai_BlockA1.Instance->FRCR |= 48U - 1U;

    hsai_BlockB1.Instance->FRCR &= ~SAI_xFRCR_FRL;
    hsai_BlockB1.Instance->FRCR |= 48U - 1U;
}

static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = HAL_RCC_GetSysClockFreq() / 100000ULL;  // Count at 100khz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 100;  // Count up to 100 before reset. PWM freq is 100khz/100 = 1kHz
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_UART_Init(&huart2);
    HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_DisableFifoMode(&huart2);
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = MAX9860_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MAX9860_IRQ_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MCO_MCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(MCO_MCLK_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Error_Handler(void) {
    while (1) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"E\n", 2, 0xFFFF);
    }
    __disable_irq();
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
#if (0)
    io_puts("SAI Error ", NULL);
    switch (hsai->ErrorCode) {
        case HAL_SAI_ERROR_OVR:
            io_puts("OVR\n", NULL);
            break;
        case HAL_SAI_ERROR_UDR:
            io_puts("UDR\n", NULL);
            break;
        case HAL_SAI_ERROR_AFSDET:
            io_puts("AFSDET\n", NULL);
            break;
        case HAL_SAI_ERROR_LFSDET:
            io_puts("LFSDET\n", NULL);
            break;
        case HAL_SAI_ERROR_CNREADY:
            io_puts("CNREADY\n", NULL);
            break;
        case HAL_SAI_ERROR_WCKCFG:
            io_puts("WCKCFG\n", NULL);
            break;
        case HAL_SAI_ERROR_TIMEOUT:
            io_puts("TIMEOUT\n", NULL);
            break;
        case HAL_SAI_ERROR_DMA:
            io_puts("DMA\n", NULL);
            break;
        case HAL_SAI_ERROR_NONE:
            break;
        default:
            io_puts("UNKNOWN\n", NULL);
            break;
    }
#endif
}
