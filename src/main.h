// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void io_npf_putc(int c, void *ctx);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define FS_LRCLK_Pin GPIO_PIN_4
#define FS_LRCLK_GPIO_Port GPIOA
#define MAX9860_IRQ_Pin GPIO_PIN_7
#define MAX9860_IRQ_GPIO_Port GPIOA
#define MAX9860_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOB
#define MCO_MCLK_Pin GPIO_PIN_8
#define MCO_MCLK_GPIO_Port GPIOA
#define SD_TX_Pin GPIO_PIN_10
#define SD_TX_GPIO_Port GPIOA
#define SCK_BCLK_Pin GPIO_PIN_3
#define SCK_BCLK_GPIO_Port GPIOB
#define SD_RX_Pin GPIO_PIN_5
#define SD_RX_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif
