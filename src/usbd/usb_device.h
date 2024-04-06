// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "usbd_def.h"

void MX_USB_Device_Init(void);

#ifdef __cplusplus
}
#endif
