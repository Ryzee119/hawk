// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} fifo_t;

void fifo_init(fifo_t *fifo, uint8_t *pbuf, uint32_t sz);
void fifo_write(fifo_t *fifo, const uint8_t *data, uint32_t len);
void fifo_read(fifo_t *fifo, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif