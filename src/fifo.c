// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2024 Ryan Wendland
// Fifo with forced overwrite and return.

#include "fifo.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define FIFO_MIN(a, b) ((a) < (b) ? (a) : (b))
#define FIFO_MAX(a, b) ((a) > (b) ? (a) : (b))

void fifo_init(fifo_t *fifo, uint8_t *pbuf, uint32_t sz) {
    fifo->buffer = pbuf;
    fifo->size = sz;
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

void fifo_write(fifo_t *fifo, const uint8_t *data, uint32_t len) {
    bool overwrite = (fifo->count + len) > fifo->size;

    while (len) {
        uint32_t chunk = FIFO_MIN(len, (fifo->size - fifo->tail));
        memcpy(&fifo->buffer[fifo->tail], data, chunk);
        fifo->tail = (fifo->tail + chunk) % fifo->size;
        data += chunk;
        len -= chunk;
        fifo->count += chunk;
    }

    if (overwrite) {
        fifo->head = fifo->tail;
        fifo->count = fifo->size;
    }
}

void fifo_read(fifo_t *fifo, uint8_t *data, uint32_t len) {
    uint32_t available_len = FIFO_MIN(len, fifo->count);

    if (available_len < len) {
        memset(&data[available_len], 0, len - available_len);
    }

    while (available_len) {
        uint32_t chunk = FIFO_MIN(available_len, fifo->size - fifo->head);
        memcpy(data, &fifo->buffer[fifo->head], chunk);
        fifo->head = (fifo->head + chunk) % fifo->size;
        data += chunk;
        available_len -= chunk;
        fifo->count -= chunk;
    }
}
