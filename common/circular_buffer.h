#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  kCircBufStatSuccess = 0,
  kCircBufStatNoData = -1,
  kCircBufStatNoMem = -2,
  kCircBufStatUnaligned = -3,
} CircBufStat;

typedef struct {
  uint8_t *data;
  uint8_t *volatile write_ptr;  // Points to next write address.
  uint8_t *volatile read_ptr;  // Points to next read address.
  uint8_t *volatile watermark_ptr;  // Points to address after largest written address.
  size_t size;
} CircBuf;

static inline size_t CircBufAlignedSize(size_t size) {
  return (size + sizeof(size_t) - 1) / sizeof(size_t) * sizeof(size_t);
}

static inline uint8_t *CircBufAlignedAddr(const uint8_t *addr) {
  return ((uintptr_t)addr + sizeof(size_t) - 1) / sizeof(size_t) * sizeof(size_t);
}

static inline CircBufStat CircBufInit(CircBuf *buf, uint8_t *data_buf, size_t size) {
  if ((uintptr_t)data_buf % sizeof(size_t)) return kCircBufStatUnaligned;

  buf->data = data_buf;
  buf->write_ptr = data_buf;
  buf->read_ptr = data_buf;
  buf->watermark_ptr = data_buf;
  buf->size = size;

  return kCircBufStatSuccess;
}

static inline bool CircBufIsEmpty(const CircBuf *buf) {
  return buf->read_ptr == buf->write_ptr;
}

static inline CircBufStat CircBufRead(CircBuf *buf, uint8_t **output_ptr,
                                      size_t *output_len) {
  uint8_t *const read_ptr = buf->read_ptr;
  uint8_t *const write_ptr = buf->write_ptr;
  uint8_t *const watermark_ptr = buf->watermark_ptr;

  uint8_t *new_read_ptr = read_ptr;
  if (read_ptr == watermark_ptr) {
    new_read_ptr = buf->data;
  }

  if (new_read_ptr == write_ptr) return kCircBufStatNoData;

  *output_ptr = new_read_ptr + sizeof(size_t);
  *output_len = *((size_t *)new_read_ptr);

  buf->read_ptr = new_read_ptr;

  return kCircBufStatSuccess;
}

static inline CircBufStat CircBufPop(CircBuf *buf) {
  uint8_t *const read_ptr = buf->read_ptr;
  uint8_t *const write_ptr = buf->write_ptr;
  uint8_t *const watermark_ptr = buf->watermark_ptr;
  if (read_ptr == write_ptr) return kCircBufStatNoData;

  const size_t size = *((size_t *)read_ptr) + sizeof(size_t);
  uint8_t *new_read_ptr = CircBufAlignedAddr(read_ptr + size);

  if (new_read_ptr >= watermark_ptr) {
    new_read_ptr = buf->data;
  }

  buf->read_ptr = new_read_ptr;

  return kCircBufStatSuccess;
}

static inline CircBufStat CircBufReserve(CircBuf *buf, size_t len, uint8_t **output_ptr) {
  uint8_t *const read_ptr = buf->read_ptr;
  uint8_t *const write_ptr = buf->write_ptr;
  uint8_t *const watermark_ptr = buf->watermark_ptr;

  const size_t total_len = CircBufAlignedSize(sizeof(size_t) + len);
  uint8_t *new_write_ptr;
  uint8_t *new_watermark_ptr;

  // The new write_ptr would need to wrap.
  if (write_ptr + total_len > buf->data + buf->size) {
    // Only allow wrapping if it doesn't jump over or land on read_ptr.
    if (read_ptr > write_ptr) return kCircBufStatNoMem;
    if (read_ptr == buf->data) return kCircBufStatNoMem;
    new_write_ptr = buf->data;
    new_watermark_ptr = write_ptr;
  } else {
    new_write_ptr = write_ptr;
    new_watermark_ptr = watermark_ptr;
  }

  size_t bytes_available = buf->size + (read_ptr - new_write_ptr) - 1;
  if (bytes_available >= buf->size) {
    bytes_available -= buf->size;
  }

  if (total_len > bytes_available) return kCircBufStatNoMem;

  buf->watermark_ptr = new_watermark_ptr;
  buf->write_ptr = new_write_ptr;
}