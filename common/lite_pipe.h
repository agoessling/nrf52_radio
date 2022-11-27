#pragma once

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

// Thread-safe single producer single consumer ring buffer.
// Pipe is assumed to only be flushed by the producer.
typedef struct {
  struct ring_buf ring_buf;
  struct k_mutex flush_mutex;
  struct k_sem data_ready_sem;
} LitePipe;

static inline void LitePipeInit(LitePipe *pipe, uint8_t *buf, size_t len) {
  ring_buf_init(&pipe->ring_buf, len, buf);
  k_mutex_init(&pipe->flush_mutex);
  k_sem_init(&pipe->data_ready_sem, 0, 1);
}

static inline int LitePipeGet(LitePipe *pipe, uint8_t *buf, size_t len, size_t *bytes_read,
                              k_timeout_t timeout) {
  // Wait for data ready.
  int ret = k_sem_take(&pipe->data_ready_sem, timeout);
  if (ret < 0) {
    return ret;
  }

  // Should always be available unless producer is actively flushing ring buffer.
  ret = k_mutex_lock(&pipe->flush_mutex, timeout);
  if (ret < 0) {
    return ret;
  }

  *bytes_read = ring_buf_get(&pipe->ring_buf, buf, len);

  k_mutex_unlock(&pipe->flush_mutex);

  if (*bytes_read == 0) {
    return -EAGAIN;
  }

  return 0;
}

static inline size_t LitePipePut(LitePipe *pipe, const uint8_t *buf, size_t len) {
  const size_t bytes_written = ring_buf_put(&pipe->ring_buf, buf, len);

  if (bytes_written > 0) {
    k_sem_give(&pipe->data_ready_sem);
  }

  return bytes_written;
}

// Only allowed to be called by producer.
static inline int LitePipeFlush(LitePipe *pipe, k_timeout_t timeout) {
  int ret = k_mutex_lock(&pipe->flush_mutex, timeout);
  if (ret < 0) {
    return ret;
  }

  ring_buf_reset(&pipe->ring_buf);

  k_mutex_unlock(&pipe->flush_mutex);

  return 0;
}

static inline int LitePipeGetAvailable(LitePipe *pipe, size_t *bytes_available,
                                       k_timeout_t timeout) {
  int ret = k_mutex_lock(&pipe->flush_mutex, timeout);
  if (ret < 0) {
    return ret;
  }

  *bytes_available = ring_buf_size_get(&pipe->ring_buf);

  k_mutex_unlock(&pipe->flush_mutex);

  return 0;
}

// Only allowed to be called by producer.
static inline size_t LitePipePutAvailable(LitePipe *pipe) {
  return ring_buf_space_get(&pipe->ring_buf);
}