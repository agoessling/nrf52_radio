#pragma once

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "common/radio.h"

#define UART_RX_NUM_BUFS 4
#define UART_RX_BUF_LEN RADIO_MAX_PKT_LEN
#define UART_RX_EVENT_QUEUE_LEN 16

BUILD_ASSERT(UART_RX_NUM_BUFS >= 4, "Must have at least 4 buffers.");

typedef struct {
  const struct device *uart;
  uint32_t baud;
  int32_t rx_timeout_us;
  struct k_pipe *output_pipe;
  bool primary;
} UartConfig;

typedef struct {
  struct {
    atomic_t isr_overflow;
    atomic_t queue_overflow;
    atomic_t pipe_overflow;
    atomic_t rx_stopped;
    atomic_t rx_disabled;
    atomic_t other;
  } errors;
  atomic_t rx_bytes;
} UartStats;

typedef struct {
  uint8_t rx_bufs[UART_RX_NUM_BUFS][UART_RX_BUF_LEN];
  uint32_t write_buf_index;
  atomic_ptr_t read_buf;

  struct uart_event_rx _rx_event_queue_buf[UART_RX_EVENT_QUEUE_LEN];
  struct k_msgq rx_event_queue;

  const UartConfig *config;
  UartStats stats;
} UartState;

void UartRxThread(const UartConfig *config, UartState *state, void *unused3);
bool UartGetErrorStatus(const UartState *state);