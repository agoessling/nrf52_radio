#pragma once

#include <zephyr/zephyr.h>

#include "common/lite_pipe.h"
#include "common/radio.h"

#define UART_BUF_LEN RADIO_MAX_PKT_LEN
#define UART_NUM_BUF 2

typedef struct {
  const struct device *uart;
  uint32_t baud;
  LitePipe *pipe;
} UartTxConfig;

typedef struct {
  struct {
    atomic_t tx_multiple_callbacks;
    atomic_t other;
  } errors;
  atomic_t tx_bytes;
} UartTxStats;

typedef struct {
  struct k_sem tx_done_sem;
  uint8_t buf[UART_NUM_BUF][UART_BUF_LEN];
  uint32_t buf_write_index;
  UartTxStats stats;
} UartTxState;

void UartTxThread(const UartTxConfig *config, UartTxState *state, void *arg3);
bool UartTxGetErrorStatus(const UartTxState *state);