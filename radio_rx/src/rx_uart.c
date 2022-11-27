#include "rx_uart.h"

#include <string.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>

LOG_MODULE_REGISTER(rx_uart, CONFIG_LOG_LEVEL_DEFAULT);

#include "common/leds.h"
#include "common/lite_pipe.h"

static void UartCallback(const struct device *uart, struct uart_event *event, UartTxState *state) {
  switch (event->type) {
    case UART_TX_ABORTED:
      atomic_inc(&state->stats.errors.other);
    case UART_TX_DONE:
      atomic_add(&state->stats.tx_bytes, event->data.tx.len);

      SetLed(LED_STATUS, false);

      if (k_sem_count_get(&state->tx_done_sem) != 0) {
        atomic_inc(&state->stats.errors.tx_multiple_callbacks);
      }
      k_sem_give(&state->tx_done_sem);
      break;
    case UART_RX_BUF_REQUEST:
    case UART_RX_RDY:
    case UART_RX_DISABLED:
    case UART_RX_STOPPED:
    case UART_RX_BUF_RELEASED:
      atomic_inc(&state->stats.errors.other);
      break;
  }
}

static int UartInit(const UartTxConfig *config, UartTxState *state) {
  int ret;
  if ((ret = k_sem_init(&state->tx_done_sem, 1, 1)) < 0) return ret;
  state->buf_write_index = 0;
  memset(&state->stats, 0, sizeof(state->stats));

  const struct uart_config uart_config = {
      .baudrate = config->baud,
      .parity = UART_CFG_PARITY_NONE,
      .stop_bits = UART_CFG_STOP_BITS_1,
      .data_bits = UART_CFG_DATA_BITS_8,
      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };

  if (!device_is_ready(config->uart)) {
    LOG_ERR("Could not get uart.");
    return -1;
  }

  if (uart_configure(config->uart, &uart_config) < 0) {
    LOG_ERR("Could not configure uart.");
    return -1;
  }

  if (uart_callback_set(config->uart, (uart_callback_t)UartCallback, state) < 0) {
    LOG_ERR("Could not set uart callback.");
    return -1;
  }

  return 0;
}

bool UartTxGetErrorStatus(const UartTxState *state) {
  bool status = false;
  status = status || atomic_get(&state->stats.errors.tx_multiple_callbacks);
  status = status || atomic_get(&state->stats.errors.other);

  return status;
}

void UartTxThread(const UartTxConfig *config, UartTxState *state, void *arg3) {
  if (UartInit(config, state) < 0) return;

  while (true) {
    // Wait for data from pipe.
    size_t bytes_read = 0;
    int ret = LitePipeGet(config->pipe, state->buf[state->buf_write_index], UART_BUF_LEN,
                          &bytes_read, K_FOREVER);
    if (ret < 0) {
      // Timeout can occur even with K_FOREVER in the case of the producer flushing the pipe.
      if (ret != -EAGAIN) {
        atomic_inc(&state->stats.errors.other);
      }
      continue;
    }

    // Wait until UART is ready for next buffer.
    if (k_sem_take(&state->tx_done_sem, K_FOREVER) < 0) {
      atomic_inc(&state->stats.errors.other);
      continue;
    }

    // Update write buffer.
    const uint32_t read_index = state->buf_write_index;
    if (++state->buf_write_index >= UART_NUM_BUF) state->buf_write_index = 0;

    // Start UART transfer.
    if (uart_tx(config->uart, state->buf[read_index], bytes_read, SYS_FOREVER_US) < 0) {
      atomic_inc(&state->stats.errors.other);
      continue;
    }
  }
}