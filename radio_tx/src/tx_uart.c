#include "tx_uart.h"

#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tx_uart, CONFIG_LOG_LEVEL_DEFAULT);

#include "common/leds.h"

static int UartEnable(UartState *state) {
  return uart_rx_enable(state->config->uart, state->rx_bufs[state->write_buf_index],
                        UART_RX_BUF_LEN, state->config->rx_timeout_us);
}

static void UartCallback(const struct device *uart, struct uart_event *event, UartState *state) {
  switch (event->type) {
    case UART_RX_DISABLED:
      atomic_inc(&state->stats.errors.rx_disabled);
      // Fall through intended.
    case UART_RX_BUF_REQUEST:
      if (++state->write_buf_index >= UART_RX_NUM_BUFS) {
        state->write_buf_index = 0;
      }

      uint8_t *const next_buf = state->rx_bufs[state->write_buf_index];

      // Next buffer is being updated to what was last read in uart thread indicating an overrun is
      // possible. Purge event queue and skip over what is currently being read.  Note this will
      // also occur when a deluge of RX_DISABLE events arrive.
      if (next_buf == atomic_ptr_get(&state->read_buf)) {
        atomic_inc(&state->stats.errors.isr_overflow);
        k_msgq_purge(&state->rx_event_queue);
      }

      if (event->type == UART_RX_BUF_REQUEST) {
        const int ret = uart_rx_buf_rsp(uart, next_buf, UART_RX_BUF_LEN);
        // -EBUSY is sometimes returned after a restart due to UART_RX_DISABLED.
        if (ret < 0 && ret != -EBUSY) {
          atomic_inc(&state->stats.errors.other);
        }
      } else {
        UartEnable(state);
      }

      break;
    case UART_RX_RDY:
      atomic_add(&state->stats.rx_bytes, event->data.rx.len);
      if (k_msgq_put(&state->rx_event_queue, &event->data.rx, K_NO_WAIT) < 0) {
        atomic_inc(&state->stats.errors.queue_overflow);
        k_msgq_purge(&state->rx_event_queue);
      }
      break;
    case UART_RX_STOPPED:
      atomic_inc(&state->stats.errors.rx_stopped);
      break;
    case UART_TX_DONE:
    case UART_TX_ABORTED:
      atomic_inc(&state->stats.errors.other);
      break;
    case UART_RX_BUF_RELEASED:
      break;
  }
}

static int UartInit(const UartConfig *config, UartState *state) {
  state->config = config;

  state->write_buf_index = 0;
  state->read_buf = NULL;

  k_msgq_init(&state->rx_event_queue, (uint8_t *)state->_rx_event_queue_buf,
              sizeof(state->_rx_event_queue_buf[0]), ARRAY_SIZE(state->_rx_event_queue_buf));

  memset(&state->stats, 0, sizeof(state->stats));

// Setup enable switch GPIO.
#ifdef CONFIG_RADIO_GPIO_ENABLE_SW
  const struct gpio_dt_spec enable_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(tx_enable_sw), gpios);

  if (!gpio_is_ready_dt(&enable_sw)) {
    LOG_ERR("Enable switch port not ready.");
    return -1;
  }

  if (gpio_pin_configure_dt(&enable_sw, GPIO_INPUT) < 0) {
    LOG_ERR("Could not set enable switch to input.");
    return -1;
  }
#endif

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

  if (UartEnable(state) < 0) {
    LOG_ERR("Could not enable uart rx.");
    return -1;
  }

  return 0;
}

bool UartGetErrorStatus(const UartState *state) {
  bool status = false;
  status = status || atomic_get(&state->stats.errors.isr_overflow);
  status = status || atomic_get(&state->stats.errors.queue_overflow);
  status = status || atomic_get(&state->stats.errors.other);

  return status;
}

void UartRxThread(const UartConfig *config, UartState *state, void *unused3) {
  if (UartInit(config, state) < 0) return;

  while (true) {
    // Wait on rx event queue.
    struct uart_event_rx event;
    if (k_msgq_get(&state->rx_event_queue, &event, K_FOREVER) < 0) {
      atomic_inc(&state->stats.errors.other);
      continue;
    }

    // Signal to ISR what buffer is currently being read.
    atomic_ptr_set(&state->read_buf, event.buf);

#ifdef CONFIG_RADIO_GPIO_ENABLE_SW
    // Discard buffer if this is the primary channel and switch is not enabled.
    const struct gpio_dt_spec enable_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(tx_enable_sw), gpios);
    if (config->primary && gpio_pin_get_dt(&enable_sw) != 1) {
      continue;
    }
#endif

    // If pipe can't accept bytes flush it.
    if (k_pipe_write_avail(config->output_pipe) < event.len) {
      k_pipe_flush(config->output_pipe);
      atomic_inc(&state->stats.errors.pipe_overflow);
    }

    // Write data into pipe.
    size_t bytes_written;
    if (k_pipe_put(config->output_pipe, event.buf + event.offset, event.len, &bytes_written,
                   event.len, K_NO_WAIT) < 0) {
      atomic_inc(&state->stats.errors.other);
    }
  }
}