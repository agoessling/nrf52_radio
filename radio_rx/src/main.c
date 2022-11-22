#include <esb.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(radio_tx, CONFIG_RADIO_TX_APP_LOG_LEVEL);

#include "leds.h"
#include "radio.h"

// Commmon defines
#define NUM_PIPES RADIO_NUM_PIPES
#define NUM_POLL_EVENTS (NUM_PIPES + 1)

// Radio defines
#define RX_BUF_LEN 1024

// UART defines
#define UART_BAUD 921600

typedef struct {
  atomic_t rx_overflow;
  atomic_t other;
} ErrorCount;

typedef struct {
  ErrorCount errors;
  atomic_t tx_bytes;
  atomic_t rx_packets;
} Stats;

static Stats g_stats;

static struct k_poll_signal g_radio_data_sig = K_POLL_SIGNAL_INITIALIZER(g_radio_data_sig);
static struct k_poll_signal g_uart_0_ready_sig = K_POLL_SIGNAL_INITIALIZER(g_uart_0_ready_sig);
static struct k_poll_signal g_uart_1_ready_sig = K_POLL_SIGNAL_INITIALIZER(g_uart_1_ready_sig);

static struct k_poll_signal *const g_uart_ready_sigs[NUM_PIPES] = {
    &g_uart_0_ready_sig,
    &g_uart_1_ready_sig,
};

static struct k_poll_event g_poll_events[NUM_POLL_EVENTS] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &g_radio_data_sig,
                                    0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
                                    &g_uart_0_ready_sig, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
                                    &g_uart_1_ready_sig, 0),
};

RING_BUF_DECLARE(g_rx_0_ring_buf, RX_BUF_LEN);
RING_BUF_DECLARE(g_rx_1_ring_buf, RX_BUF_LEN);

static struct ring_buf *const g_rx_ring_bufs[NUM_PIPES] = {
    &g_rx_0_ring_buf,
    &g_rx_1_ring_buf,
};

static const struct device *const g_uarts[NUM_PIPES] = {
    DEVICE_DT_GET(DT_NODELABEL(uart0)),
    DEVICE_DT_GET(DT_NODELABEL(uart1)),
};

static atomic_t g_uart_busy_flags[NUM_PIPES] = {
    ATOMIC_INIT(0),
    ATOMIC_INIT(0),
};

static void UartCallback(const struct device *uart, struct uart_event *event, void *user_data) {
  const uint32_t pipe = (uintptr_t)user_data;  // Abuse user pointer as data.
  if (pipe >= NUM_PIPES) {
    atomic_inc(&g_stats.errors.other);
    return;
  }
  struct k_poll_signal *const ready_signal = g_uart_ready_sigs[pipe];
  atomic_t *const busy_flag = &g_uart_busy_flags[pipe];

  switch (event->type) {
    case UART_RX_BUF_REQUEST:
    case UART_RX_RDY:
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
    case UART_RX_STOPPED:
      break;
    case UART_TX_DONE:
    case UART_TX_ABORTED:
      atomic_add(&g_stats.tx_bytes, event->data.tx.len);
      atomic_clear(busy_flag);
      k_poll_signal_raise(ready_signal, event->data.tx.len);
      break;
  }
}

static void EsbHandler(const struct esb_evt *event) {
  switch (event->evt_id) {
    case ESB_EVENT_RX_RECEIVED:
      atomic_inc(&g_stats.rx_packets);
      k_poll_signal_raise(&g_radio_data_sig, 0);
      break;
    case ESB_EVENT_TX_SUCCESS:
    case ESB_EVENT_TX_FAILED:
      atomic_inc(&g_stats.errors.other);
      break;
  }
}

static int SetupUart(const struct device *uart, uint32_t pipe) {
  const struct uart_config uart_config = {
      .baudrate = UART_BAUD,
      .parity = UART_CFG_PARITY_NONE,
      .stop_bits = UART_CFG_STOP_BITS_1,
      .data_bits = UART_CFG_DATA_BITS_8,
      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };

  if (uart_configure(uart, &uart_config) < 0) {
    LOG_ERR("Could not configure uart.");
    return -1;
  }

  if (uart_callback_set(uart, UartCallback, (void *)pipe) < 0) {
    LOG_ERR("Could not set uart callback.");
    return -1;
  }

  return 0;
}

void main(void) {
  LOG_INF("Starting RX radio...");

  for (uint32_t i = 0; i < ARRAY_SIZE(g_uarts); ++i) {
    if (!device_is_ready(g_uarts[i])) {
      atomic_inc(&g_stats.errors.other);
      LOG_ERR("Could not get uart.");
      return;
    }

    if (SetupUart(g_uarts[i], i) < 0) {
      atomic_inc(&g_stats.errors.other);
      return;
    };
  }

  if (SetupLeds() < 0) {
    atomic_inc(&g_stats.errors.other);
    return;
  };

  if (SetupEsb(ESB_MODE_PRX, EsbHandler) < 0) {
    atomic_inc(&g_stats.errors.other);
    return;
  }

  if (esb_start_rx() < 0) {
    LOG_ERR("Could not start ESB RX.");
    atomic_inc(&g_stats.errors.other);
    return;
  }

  uint32_t uart_pending_bytes[NUM_PIPES] = {0, 0};

  while (true) {
    int ret = k_poll(g_poll_events, NUM_POLL_EVENTS, K_FOREVER);
    if (ret < 0) {
      atomic_inc(&g_stats.errors.other);
      goto reset_poll_events;
    }

    // Read all radio packets currently in RX FIFO into circular buffers.
    while (true) {
      struct esb_payload payload;

      k_poll_signal_reset(&g_radio_data_sig);
      int ret = esb_read_rx_payload(&payload);

      if (ret < 0) {
        if (ret != -ENODATA) {
          atomic_inc(&g_stats.errors.other);
        }
        break;
      }

      const uint32_t active_pipe = payload.pipe;
      if (active_pipe >= NUM_PIPES) {
        atomic_inc(&g_stats.errors.other);
        continue;
      }

      struct ring_buf *const active_buf = g_rx_ring_bufs[active_pipe];
      const struct device *const active_uart = g_uarts[active_pipe];

      // Check for buffer overflow.
      const uint32_t available_bytes = ring_buf_space_get(active_buf);
      if (available_bytes < payload.length) {
        atomic_inc(&g_stats.errors.rx_overflow);

        // Reset UART TX if transmitting.
        if (atomic_get(&g_uart_busy_flags[active_pipe])) {
          if (uart_tx_abort(active_uart) < 0) {
            atomic_inc(&g_stats.errors.other);
          }
        }

        // Reset RX buffer.
        ring_buf_reset(active_buf);
        uart_pending_bytes[active_pipe] = 0;
      }

      // Write data into ring buffer.
      if (ring_buf_put(active_buf, payload.data, payload.length) < 0) {
        atomic_inc(&g_stats.errors.other);
      }
    }

    // Write any available data to UARTS.
    for (uint32_t i = 0; i < NUM_PIPES; ++i) {
      if (atomic_get(&g_uart_busy_flags[i])) continue;

      struct ring_buf *const active_buf = g_rx_ring_bufs[i];
      uint32_t *const pending_bytes = &uart_pending_bytes[i];

      // If UART not busy then the pending bytes must have been transmitted.
      if (ring_buf_get_finish(active_buf, *pending_bytes) < 0) {
        atomic_inc(&g_stats.errors.other);
        // Unclear what to do here.  Reset buffer.
        ring_buf_reset(active_buf);
      }

      *pending_bytes = 0;

      uint8_t *data;
      const uint32_t len = ring_buf_get_claim(active_buf, &data, RX_BUF_LEN);

      struct k_poll_signal *const signal = g_uart_ready_sigs[i];
      k_poll_signal_reset(signal);

      // No data to send.
      if (len == 0) continue;

      const struct device *const active_uart = g_uarts[i];

      // Attempt to start transfer.
      atomic_set(&g_uart_busy_flags[i], 1);
      int ret = uart_tx(active_uart, data, len, SYS_FOREVER_US);

      if (ret == 0) {
        *pending_bytes = len;
      } else {
        atomic_inc(&g_stats.errors.other);
        if (ret != -EBUSY) {
          atomic_set(&g_uart_busy_flags[i], 0);
        }
      }
    }

  reset_poll_events:
    for (uint32_t i = 0; i < NUM_POLL_EVENTS; ++i) {
      g_poll_events[i].state = K_POLL_STATE_NOT_READY;
    }
  }
}