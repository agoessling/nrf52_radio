#include <esb.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(radio_tx, CONFIG_RADIO_TX_APP_LOG_LEVEL);

#include "leds.h"
#include "radio.h"

// Common defines.
#define NUM_PIPES RADIO_NUM_PIPES
#define NUM_POLL_EVENTS (NUM_PIPES + 1)

// TX radio defines
#define TX_MAX_PKT_LEN RADIO_MAX_PKT_LEN
#define TX_TIMEOUT_US 10000

// UART defines
#define UART uart0
#define UART_BAUD 921600
#define UART_TIMEOUT_US 500
#define UART_NUM_SLABS 4
#define UART_EVENT_QUEUE_LEN 50

// LED defines
#define LED_STATUS 0
#define LED_ERROR 1

typedef struct {
  atomic_t rx_overflow;
  atomic_t rx_flush;
  atomic_t queue_overflow;
  atomic_t tx_failed;
  atomic_t tx_timeout;
  atomic_t other;
} ErrorCount;

typedef struct {
  ErrorCount errors;
  atomic_t rx_bytes;
  atomic_t tx_packets;
} Stats;

static Stats g_stats;

typedef struct {
  uint8_t buf[UART_NUM_SLABS][TX_MAX_PKT_LEN];
  uint32_t write_slab_index;  // Index of slab currently being written by uart.
  atomic_ptr_t read_slab;  // Pointer to slab currently being read by main thread.
} UartRxSlabs;

static UartRxSlabs g_uart_rx_slabs[NUM_PIPES] = {
    {.write_slab_index = 0, .read_slab = ATOMIC_PTR_INIT(g_uart_rx_slabs[0].buf)},
    {.write_slab_index = 0, .read_slab = ATOMIC_PTR_INIT(g_uart_rx_slabs[1].buf)},
};

K_MSGQ_DEFINE(g_rx_event_queue_0, sizeof(struct uart_event_rx), UART_EVENT_QUEUE_LEN, 1);
K_MSGQ_DEFINE(g_rx_event_queue_1, sizeof(struct uart_event_rx), UART_EVENT_QUEUE_LEN, 1);
static struct k_msgq *const g_rx_event_queue[NUM_PIPES] = {
    &g_rx_event_queue_0,
    &g_rx_event_queue_1,
};

// Raised with a boolean result that indicates that at least one TX pkt has succeeded or failed.
static struct k_poll_signal g_tx_radio_signal = K_POLL_SIGNAL_INITIALIZER(g_tx_radio_signal);

static struct k_poll_event g_poll_events[NUM_POLL_EVENTS] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &g_tx_radio_signal,
                                    0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
                                    &g_rx_event_queue_0, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
                                    &g_rx_event_queue_1, 0),
};

typedef struct {
  struct esb_payload payload;
  uint32_t write_index;
} TxPayload;

typedef enum {
  kWaitStateRadio,
  kWaitStateUart,
} WaitState;

typedef struct {
  uint32_t read_index;
} UartEventReadState;

static void UartCallback(const struct device *uart, struct uart_event *event, void *user_data) {
  // TODO: Get rid of hardcoded pipe.
  const uint32_t pipe = 0;
  UartRxSlabs *const rx_slabs = &g_uart_rx_slabs[pipe];
  struct k_msgq *const event_queue = g_rx_event_queue[pipe];

  switch (event->type) {
    case UART_RX_BUF_REQUEST:
      if (++rx_slabs->write_slab_index >= UART_NUM_SLABS) {
        rx_slabs->write_slab_index = 0;
      }

      uint8_t *const next_buf = rx_slabs->buf[rx_slabs->write_slab_index];

      // Next buffer is being updated to what was last read in main thread indicating an overrun is
      // possible. Purge event queue so that main thread drops bytes and catches up.
      if (next_buf == atomic_ptr_get(&rx_slabs->read_slab)) {
        atomic_inc(&g_stats.errors.rx_overflow);
        k_msgq_purge(event_queue);
      }

      if (uart_rx_buf_rsp(uart, next_buf, TX_MAX_PKT_LEN) < 0) {
        atomic_inc(&g_stats.errors.other);
      }
      break;
    case UART_RX_RDY:
      if (k_msgq_put(event_queue, &event->data.rx, K_NO_WAIT) < 0) {
        atomic_inc(&g_stats.errors.queue_overflow);
        k_msgq_purge(event_queue);
      }
      break;
    case UART_TX_DONE:
    case UART_TX_ABORTED:
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
    case UART_RX_STOPPED:
      break;
  }
}

static void EsbHandler(const struct esb_evt *event) {
  switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
      k_poll_signal_raise(&g_tx_radio_signal, true);
      break;

    case ESB_EVENT_TX_FAILED:
      if (esb_pop_tx() < 0) {
        atomic_inc(&g_stats.errors.other);
      }
      int ret = esb_start_tx();
      if (ret < 0 && ret != -ENODATA) {
        atomic_inc(&g_stats.errors.other);
      }
      atomic_inc(&g_stats.errors.tx_failed);

      k_poll_signal_raise(&g_tx_radio_signal, false);
      break;

    case ESB_EVENT_RX_RECEIVED:
      atomic_inc(&g_stats.errors.other);
      break;
  }
}

void HandleErrorLed(void) {
  if (atomic_get(&g_stats.errors.rx_overflow) != 0 ||
      atomic_get(&g_stats.errors.queue_overflow) != 0 ||
      atomic_get(&g_stats.errors.tx_timeout) != 0 || atomic_get(&g_stats.errors.other) != 0) {
    SetLed(LED_ERROR, true);
  } else {
    SetLed(LED_ERROR, false);
  }
}

static int SetupUart(const struct device *uart) {
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

  if (uart_callback_set(uart, UartCallback, NULL) < 0) {
    LOG_ERR("Could not set uart callback.");
    return -1;
  }

  // TODO: Get rid of hardcoded pipe.
  const uint32_t pipe = 0;
  UartRxSlabs *const rx_slabs = &g_uart_rx_slabs[pipe];

  if (uart_rx_enable(uart, rx_slabs->buf[0], TX_MAX_PKT_LEN, UART_TIMEOUT_US) < 0) {
    LOG_ERR("Could not enable uart rx.");
    return -1;
  }

  return 0;
}

void main(void) {
  LOG_INF("Starting TX radio...");

  const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(UART));
  if (!device_is_ready(uart)) {
    atomic_inc(&g_stats.errors.other);
    LOG_ERR("Could not get uart.");
    return;
  }

  if (SetupUart(uart) < 0) {
    atomic_inc(&g_stats.errors.other);
    return;
  };

  if (SetupEsb(ESB_MODE_PTX, EsbHandler) < 0) {
    atomic_inc(&g_stats.errors.other);
    return;
  }

  if (SetupLeds() < 0) {
    atomic_inc(&g_stats.errors.other);
    return;
  };

  TxPayload tx_payloads[NUM_PIPES] = {
      {.payload = {.pipe = 0, .noack = false}, .write_index = 0},
      {.payload = {.pipe = 1, .noack = false}, .write_index = 0},
  };

  UartEventReadState read_states[NUM_PIPES] = {
      {.read_index = 0},
      {.read_index = 0},
  };

  WaitState wait_state = kWaitStateUart;
  k_timeout_t timeout;

  while (true) {
    HandleErrorLed();

    // Setup polling based on what we are waiting for.
    switch (wait_state) {
      case kWaitStateUart:
        timeout = K_FOREVER;
        g_poll_events[0].type = K_POLL_TYPE_IGNORE;
        SetLed(LED_STATUS, false);
        break;
      case kWaitStateRadio:
        timeout = K_USEC(TX_TIMEOUT_US);
        g_poll_events[0].type = K_POLL_TYPE_SIGNAL;
        break;
    }

    // Always default back to waiting on UART RX events.
    wait_state = kWaitStateUart;

    // Wait for poll event.
    int ret = k_poll(g_poll_events, NUM_POLL_EVENTS, timeout);
    if (ret == -EAGAIN) {
      atomic_inc(&g_stats.errors.tx_timeout);
    } else if (ret < 0) {
      atomic_inc(&g_stats.errors.other);
      goto reset_poll_events;
    }

    SetLed(LED_STATUS, true);

    // Determine highest priority pipe with UART RX events.
    int32_t active_pipe = -1;
    bool has_uart_rx_event = false;
    for (uint32_t i = 1; i < NUM_POLL_EVENTS; ++i) {
      if (g_poll_events[i].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
        active_pipe = i - 1;
        has_uart_rx_event = true;
        break;
      }
    }

    // If no UART RX events look for any pending payloads with bytes.
    if (active_pipe < 0) {
      for (uint32_t i = 0; i < NUM_PIPES; ++i) {
        if (tx_payloads[i].write_index != 0) {
          active_pipe = i;
          break;
        }
      }
    }

    // Nothing to do.  Should never be reached.
    if (active_pipe < 0) {
      atomic_inc(&g_stats.errors.other);
      goto reset_poll_events;
    }

    // Active payload.
    TxPayload *const payload = &tx_payloads[active_pipe];

    if (has_uart_rx_event) {
      // Get event.
      struct k_msgq *const event_queue = g_rx_event_queue[active_pipe];
      struct uart_event_rx event;
      if (k_msgq_peek(event_queue, &event) < 0) {
        atomic_inc(&g_stats.errors.rx_flush);
        goto reset_poll_events;
      }

      // Signal to publisher which slab we are reading.
      UartRxSlabs *const rx_slabs = &g_uart_rx_slabs[active_pipe];
      atomic_ptr_set(&rx_slabs->read_slab, event.buf);

      // Write event data to payload.
      UartEventReadState *const read_state = &read_states[active_pipe];

      const size_t write_available = TX_MAX_PKT_LEN - payload->write_index;
      const size_t read_available = event.len - read_state->read_index;
      uint8_t *const dest = payload->payload.data + payload->write_index;
      uint8_t *const src = event.buf + event.offset + read_state->read_index;
      const size_t len = MIN(write_available, read_available);

      memcpy(dest, src, len);

      payload->write_index += len;
      read_state->read_index += len;

      // Queued event has been fully read.  Pop from queue.
      if (read_state->read_index == event.len) {
        if (k_msgq_get(event_queue, &event, K_NO_WAIT) < 0) {
          atomic_inc(&g_stats.errors.rx_flush);
        }
        read_state->read_index = 0;
      }
    }

    // Attempt to put payload on ESB FIFO.
    payload->payload.length = payload->write_index;

    // Reset signal to catch new freeing of the TX FIFO.
    k_poll_signal_reset(&g_tx_radio_signal);

    ret = esb_write_payload(&payload->payload);

    if (ret == -ENOMEM) {
      wait_state = kWaitStateRadio;
    } else if (ret == 0) {
      payload->write_index = 0;
      atomic_inc(&g_stats.tx_packets);
      atomic_add(&g_stats.rx_bytes, payload->payload.length);
    } else {
      atomic_inc(&g_stats.errors.other);
    }

  reset_poll_events:
    for (uint32_t i = 0; i < NUM_POLL_EVENTS; ++i) {
      g_poll_events[i].state = K_POLL_STATE_NOT_READY;
    }
  }
}