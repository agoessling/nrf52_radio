#include <esb.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/notify.h>
#include <zephyr/sys/ring_buffer.h>

// Common defines.
#define NUM_PIPES 2

// TX radio defines
#define TX_MAX_PKT_LEN 248
#define TX_RETRANSMIT_DELAY_US 600
#define TX_RETRANSMIT_COUNT 3

// UART defines
#define UART uart0
#define UART_BAUD 921600
#define UART_TIMEOUT_US 50
#define UART_NUM_SLABS 8
#define UART_EVENT_QUEUE_LEN 20

LOG_MODULE_REGISTER(radio_tx, CONFIG_RADIO_TX_APP_LOG_LEVEL);

typedef struct {
  atomic_t rx_overflow;
  atomic_t pkt_overflow;
  atomic_t poll_fail;
  atomic_t data_not_available;
  atomic_t queue_fail;
  atomic_t other;
} ErrorCount;

typedef struct {
  ErrorCount errors;
  atomic_t rx_bytes;
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

static struct k_poll_event g_poll_events[NUM_PIPES] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
                                    &g_rx_event_queue_0, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
                                    &g_rx_event_queue_1, 0),
};

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
        atomic_inc(&g_stats.errors.pkt_overflow);
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
  (void)event;
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

static int SetupEsbTx(void) {
  // Setup HF clock required for ESB.
  struct onoff_manager *const clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
  if (!clk_mgr) {
    LOG_ERR("Could not get clock manager.");
    return -1;
  }

  struct onoff_client clk_client;
  sys_notify_init_spinwait(&clk_client.notify);

  if (onoff_request(clk_mgr, &clk_client) < 0) {
    LOG_ERR("Could not request clock on.");
    return -1;
  }

  bool completed = false;
  while (!completed) {
    int result;
    completed = sys_notify_fetch_result(&clk_client.notify, &result) == 0;

    if (completed && result < 0) {
      LOG_ERR("Could not start clock.");
      return -1;
    }
  }

  // Setup ESB itself.
  const struct esb_config esb_config = {
      .protocol = ESB_PROTOCOL_ESB_DPL,
      .mode = ESB_MODE_PTX,
      .event_handler = EsbHandler,
      .bitrate = ESB_BITRATE_1MBPS,
      .crc = ESB_CRC_16BIT,
      .tx_output_power = ESB_TX_POWER_4DBM,
      .retransmit_delay = TX_RETRANSMIT_DELAY_US,
      .retransmit_count = TX_RETRANSMIT_COUNT,
      .tx_mode = ESB_TXMODE_AUTO,
      .payload_length = TX_MAX_PKT_LEN,
      .selective_auto_ack = true,
  };

  if (esb_init(&esb_config) < 0) {
    LOG_ERR("Could not initialize ESB.");
    return -1;
  }

  const uint8_t pipe_0_base_addr[4] = {0xD8, 0xD8, 0xD8, 0xD8};
  const uint8_t pipe_1_7_base_addr[4] = {0xC9, 0xC9, 0xC9, 0xC9};
  const uint8_t addr_prefixes[8] = {0xD8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF};

  if (esb_set_base_address_0(pipe_0_base_addr) < 0) {
    LOG_ERR("Could not set base address for pipe 0.");
    return -1;
  }

  if (esb_set_base_address_1(pipe_1_7_base_addr) < 0) {
    LOG_ERR("Could not set base address for pipes 1-7.");
    return -1;
  }

  if (esb_set_prefixes(addr_prefixes, ARRAY_SIZE(addr_prefixes)) < 0) {
    LOG_ERR("Could not set prefix address for pipes.");
    return -1;
  }

  // Enable pipes 0 and 1.
  if (esb_enable_pipes((1 << 1) | (1 << 0)) < 0) {
    LOG_ERR("Could not enable pipes.");
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

  if (SetupEsbTx() < 0) {
    atomic_inc(&g_stats.errors.other);
    return;
  }

  struct esb_payload tx_payload = {
      .noack = false,
  };
  (void)tx_payload;

  // TODO: Get rid of hardcoded pipe.
  const uint32_t pipe = 0;
  struct k_msgq *const event_queue = g_rx_event_queue[pipe];

  while (true) {
    if (k_poll(g_poll_events, ARRAY_SIZE(g_poll_events), K_FOREVER) < 0) {
      atomic_inc(&g_stats.errors.other);
      goto reset_poll_events;
    }

    if (g_poll_events[0].state != K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
      atomic_inc(&g_stats.errors.other);
      goto reset_poll_events;
    }

    struct uart_event_rx event;
    if (k_msgq_get(event_queue, &event, K_NO_WAIT) < 0) {
      atomic_inc(&g_stats.errors.other);
      goto reset_poll_events;
    }

    UartRxSlabs *const rx_slabs = &g_uart_rx_slabs[pipe];
    atomic_ptr_set(&rx_slabs->read_slab, event.buf);
    atomic_add(&g_stats.rx_bytes, event.len);

    // Copy data into esb payload.
    // const uint8_t *const rx_data = rx_packet.event.buf + rx_packet.event.offset;
    // tx_payload.length = rx_packet.event.len;
    // tx_payload.pipe = rx_packet.pipe;
    // memcpy(tx_payload.data, rx_data, rx_packet.event.len);

reset_poll_events:
    for (uint32_t i = 0; i < ARRAY_SIZE(g_poll_events); ++i) {
      g_poll_events[i].state = K_POLL_STATE_NOT_READY;
    }
  }
}