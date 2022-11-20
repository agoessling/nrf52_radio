#include <esb.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/notify.h>

#define TRANSMIT_PACKET_LEN 248
#define RX_TIMEOUT_US 500

#define UART uart0
#define BAUD 921600
#define NUM_UART_RX_BUF 4

#define RETRANSMIT_DELAY_US 600
#define RETRANSMIT_COUNT 3

LOG_MODULE_REGISTER(radio_tx, CONFIG_RADIO_TX_APP_LOG_LEVEL);

typedef struct {
  uint8_t buf[NUM_UART_RX_BUF][TRANSMIT_PACKET_LEN];
  uint32_t write_index;  // Index of buffer currently being written by uart.
  uint8_t *read_buf;  // Pointer to buffer currently being read by main thread.
} UartRxSlab;

typedef struct {
  struct uart_event_rx event;
  uint8_t pipe;
} UartRxPacket;

typedef struct {
  uint32_t rx_overflow;
  uint32_t pkt_overflow;
  uint32_t other;
} ErrorCount;

typedef struct {
  ErrorCount errors;
  uint32_t rx_bytes;
} Stats;

static UartRxSlab g_uart_rx_slab = {.write_index = 0, .read_buf = (uint8_t *)g_uart_rx_slab.buf};

K_MSGQ_DEFINE(g_rx_packet_queue, sizeof(UartRxPacket), 20, 1);

static Stats g_stats;

static void UartCallback(const struct device *uart, struct uart_event *event, void *user_data) {
  switch (event->type) {
    case UART_RX_BUF_REQUEST:
      g_uart_rx_slab.write_index = (g_uart_rx_slab.write_index + 1) % NUM_UART_RX_BUF;

      uint8_t *const next_buf = g_uart_rx_slab.buf[g_uart_rx_slab.write_index];

      // Buffer is being updated to what was last read in main thread indicating likely overrun.
      if (next_buf == g_uart_rx_slab.read_buf) {
        g_stats.errors.rx_overflow++;
        k_msgq_purge(&g_rx_packet_queue);
      }

      if (uart_rx_buf_rsp(uart, next_buf, TRANSMIT_PACKET_LEN) < 0) {
        g_stats.errors.other++;
      }

      break;
    case UART_RX_RDY: {
      // This copies uart_event_rx twice (once into UartRxPacket and again into the queue), but as
      // it is only 3 words this is deemed faster than implementing an allocated approach (FIFO).
      const UartRxPacket rx_packet = {.event = event->data.rx, .pipe = 0};
      if (k_msgq_put(&g_rx_packet_queue, &rx_packet, K_NO_WAIT) < 0) {
        g_stats.errors.pkt_overflow++;
        k_msgq_purge(&g_rx_packet_queue);
      }
      break;
    }
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
      .baudrate = BAUD,
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

  if (uart_rx_enable(uart, g_uart_rx_slab.buf[0], TRANSMIT_PACKET_LEN, RX_TIMEOUT_US) < 0) {
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
      .retransmit_delay = RETRANSMIT_DELAY_US,
      .retransmit_count = RETRANSMIT_COUNT,
      .tx_mode = ESB_TXMODE_AUTO,
      .payload_length = TRANSMIT_PACKET_LEN,
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
    ++g_stats.errors.other;
    LOG_ERR("Could not get uart.");
    return;
  }

  if (SetupUart(uart) < 0) {
    ++g_stats.errors.other;
    return;
  };

  if (SetupEsbTx() < 0) {
    ++g_stats.errors.other;
    return;
  }

  struct esb_payload tx_payload = {
      .noack = false,
  };

  while (true) {
    UartRxPacket rx_packet;
    if (k_msgq_get(&g_rx_packet_queue, &rx_packet, K_FOREVER) < 0) {
      continue;
    }

    g_uart_rx_slab.read_buf = rx_packet.event.buf;
    g_stats.rx_bytes += rx_packet.event.len;

    // Copy data into esb payload.
    const uint8_t * const rx_data = rx_packet.event.buf + rx_packet.event.offset;
    tx_payload.length = rx_packet.event.len;
    tx_payload.pipe = rx_packet.pipe;
    memcpy(tx_payload.data, rx_data, rx_packet.event.len);
  }
}