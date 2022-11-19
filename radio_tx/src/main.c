#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define TRANSMIT_PACKET_LEN 248
#define RX_TIMEOUT_US 500

#define UART uart0
#define BAUD 921600
#define NUM_UART_RX_BUF 4

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

static UartRxSlab g_uart_rx_slab = {.write_index = 0, .read_buf = (uint8_t *)g_uart_rx_slab.buf};

K_MSGQ_DEFINE(g_rx_packet_queue, sizeof(UartRxPacket), 20, 1);

static ErrorCount g_error_count;

static void UartCallback(const struct device *uart, struct uart_event *event, void *user_data) {
  switch (event->type) {
    case UART_RX_BUF_REQUEST:
      g_uart_rx_slab.write_index = (g_uart_rx_slab.write_index + 1) % NUM_UART_RX_BUF;

      uint8_t *const next_buf = g_uart_rx_slab.buf[g_uart_rx_slab.write_index];

      // Buffer is being updated to what was last read in main thread indicating likely overrun.
      if (next_buf == g_uart_rx_slab.read_buf) {
        g_error_count.rx_overflow++;
        k_msgq_purge(&g_rx_packet_queue);
      }

      if (uart_rx_buf_rsp(uart, next_buf, TRANSMIT_PACKET_LEN) < 0) {
        g_error_count.other++;
      }

      break;
    case UART_RX_RDY: {
      // This copies uart_event_rx twice (once into UartRxPacket and again into the queue), but as
      // it is only 3 words this is deemed faster than implementing an allocated approach (FIFO).
      const UartRxPacket pkt = {.event = event->data.rx, .pipe = 0};
      if (k_msgq_put(&g_rx_packet_queue, &pkt, K_NO_WAIT) < 0) {
        g_error_count.pkt_overflow++;
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

static uint32_t g_rx_count = 0;
static volatile bool g_run_main_thread = true;

void main(void) {
  LOG_INF("Starting TX radio...");

  const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(UART));
  if (!device_is_ready(uart)) {
    LOG_ERR("Could not get uart.");
    return;
  }

  if (SetupUart(uart) < 0) {
    return;
  };

  while (true) {
    if (!g_run_main_thread) continue;

    UartRxPacket pkt;
    if (k_msgq_get(&g_rx_packet_queue, &pkt, K_FOREVER) == 0) {
      g_uart_rx_slab.read_buf = pkt.event.buf;
      g_rx_count += pkt.event.len;
    }
  }
}