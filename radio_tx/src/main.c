#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#define TRANSMIT_PACKET_LEN 248
#define RX_TIMEOUT_US 500

#define UART uart1
#define BAUD 921600
#define NUM_UART_RX_BUF 4

LOG_MODULE_REGISTER(radio_tx, CONFIG_RADIO_TX_APP_LOG_LEVEL);

typedef struct {
  uint8_t buf[NUM_UART_RX_BUF][TRANSMIT_PACKET_LEN];
  uint32_t write_index;  // Index of buffer currently being written by uart.
  uint8_t *read_buf;  // Pointer to buffer currently being read by main thread.
} UartRxQueue;

static UartRxQueue g_uart_rx_queue = {.write_index = 0, .read_buf = &g_uart_rx_queue.buf[0]};

static void UartCallback(const struct device *uart, struct uart_event *event, void *user_data) {
  switch (event->type) {
    case UART_RX_BUF_REQUEST:
      g_uart_rx_queue.write_index = (g_uart_rx_queue.write_index + 1) % NUM_UART_RX_BUF;

      uint8_t * const next_buf = g_uart_rx_queue.buf[g_uart_rx_queue.write_index];

      // Buffer is being updated to what was last read in main thread indicating likely overrun.
      if (next_buf == g_uart_rx_queue.read_buf) {
        LOG_ERR("UART Rx overrun.");
      }

      if (uart_rx_buf_rsp(uart, next_buf, TRANSMIT_PACKET_LEN) < 0) {
        LOG_ERR("Could not provide UART RX buffer.");
      }

      break;
    case UART_RX_RDY:
      LOG_INF("RX %d bytes.", event->data.rx.len);
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

  if (uart_rx_enable(uart, g_uart_rx_queue.buf[0], TRANSMIT_PACKET_LEN, RX_TIMEOUT_US) < 0) {
    LOG_ERR("Could not enable uart rx.");
    return -1;
  }

  return 0;
}

void main(void) {
  LOG_INF("Starting TX radio...");
  printk("HEY!\n");

  const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(UART));
  if (!device_is_ready(uart)) {
    LOG_ERR("Could not get uart.");
    return;
  }

  (void)g_uart_rx_queue;

  if (SetupUart(uart) < 0) {
    return;
  };

  while (true) {
    k_cpu_idle();
  }
}