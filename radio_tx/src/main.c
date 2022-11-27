#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_RADIO_TX_APP_LOG_LEVEL);

#include "common/leds.h"
#include "tx_radio.h"
#include "tx_uart.h"

// RX Pipe defines
#define RX_PIPE_SIZE 2048

// Thread defines
#define THREAD_NUM 4
#define THREAD_STACK_SIZE 512
#define THREAD_UART_PRIORITY 10
#define THREAD_RADIO_PRIORITY 20
#define THREAD_MAIN_PRIORITY 30

// Data channel defines
#define NUM_CHANNELS 2

typedef struct {
  const char *name;
  struct k_thread *thread_data;
  k_thread_stack_t *stack;
  size_t stack_size;
  k_thread_entry_t entry;
  void *entry_arg1;
  void *entry_arg2;
  void *entry_arg3;
  int priority;
  uint32_t options;
  k_timeout_t delay;
} ThreadConfig;

// Static state for application / threads.
K_PIPE_DEFINE(g_pipe_0, RX_PIPE_SIZE, 1);
K_PIPE_DEFINE(g_pipe_1, RX_PIPE_SIZE, 1);
static struct k_pipe *const g_pipes[NUM_CHANNELS] = {&g_pipe_0, &g_pipe_1};

static UartState g_uart_state[NUM_CHANNELS];
static RadioState g_radio_state[NUM_CHANNELS];

K_THREAD_STACK_ARRAY_DEFINE(g_thread_stacks, THREAD_NUM, THREAD_STACK_SIZE);
static struct k_thread g_thread_data[THREAD_NUM];

void main(void) {
  LOG_INF("Starting TX radio...");

  if (SetupLeds() < 0) {
    return;
  };

  if (RadioInit() < 0) {
    return;
  }

  const UartConfig uart_configs[NUM_CHANNELS] = {
      {
          .uart = DEVICE_DT_GET(DT_NODELABEL(uart0)),
          .baud = 921600,
          .rx_timeout_us = 500,
          .output_pipe = g_pipes[0],
      },
      {
          .uart = DEVICE_DT_GET(DT_NODELABEL(uart1)),
          .baud = 921600,
          .rx_timeout_us = 500,
          .output_pipe = g_pipes[1],
      },
  };

  const RadioTxConfig radio_configs[NUM_CHANNELS] = {
      {
          .input_data_pipe = g_pipes[0],
          .radio_tx_pipe = 0,
      },
      {
          .input_data_pipe = g_pipes[1],
          .radio_tx_pipe = 1,
      },
  };

  k_tid_t threads[THREAD_NUM];
  const ThreadConfig thread_configs[THREAD_NUM] = {
      {
          .name = "Uart0",
          .thread_data = &g_thread_data[0],
          .stack = g_thread_stacks[0],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[0]),
          .entry = (k_thread_entry_t)UartRxThread,
          .entry_arg1 = (void *)&uart_configs[0],
          .entry_arg2 = &g_uart_state[0],
          .entry_arg3 = NULL,
          .priority = THREAD_UART_PRIORITY + 0,
          .options = 0,
          .delay = K_NO_WAIT,
      },
      {
          .name = "Uart1",
          .thread_data = &g_thread_data[1],
          .stack = g_thread_stacks[1],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[1]),
          .entry = (k_thread_entry_t)UartRxThread,
          .entry_arg1 = (void *)&uart_configs[1],
          .entry_arg2 = &g_uart_state[1],
          .entry_arg3 = NULL,
          .priority = THREAD_UART_PRIORITY + 1,
          .options = 0,
          .delay = K_NO_WAIT,
      },
      {
          .name = "Radio0",
          .thread_data = &g_thread_data[2],
          .stack = g_thread_stacks[2],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[2]),
          .entry = (k_thread_entry_t)RadioTxThread,
          .entry_arg1 = (void *)&radio_configs[0],
          .entry_arg2 = &g_radio_state[0],
          .entry_arg3 = NULL,
          .priority = THREAD_RADIO_PRIORITY + 0,
          .options = 0,
          .delay = K_NO_WAIT,
      },
      {
          .name = "Radio1",
          .thread_data = &g_thread_data[3],
          .stack = g_thread_stacks[3],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[3]),
          .entry = (k_thread_entry_t)RadioTxThread,
          .entry_arg1 = (void *)&radio_configs[1],
          .entry_arg2 = &g_radio_state[1],
          .entry_arg3 = NULL,
          .priority = THREAD_RADIO_PRIORITY + 1,
          .options = 0,
          .delay = K_NO_WAIT,
      },
  };

  for (int i = 0; i < THREAD_NUM; ++i) {
    const ThreadConfig *const c = &thread_configs[i];
    threads[i] = k_thread_create(c->thread_data, c->stack, c->stack_size, c->entry, c->entry_arg1,
                                 c->entry_arg2, c->entry_arg3, c->priority, c->options, c->delay);
    k_thread_name_set(threads[i], c->name);
  }

  // Set priority to background allowing other threads to start.
  k_thread_priority_set(k_current_get(), THREAD_MAIN_PRIORITY);

  while (true) {
    k_msleep(200);

    // Set error LED based on state of threads.
    bool error_status = false;
    error_status = error_status || RadioGetGlobalErrorStatus();
    error_status = error_status || RadioGetErrorStatus(&g_radio_state[0]);
    error_status = error_status || RadioGetErrorStatus(&g_radio_state[1]);
    error_status = error_status || UartGetErrorStatus(&g_uart_state[0]);
    error_status = error_status || UartGetErrorStatus(&g_uart_state[1]);

    SetLed(LED_ERROR, error_status);

    // If any threads exit, toggle error LED indefinitely.
    for (int i = 0; i < THREAD_NUM; ++i) {
      int ret = k_thread_join(thread_configs[i].thread_data, K_NO_WAIT);
      while (ret == 0) {
        SetLed(LED_ERROR, true);
        k_msleep(500);
        SetLed(LED_ERROR, false);
        k_msleep(500);
      }
    }
  }
}