#include <esb.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(radio_tx, CONFIG_RADIO_TX_APP_LOG_LEVEL);

#include "common/leds.h"
#include "common/radio.h"
#include "rx_radio.h"
#include "rx_uart.h"

// Application defines
#define NUM_CHANNEL RADIO_NUM_PIPES
#define PIPE_BUF_LEN 512

// Thread defines
#define THREAD_NUM 3
#define THREAD_STACK_SIZE 512
#define THREAD_RADIO_PRIORITY 10
#define THREAD_UART_PRIORITY 20
#define THREAD_MAIN_PRIORITY 30

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

// Static state for threads and application.
static uint8_t g_pipe_bufs[NUM_CHANNEL][PIPE_BUF_LEN];
static LitePipe g_pipes[NUM_CHANNEL];

static UartTxState g_uart_states[NUM_CHANNEL];

K_THREAD_STACK_ARRAY_DEFINE(g_thread_stacks, THREAD_NUM, THREAD_STACK_SIZE);
static struct k_thread g_thread_data[THREAD_NUM];

static void InitPipes(void) {
  for (int i = 0; i < NUM_CHANNEL; ++i) {
    LitePipeInit(&g_pipes[i], g_pipe_bufs[i], sizeof(g_pipe_bufs[i]));
  }
}

void main(void) {
  LOG_INF("Starting RX radio...");

  if (SetupLeds() < 0) {
    return;
  };

  InitPipes();

  const UartTxConfig uart_configs[NUM_CHANNEL] = {
      {
          .uart = DEVICE_DT_GET(DT_NODELABEL(uart0)),
          .baud = 921600,
          .pipe = &g_pipes[0],
      },
      {
          .uart = DEVICE_DT_GET(DT_NODELABEL(uart1)),
          .baud = 921600,
          .pipe = &g_pipes[1],
      },
  };

  const RadioRxConfig radio_config = {.pipes = {&g_pipes[0], &g_pipes[1]}};

  k_tid_t threads[THREAD_NUM];
  const ThreadConfig thread_configs[THREAD_NUM] = {
      {
          .name = "RadioRx",
          .thread_data = &g_thread_data[0],
          .stack = g_thread_stacks[0],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[0]),
          .entry = (k_thread_entry_t)RadioRxThread,
          .entry_arg1 = (void *)&radio_config,
          .entry_arg2 = NULL,
          .entry_arg3 = NULL,
          .priority = THREAD_RADIO_PRIORITY,
          .options = 0,
          .delay = K_NO_WAIT,
      },
      {
          .name = "Uart0",
          .thread_data = &g_thread_data[1],
          .stack = g_thread_stacks[1],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[1]),
          .entry = (k_thread_entry_t)UartTxThread,
          .entry_arg1 = (void *)&uart_configs[0],
          .entry_arg2 = &g_uart_states[0],
          .entry_arg3 = NULL,
          .priority = THREAD_UART_PRIORITY + 0,
          .options = 0,
          .delay = K_NO_WAIT,
      },
      {
          .name = "Uart1",
          .thread_data = &g_thread_data[2],
          .stack = g_thread_stacks[2],
          .stack_size = K_THREAD_STACK_SIZEOF(g_thread_stacks[2]),
          .entry = (k_thread_entry_t)UartTxThread,
          .entry_arg1 = (void *)&uart_configs[1],
          .entry_arg2 = &g_uart_states[1],
          .entry_arg3 = NULL,
          .priority = THREAD_UART_PRIORITY + 1,
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
    error_status = error_status || RadioRxGetErrorStatus();
    error_status = error_status || UartTxGetErrorStatus(&g_uart_states[0]);
    error_status = error_status || UartTxGetErrorStatus(&g_uart_states[1]);

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