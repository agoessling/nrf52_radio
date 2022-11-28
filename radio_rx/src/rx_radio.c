#include "rx_radio.h"

#include <string.h>

#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(rx_radio, CONFIG_LOG_LEVEL_DEFAULT);

#include "common/leds.h"
#include "common/lite_pipe.h"
#include "common/radio.h"

typedef struct {
  struct {
    atomic_t pipe_overflow[RADIO_NUM_PIPES];
    atomic_t other;
  } errors;
  atomic_t rx_packets;
  atomic_t rx_bytes[RADIO_NUM_PIPES];
} RadioStats;

typedef struct {
  struct k_sem rx_sem;
  struct esb_payload rx_payload;
  RadioStats stats;
} RadioRxState;

static RadioRxState g_radio_state;

static void EsbHandler(const struct esb_evt *event) {
  switch (event->evt_id) {
    case ESB_EVENT_RX_RECEIVED:
      atomic_inc(&g_radio_state.stats.rx_packets);
      SetLed(LED_RADIO_STATUS, true);
      k_sem_give(&g_radio_state.rx_sem);
      break;
    case ESB_EVENT_TX_SUCCESS:
    case ESB_EVENT_TX_FAILED:
      atomic_inc(&g_radio_state.stats.errors.other);
      break;
  }
}

static int RadioInit(void) {
  k_sem_init(&g_radio_state.rx_sem, 0, 1);
  memset(&g_radio_state.stats, 0, sizeof(g_radio_state.stats));

  if (SetupEsb(ESB_MODE_PRX, EsbHandler) < 0) return -1;

  if (esb_start_rx() < 0) {
    LOG_ERR("Could not start ESB receiving.");
    return -1;
  }

  return 0;
}

bool RadioRxGetErrorStatus(void) {
  bool status = false;

  for (int i = 0; i < RADIO_NUM_PIPES; ++i) {
    status = status || atomic_get(&g_radio_state.stats.errors.pipe_overflow[i]); 
  }

  status = status || atomic_get(&g_radio_state.stats.errors.other);

  return status;
}

void RadioRxThread(const RadioRxConfig *config, void *arg2, void *arg3) {
  if (RadioInit() < 0) return;

  while (true) {
    // Wait for RX data from radio.
    if (k_sem_take(&g_radio_state.rx_sem, K_FOREVER) < 0) {
      atomic_inc(&g_radio_state.stats.errors.other);
      continue;
    }

    // Continually move data from RX FIFO to appropriate ring buffer.
    while (true) {
      int ret = esb_read_rx_payload(&g_radio_state.rx_payload);
      if (ret < 0) {
        if (ret != -ENODATA) {
          atomic_inc(&g_radio_state.stats.errors.other);
        }
        break;
      }

      // Ignore if pipe is out of bounds.
      const uint8_t pipe_num = g_radio_state.rx_payload.pipe;
      if (pipe_num >= RADIO_NUM_PIPES) {
        atomic_inc(&g_radio_state.stats.errors.other);
        continue;
      }

      LitePipe *const pipe = config->pipes[pipe_num];
      uint8_t *const data = g_radio_state.rx_payload.data;
      const size_t len = g_radio_state.rx_payload.length;

      // Flush pipe if not enough room to write RX payload.
      if (LitePipePutAvailable(pipe) < len) {
        atomic_inc(&g_radio_state.stats.errors.pipe_overflow[pipe_num]);

        if (LitePipeFlush(pipe, K_FOREVER) < 0) {
          atomic_inc(&g_radio_state.stats.errors.other);
        }
      }

      // Write RX payload to pipe.
      const size_t bytes_written = LitePipePut(pipe, data, len);
      if (bytes_written != len) {
        atomic_inc(&g_radio_state.stats.errors.other);
      }

      atomic_add(&g_radio_state.stats.rx_bytes[pipe_num], bytes_written);
    }
  }
}