#include "tx_radio.h"

#include <string.h>

#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tx_radio, CONFIG_LOG_LEVEL_DEFAULT);

#include "common/leds.h"
#include "common/radio.h"

typedef struct {
  atomic_t tx_failed;
  atomic_t tx_multiple_callbacks;
  atomic_t other;
} RadioHandlerErrors;

typedef struct {
  struct k_sem tx_sem;  // Semaphore indicating TX FIFO is available.
  atomic_t tx_failed_flag;  // Flag indicating TX failed and FIFO should be flushed.
  atomic_t tx_attempts;  // Total TX attempts.
  atomic_t tx_pkts;  // Total successful packets.
  RadioHandlerErrors errors;
  atomic_t initialized;
} RadioGlobalState;

static RadioGlobalState g_radio_state;

static void EsbHandler(const struct esb_evt *event) {
  switch (event->evt_id) {
    case ESB_EVENT_TX_FAILED:
      atomic_set(&g_radio_state.tx_failed_flag, true);
      atomic_inc(&g_radio_state.errors.tx_failed);

    // Fall through intended.
    case ESB_EVENT_TX_SUCCESS:
      atomic_add(&g_radio_state.tx_attempts, event->tx_attempts);
      atomic_inc(&g_radio_state.tx_pkts);

      if (k_sem_count_get(&g_radio_state.tx_sem) != 0) {
        atomic_inc(&g_radio_state.errors.tx_multiple_callbacks);
      }
      k_sem_give(&g_radio_state.tx_sem);

      // Signal transmission finished.
      SetLed(LED_RADIO_STATUS, false);
      break;
    case ESB_EVENT_RX_RECEIVED:
      atomic_inc(&g_radio_state.errors.other);
      break;
  }
}

static int RadioStateInit(RadioState *state, uint8_t pipe) {
  if (pipe > RADIO_NUM_PIPES) {
    LOG_ERR("Pipe number out of range.");
    return -1;
  }

  state->tx_payload.pipe = pipe;
  state->tx_payload.noack = false;
  state->tx_write_index = 0;

  memset(&state->stats, 0, sizeof(state->stats));

  return 0;
}

static int ReadSingleByte(RadioState *state, struct k_pipe *pipe, k_timeout_t timeout) {
  const size_t bytes_to_read = RADIO_MAX_PKT_LEN - state->tx_write_index;
  if (bytes_to_read == 0) return 0;

  uint8_t *const write_ptr = state->tx_payload.data + state->tx_write_index;

  size_t bytes_read = 0;
  const int ret = k_pipe_get(pipe, write_ptr, 1, &bytes_read, 1, timeout);
  state->tx_write_index += bytes_read;

  return ret;
}

static int ReadAllAvailable(RadioState *state, struct k_pipe *pipe) {
  const size_t bytes_to_read = RADIO_MAX_PKT_LEN - state->tx_write_index;
  if (bytes_to_read == 0) return 0;

  uint8_t *const write_ptr = state->tx_payload.data + state->tx_write_index;

  size_t bytes_read = 0;
  const int ret = k_pipe_get(pipe, write_ptr, bytes_to_read, &bytes_read, 0, K_NO_WAIT);
  state->tx_write_index += bytes_read;

  return ret;
}

int RadioInit(void) {
  if (SetupEsb(ESB_MODE_PTX, EsbHandler) < 0) return -1;

  // Radio TX FIFO is set to 1.
  if (k_sem_init(&g_radio_state.tx_sem, 1, 1) < 0) return -1;

  atomic_set(&g_radio_state.tx_failed_flag, false);

  memset(&g_radio_state.errors, 0, sizeof(g_radio_state.errors));

  atomic_set(&g_radio_state.initialized, true);
  return 0;
}

bool RadioGetGlobalErrorStatus(void) {
  bool status = false;
  status = status || atomic_get(&g_radio_state.errors.tx_multiple_callbacks);
  status = status || atomic_get(&g_radio_state.errors.other);

  return status;
}

bool RadioGetErrorStatus(const RadioState *state) {
  bool status = false;
  status = status || atomic_get(&state->stats.errors.other);

  return status;
}

void RadioTxThread(const RadioTxConfig *config, RadioState *state, void *unused_3) {
  if (!atomic_get(&g_radio_state.initialized)) {
    LOG_ERR("Radio not initialized. Terminating.");
    return;
  }

  if(RadioStateInit(state, config->radio_tx_pipe) < 0) {
    return;
  }

  while (true) {
    // Only wait on data from pipe if there is nothing in the tx payload.
    if (state->tx_write_index == 0) {
      if (ReadSingleByte(state, config->input_data_pipe, K_FOREVER) < 0) {
        atomic_inc(&state->stats.errors.other);
        continue;
      }
    }

    // Wait on access to radio TX FIFO.
    if (k_sem_take(&g_radio_state.tx_sem, K_FOREVER) < 0) {
      atomic_inc(&state->stats.errors.other);
      continue;
    }

    // Read any newly available data into payload without waiting.
    if (ReadAllAvailable(state, config->input_data_pipe) < 0) {
      atomic_inc(&state->stats.errors.other);
    }

    // If a failed transmit has occurred flush the TX FIFO and clear the flag.
    if (atomic_clear(&g_radio_state.tx_failed_flag)) {
      esb_flush_tx();
    }

    // Place payload into TX FIFO.
    state->tx_payload.length = state->tx_write_index;
    int ret = esb_write_payload(&state->tx_payload);
    if (ret == 0) {
      atomic_inc(&state->stats.tx_pkts);
      state->tx_write_index = 0;
    } else {
      atomic_inc(&state->stats.errors.other);
    }

    // Signal transmission ongoing.
    SetLed(LED_RADIO_STATUS, true);
  }
}