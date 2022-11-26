#pragma once

#include <esb.h>
#include <zephyr/kernel.h>

typedef struct {
  struct {
    atomic_t other;
  } errors;
  atomic_t tx_pkts;
} RadioStats;

typedef struct {
  struct esb_payload tx_payload;
  uint32_t tx_write_index;
  RadioStats stats;
} RadioState;

typedef struct {
  struct k_pipe *input_data_pipe;  // Input byte pipe.
  uint8_t radio_tx_pipe;  // ESB pipe for outgoing payloads.
} RadioTxConfig;

int RadioInit(void);
void RadioTxThread(const RadioTxConfig *config, RadioState *state, void *unused_3);
bool RadioGetGlobalErrorStatus(void);
bool RadioGetErrorStatus(const RadioState *state);