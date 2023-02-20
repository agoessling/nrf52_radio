#pragma once

#include <esb.h>

#define RADIO_MAX_PKT_LEN 252
#define RADIO_NUM_PIPES 2
#define RADIO_RETRANSMIT_DELAY_US 600
#define RADIO_RETRANSMIT_COUNT 2

#if DT_NODE_EXISTS(DT_NODELABEL(nrf_radio_fem))
  #define RADIO_TX_POWER_DBM 20
#else
  #define RADIO_TX_POWER_DBM 8
#endif

BUILD_ASSERT(RADIO_MAX_PKT_LEN <= CONFIG_ESB_MAX_PAYLOAD_LENGTH, "RADIO_MAX_PKT_LEN too big.");
BUILD_ASSERT(RADIO_NUM_PIPES > 0, "Too few radio pipes.");
BUILD_ASSERT(RADIO_NUM_PIPES <= 8, "Too many radio pipes.");

int SetupEsb(enum esb_mode mode, const esb_event_handler handler);