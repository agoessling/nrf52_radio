#include <esb.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/notify.h>

#define RADIO_MAX_PKT_LEN 248
#define RADIO_NUM_PIPES 2
#define RADIO_RETRANSMIT_DELAY_US 600
#define RADIO_RETRANSMIT_COUNT 2

BUILD_ASSERT(RADIO_MAX_PKT_LEN <= CONFIG_ESB_MAX_PAYLOAD_LENGTH, "RADIO_MAX_PKT_LEN too big.");
BUILD_ASSERT(RADIO_NUM_PIPES > 0, "Too few radio pipes.");
BUILD_ASSERT(RADIO_NUM_PIPES <= 8, "Too many radio pipes.");

static int SetupEsb(enum esb_mode mode, const esb_event_handler handler) {
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
      .mode = mode,
      .event_handler = handler,
      .bitrate = ESB_BITRATE_1MBPS,
      .crc = ESB_CRC_16BIT,
      .tx_output_power = ESB_TX_POWER_4DBM,
      .retransmit_delay = RADIO_RETRANSMIT_DELAY_US,
      .retransmit_count = RADIO_RETRANSMIT_COUNT,
      .tx_mode = ESB_TXMODE_AUTO,
      .payload_length = RADIO_MAX_PKT_LEN,
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

  // Enable pipes.
  if (esb_enable_pipes(0xFF >> (8 - RADIO_NUM_PIPES)) < 0) {
    LOG_ERR("Could not enable pipes.");
    return -1;
  }

  return 0;
}