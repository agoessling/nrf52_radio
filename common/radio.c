#include "radio.h"

#include <esb.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/notify.h>

LOG_MODULE_REGISTER(radio, CONFIG_LOG_LEVEL_DEFAULT);

int SetupEsb(enum esb_mode mode, const esb_event_handler handler) {
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
      .tx_output_power = RADIO_TX_POWER_DBM,
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

  // Setup CHL and CPS pins of FEM if present.
  #if DT_NODE_HAS_PROP(DT_NODELABEL(nrf_radio_fem), chl_gpios)
  {
    const struct gpio_dt_spec chl_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(nrf_radio_fem), chl_gpios);

    if (!gpio_is_ready_dt(&chl_pin)) {
      LOG_ERR("CHL GPIO port not ready.");
      return -1;
    }

    gpio_pin_set_dt(&chl_pin, 1);

    if (gpio_pin_configure_dt(&chl_pin, GPIO_OUTPUT) < 0) {
      LOG_ERR("Could not set CHL GPIO to output.");
      return -1;
    }
  }
  #endif

  #if DT_NODE_HAS_PROP(DT_NODELABEL(nrf_radio_fem), cps_gpios)
  {
    const struct gpio_dt_spec cps_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(nrf_radio_fem), cps_gpios);

    if (!gpio_is_ready_dt(&cps_pin)) {
      LOG_ERR("CPS GPIO port not ready.");
      return -1;
    }

    gpio_pin_set_dt(&cps_pin, 0);

    if (gpio_pin_configure_dt(&cps_pin, GPIO_OUTPUT) < 0) {
      LOG_ERR("Could not set CPS GPIO to output.");
      return -1;
    }
  }
  #endif

  return 0;
}