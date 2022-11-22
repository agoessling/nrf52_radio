#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

static struct gpio_dt_spec g_leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
};

static int SetLed(unsigned int led, bool value) {
  if (led >= ARRAY_SIZE(g_leds)) {
    return -1;
  }

  gpio_port_pins_t mask = 1 << g_leds[led].pin;
  gpio_port_value_t val = !value << g_leds[led].pin;

  gpio_port_set_masked_raw(g_leds[led].port, mask, val);
  return 0;
}

static int SetupLeds(void) {
  for (uint32_t i = 0; i < ARRAY_SIZE(g_leds); ++i) {
    struct gpio_dt_spec *const led = &g_leds[i];

    if (!device_is_ready(led->port)) {
      LOG_ERR("LED port not ready.");
      return -1;
    }

    SetLed(i, false);

    if (gpio_pin_configure_dt(led, GPIO_OUTPUT) < 0) {
      LOG_ERR("Could not set LED GPIO to output.");
      return -1;
    }
  }
  return 0;
}