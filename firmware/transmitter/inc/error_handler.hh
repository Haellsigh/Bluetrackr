#pragma once

#include <blt/gpio.hh>
#include <blt/time.hh>

static void error_handler()
{
  using namespace blt;
  using namespace gpio;
  using namespace time::literals;

  time::init();
  using led_power = gpio::pin_out<PB, 9>;

  while (true) {
    led_power::toggle();
    time::delay(50_ms);
  }
}