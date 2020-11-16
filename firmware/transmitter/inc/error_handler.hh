#pragma once

#include <blt/chrono.hh>
#include <blt/gpio.hh>

#include "main_loop.hh"

static void error_handler()
{
  using namespace blt;
  using namespace gpio;
  using namespace chrono::literals;

  using led_power = gpio::pin_out<PB, 9>;

  while (true) {
    led_power::toggle();
    chrono::delay(50ms);
  }
}