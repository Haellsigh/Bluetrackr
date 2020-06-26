#pragma once

#include <blt/gpio.hh>
#include <blt/time.hh>

static void error_handler()
{
  using namespace blt;
  using namespace gpio;
  using namespace time::literals;

  time::init();
  using led_status = gpio::pin_out<PB, 3>;

  while (true) {
    led_status::toggle();
    time::delay(50_ms);
  }
}