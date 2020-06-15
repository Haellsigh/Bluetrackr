#pragma once

#include <blt/gpio.hh>
#include <blt/time.hh>

static void error_handler() {
  using namespace blt;
  using namespace gpio;

  time::init();
  using led_status = gpio::pin_out<PB, 3>;

  while (true) {
    using namespace time::literals;
    time::delay(50_ms);
    led_status::toggle();
  }
}