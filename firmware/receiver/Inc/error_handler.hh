#pragma once

#include <blt/gpio.hh>
#include <blt/time.hh>

namespace blt {

static void error_handler() {
  using namespace gpio;

  delay::init();
  using led_red = gpio::pin_out<PC, p6>;

  while (true) {
    delay::ms(50);
    led_red::toggle();
  }
}

}  // namespace blt
