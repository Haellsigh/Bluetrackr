#pragma once

#include <blt/gpio.hh>
#include <blt/time.hh>

static void error_handler()
{
  using namespace blt;
  using namespace gpio;
  using namespace time::literals;

  time::init();

  using led_red    = gpio::pin_out<PC, 6>;
  using led_blue   = gpio::pin_out<PC, 7>;
  using led_orange = gpio::pin_out<PC, 8>;
  using led_green  = gpio::pin_out<PC, 9>;
  using leds       = gpio::pin_out<PA, 6, 8, 9, 7>;

  while (true) {
    leds::toggle();
    time::delay(50_ms);
  }
}