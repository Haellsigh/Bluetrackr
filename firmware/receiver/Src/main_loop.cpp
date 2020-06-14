#include "main_loop.hh"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <error_handler.hh>

using namespace blt;
using namespace gpio;

void main_loop() {
  // Outputs
  using led_red    = gpio::pin_out<PC, p6>;
  using led_orange = gpio::pin_out<PC, p8>;
  using led_green  = gpio::pin_out<PC, p9>;
  using led_blue   = gpio::pin_out<PC, p7>;
  using leds       = gpio::pin_out<PA, p6, p8, p9, p7>;

  // Inputs
  using btn_pair = gpio::pin_in<PA, p0>;

  delay::init();
  leds::clear();

  while (true) {
    led_red::toggle();
    delay::ms(100);
    led_orange::toggle();
    delay::ms(100);
    led_green::toggle();
    delay::ms(100);
    led_blue::toggle();
    delay::ms(100);
  }
}
