#ifndef BLT_ERROR_HANDLER_H_
#define BLT_ERROR_HANDLER_H_

#include <blt/gpio.hh>
#include <blt/time.hh>

namespace blt {

static void error_handler() {
  using namespace gpio;

  delay::init();
  using led_power = gpio::pin_out<PC, p6>;

  while (true) {
    delay::ms(100);
    led_power::toggle();
  }
}

}  // namespace blt

#endif  // BLT_ERROR_HANDLER_H_
