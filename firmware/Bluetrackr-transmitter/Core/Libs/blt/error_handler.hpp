#ifndef BLT_ERROR_HANDLER_H_
#define BLT_ERROR_HANDLER_H_

#include <blt/delay.hpp>
#include <blt/gpio.hpp>

namespace blt {

static const void error_handler() {
  using led_power = gpio::pin_out<gpio::PortA, gpio::pin3>;

  while (true) {
    delay::ms(100);
    led_power::toggle();
  }
}

}  // namespace blt

#endif  // BLT_ERROR_HANDLER_H_
