#ifndef BLT_ERROR_HANDLER_H_
#define BLT_ERROR_HANDLER_H_

#include <blt/gpio.hh>
#include <blt/time.hh>

namespace blt {

static void error_handler() {
  using namespace gpio;

  delay::init();
  using led_status = gpio::pin_out<PB, p3>;

  while (true) {
    delay::ms(50);
    led_status::toggle();
  }
}

}  // namespace blt

#endif  // BLT_ERROR_HANDLER_H_
