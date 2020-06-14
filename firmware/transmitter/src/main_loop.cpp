#include "main_loop.hh"
#include "main.h"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <error_handler.hh>
//#include "mpu.h"

#include <sdbg.hh>

using namespace blt;
using namespace gpio;

void main_loop(UART_HandleTypeDef* huart) {
  sdbg::init(huart);

  // Outputs
  using led_status = gpio::pin_out<PB, p3>;
  // using led_power  = gpio::pin_out<PA, p3>;
  using leds = gpio::pin_out<PB, p3>;

  // Inputs
  using btn_pair = gpio::pin_in<PB, p8>;

  delay::init();
  leds::clear();

  while (true) {
    led_status::set();
    delay::ms(500);
    led_status::clear();
    delay::ms(500);
  }

  led_status::clear();
}
