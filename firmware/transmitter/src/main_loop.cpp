#include "main_loop.hh"
#include "main.h"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <error_handler.hh>

//#include "mpu.h"

using namespace blt;
using namespace gpio;

void main_loop(SPI_HandleTypeDef* hspi) {
  // Outputs
  using led_status = gpio::pin_out<PB, p1>;
  //using led_power  = gpio::pin_out<PA, p3>;
  using leds       = gpio::pin_out<PB, p1>;

  // Inputs
  using btn_pair = gpio::pin_in<PB, p8>;

  delay::init();
  leds::clear();

  /// Begin radio init
  led_status::set();

  delay::ms(50);

  while (true) {
    led_status::set();
    delay::ms(100);
    led_status::clear();
    delay::ms(100);
  }

  led_status::clear();
}
