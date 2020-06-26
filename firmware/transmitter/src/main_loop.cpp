#include "main_loop.hh"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <blt/uart.hh>
#include <error_handler.hh>

#include <nrf24_custom/nrf24.hh>
#include <spi/spi.hh>

SPI_HandleTypeDef* g_hspi = nullptr;

auto fhSpi()
{
  return g_hspi;
}

void main_loop(UART_HandleTypeDef* huart, SPI_HandleTypeDef* hspi)
{
  using namespace blt;
  using namespace gpio;
  using namespace time::literals;

  using led_status = gpio::pin_out<PB, 3>;
  using leds       = gpio::pin_out<PB, 3>;

  using csn = gpio::pin_out<PA, 3>;
  using ce  = gpio::pin_out<PA, 4>;

  g_hspi = hspi;

  time::init();
  leds::clear();

  nrf24::device<spi::device<fhSpi>, csn, ce> radio;
  uart::init(huart);

  if (!radio.init()) {
    error_handler();
  }

  if (!radio.test()) {
    error_handler();
  }

  while (true) {
    led_status::set();
    time::delay(500_ms);
    led_status::clear();
    time::delay(500_ms);
  }

  error_handler();
}
