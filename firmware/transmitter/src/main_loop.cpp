#include "main_loop.hh"

#include "error_handler.hh"

#include <cmath>

#include <blt/devices/nrf24/nrf24.hh>
#include <blt/gpio.hh>
#include <blt/messages.hh>
#include <blt/time.hh>
#include <blt/uart.hh>
#include <spi/spi.hh>

SPI_HandleTypeDef* g_hspi = nullptr;
auto               fhSpi()
{
  return g_hspi;
}

const uint8_t nrf24_addresses[][6] = {"1Node", "2Node"};

void main_loop(SPI_HandleTypeDef* hspi)
{
  using namespace blt;
  using namespace gpio;
  using namespace time::literals;

  using led_status = gpio::pin_out<PB, 8>;
  // using led_power  = gpio::pin_out<PB, 9>;
  using leds = gpio::pin_out<PB, 8 /*, 9*/>;

  using btn_pair = gpio::pin_in<PB, 6>;

  using csn = gpio::pin_out<PA, 4>;
  using ce  = gpio::pin_out<PA, 3>;

  g_hspi = hspi;

  time::init();
  leds::clear();

  nrf24::device<spi::device<fhSpi>, csn, ce> radio;
  blt::message::motion::type                 data;

  led_status::set();
  if (!radio.init()) {
    error_handler();
  }

  if (!radio.test()) {
    error_handler();
  }

  radio.setChannel(40);
  radio.setRfPowerLevel<nrf24::Value::RfPowerMinimum>();
  radio.setDataRate<nrf24::Value::RfDatarate250kbps>();
  radio.setAutoAck(true);
  radio.setupAutoRetransmit<nrf24::Value::AutoRetransmitDelay1500us, 15>();
  radio.setAddressWidth(5);
  radio.openWritingPipe(nrf24_addresses[1]);
  radio.openReadingPipe(1, nrf24_addresses[0]);

  radio.startListening();
  radio.stopListening();

  radio.powerUp();

  led_status::clear();

  int16_t value = 0;

  while (true) {
    if (btn_pair::read()) {
      dfu_run_bootloader();
    }

    float sv = sin(value / 1000.f);
    float cv = cos(value / 1000.f);
    data.x   = 1000 * sv;
    data.y   = 1000 * cv;
    data.z   = value % 1000;
    data.rx  = (value - 5000) % 18000;
    data.ry  = (value) % 18000;
    data.rz  = (value + 5000) % 18000;
    value++;
    if (value >= (18000 - 5000))
      value = -(18000 - 5000);

    led_status::toggle();
    radio.writeFast(reinterpret_cast<uint8_t*>(&data), blt::message::motion::size);
  }

  error_handler();
}
