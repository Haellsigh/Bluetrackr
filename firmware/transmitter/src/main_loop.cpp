#include "main_loop.hh"

#include "error_handler.hh"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <blt/uart.hh>
#include <nrf24_custom/nrf24.hh>
#include <spi/spi.hh>

SPI_HandleTypeDef* g_hspi = nullptr;
auto               fhSpi()
{
  return g_hspi;
}

const uint8_t nrf24_addresses[][6] = {"1Node", "2Node"};
uint8_t       tx_buffer[]          = "some data from transmitter!!!!\n";

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

  led_status::set();
  if (!radio.init()) {
    error_handler();
  }

  if (!radio.test()) {
    error_handler();
  }

  radio.setChannel(40);
  radio.setRfPowerLevel<nrf24::Value::RfPowerMinus18dBm>();
  radio.setDataRate<nrf24::Value::RfDatarate1Mbps>();
  radio.setAutoAck(true);
  radio.setupAutoRetransmit<nrf24::Value::AutoRetransmitDelay1500us, 15>();
  radio.setAddressWidth(5);
  radio.openWritingPipe(nrf24_addresses[1]);
  radio.openReadingPipe(1, nrf24_addresses[0]);

  radio.startListening();
  radio.stopListening();

  radio.powerUp();

  led_status::clear();

  // std::size_t index = 0;

  while (true) {
    led_status::toggle();
    // radio.write(tx_buffer, 32);
    // snprintf(reinterpret_cast<char*>(tx_buffer), 32, "%zu\n", index++);
    radio.writeFast(tx_buffer, 32);
    // time::delay(2_ms);
  }

  error_handler();
}
