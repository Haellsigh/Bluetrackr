#include "main_loop.hh"

#include "error_handler.hh"
#include "usb_device.h"
#include "usbd_hid.h"

#include <blt/chrono.hh>
#include <blt/devices/nrf24/nrf24.hh>
#include <blt/gpio.hh>
#include <blt/messages.hh>
#include <spi/spi.hh>

SPI_HandleTypeDef* g_hspi = nullptr;
TIM_HandleTypeDef* g_htim = nullptr;

auto fhSpi()
{
  return g_hspi;
}

const uint8_t nrf24_addresses[][6] = {"1Node", "2Node"};
uint8_t       rx_buffer[blt::message::motion::size];

void main_loop(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* htim)
{
  using namespace blt;
  using namespace gpio;
  using namespace chrono::literals;

  using led_red    = gpio::pin_out<PC, 6>;
  using led_blue   = gpio::pin_out<PC, 7>;
  using led_orange = gpio::pin_out<PC, 8>;
  using led_green  = gpio::pin_out<PC, 9>;
  using leds       = gpio::pin_out<PC, 6, 8, 9, 7>;

  using btn_pair = gpio::pin_in<PA, 0>;

  using ce  = gpio::pin_out<PB, 1>;
  using csn = gpio::pin_out<PB, 2>;

  g_hspi = hspi;
  g_htim = htim;

  HAL_TIM_Base_Start(g_htim);
  chrono::init([]() noexcept {
    return chrono::clock::time_point{chrono::clock::duration{__HAL_TIM_GET_COUNTER(g_htim)}};
  });
  leds::clear();

  chrono::delay(10ms);

  nrf24::device<spi::device<fhSpi>, csn, ce> radio;

  led_green::set();

  if (!radio.init()) {
    error_handler();
  }

  if (!radio.test()) {
    error_handler();
  }

  radio.setChannel(40);
  radio.setRfPowerLevel<nrf24::Value::RfPower0dBm>();
  radio.setDataRate<nrf24::Value::RfDatarate250kbps>();
  // radio.setAutoAck(true);
  // radio.setupAutoRetransmit<nrf24::Value::AutoRetransmitDelay500us, 15>();
  radio.setAddressWidth(5);
  radio.setPayloadSize(blt::message::motion::size);
  radio.openWritingPipe(nrf24_addresses[0]);
  radio.openReadingPipe(1, nrf24_addresses[1]);

  radio.startListening();
  radio.powerUp();

  led_green::clear();

  while (true) {
    while (!radio.available())
      ;
    led_orange::set();
    radio.read(rx_buffer, blt::message::motion::size);
    USBD_HID_SendReport(&hUsbDeviceFS, rx_buffer, blt::message::motion::size);
    led_orange::clear();
  }

  error_handler();
}
