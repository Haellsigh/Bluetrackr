#include "main_loop.h"
#include "main.h"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <error_handler.hh>
#include <nrf24/nrf24.hh>
#include <spi/spi.hh>

//#include "mpu.h"

using namespace blt;
using namespace gpio;

void main_loop(ADC_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi) {
  // Outputs
  using led_status = gpio::pin_out<PA, p2>;
  using led_power  = gpio::pin_out<PA, p3>;
  using leds       = gpio::pin_out<PA, p2, p3>;

  // Inputs
  using btn_pair = gpio::pin_in<PB, p8>;

  // nRF24L01 pins
  using csn = gpio::pin_out<PB, NRF_CSN_Pin>;
  using ce  = gpio::pin_out<PB, NRF_CE_Pin>;

  delay::init();

  // Initialises the radio
  nrf24::device<spi::device, csn, ce> radio(hspi);

  led_status::set();
  if (!radio.init())
    blt::error_handler();
  // radio.setAutoAck(true);
  // radio.enableAckPayload();
  radio.setAutoRetransmit(nrf24::AutoRetransmitDelay::k4000us, 15);
  // Single byte payloads to test the speed
  // radio.setPayloadSize(32);
  radio.setPALevel(nrf24::PowerAmplifier::kMinimum);

  radio.openWritingPipe(0x544d52687C);
  radio.openReadingPipe(1, 0xABCDABCD71);

  radio.stopListening();
  led_status::clear();

  while (true) {
    if (btn_pair::read()) {
      while (btn_pair::read())
        ;
      const uint8_t data[] = "11112222333344445555666677778888";
      led_status::set();
      if (!radio.write(data, 32, false)) {
        led_status::clear();
        led_power::set();
        delay::ms(500);
        led_power::clear();
      }
      led_status::clear();
    }
  }
}
