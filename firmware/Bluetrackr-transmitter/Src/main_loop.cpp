#include "main_loop.hh"
#include "main.h"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <error_handler.hh>
//#include <spi/spi.hh>

extern "C" {
#include <nrf24/nrf24.h>
}

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
  using cs_mpu = gpio::pin_out<PB, MPU_CS_Pin>;
  using csn    = gpio::pin_out<PB, NRF_CSN_Pin>;
  using ce     = gpio::pin_out<PB, NRF_CE_Pin>;

  delay::init();
  leds::clear();

  /// Begin radio init
  led_status::set();

  // RX/TX disabled
  ce::clear();
  cs_mpu::clear();

  nRF24_LL_INIT(hspi);
  delay::ms(50);
  if (!nRF24_Check()) {
    blt::error_handler();
  }

  while (true) {
  }

  nRF24_Init();
  led_status::clear();

  // Initialises the radio
  // nrf24::device<spi::device, csn, ce> radio(hspi);

  /*
  led_status::set();
  if (!radio.init())
    blt::error_handler();
  radio.setAutoAck(true);
  // radio.setAutoAck(true);
  // radio.enableAckPayload();
  radio.setAutoRetransmit(nrf24::AutoRetransmitDelay::k3000us, 10);
  // Single byte payloads to test the speed
  // radio.setPayloadSize(1);
  radio.setPALevel(nrf24::PowerAmplifier::kMinimum);

  radio.openWritingPipe(0x544D52687C);
  // radio.openReadingPipe(1, 0xABCDABCD71);

  radio.stopListening();
  led_status::clear();

  while (true) {
    if (btn_pair::read()) {
      while (btn_pair::read())
        ;
      led_status::set();
      const uint8_t data[] = "11112222333344445555666677778888";
      if (!radio.write(data, 32, false)) {
        led_power::set();
        delay::ms(500);
        led_power::clear();
      }
      led_status::clear();
    }
  }
  */
}
