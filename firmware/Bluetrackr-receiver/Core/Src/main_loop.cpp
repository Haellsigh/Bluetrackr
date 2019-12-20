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

void main_loop(SPI_HandleTypeDef* hspi) {
  // Outputs
  using led_orange = gpio::pin_out<PC, p8>;
  using led_red    = gpio::pin_out<PC, p6>;
  using led_green  = gpio::pin_out<PC, p9>;
  using led_blue   = gpio::pin_out<PC, p7>;
  using leds       = gpio::pin_out<PC, p6, p7, p8, p9>;

  // Inputs
  // using btn_pair = gpio::pin_in<PortA, pin0>;

  // nRF24L01 pins
  using csn = gpio::pin_out<PB, NRF_CSN_Pin>;
  using ce  = gpio::pin_out<PB, NRF_CE_Pin>;

  delay::init();
  leds::clear();

  /*{
    uint8_t data[] = "Step 2\n";
    uint16_t len = sizeof(data) / sizeof(uint8_t);
    CDC_Transmit_FS(data, len);
  }*/

  // Initialises the radio
  nrf24::device<spi::device, csn, ce> radio(hspi);

  led_blue::set();
  if (!radio.init())
    blt::error_handler();
  // radio.setAutoAck(true);
  // radio.enableAckPayload();
  radio.setAutoRetransmit(nrf24::AutoRetransmitDelay::k4000us, 15);
  // Single byte payloads to test the speed
  // radio.setPayloadSize(1);
  radio.setPALevel(nrf24::PowerAmplifier::kMinimum);

  radio.openWritingPipe(0xABCDABCD71);
  radio.openReadingPipe(0, 0x544D52687C);

  radio.stopListening();
  led_blue::clear();

  /*MPU9250_CONFIG_t cfg;
  cfg.ACCEL_SCALE = ACCEL_SCALE_16G;
  cfg.GYRO_SCALE = GYRO_SCALE_2000dps;
  cfg.GPIOx = GPIOB;
  cfg.GPIO_PIN = MPU_CS_Pin;
  cfg.hspi = hspi;*/

  // HAL_GPIO_WritePin(useled_GPIO_Port, useled_Pin, GPIO_PIN_SET);

  /*if (MPU9250_Config(&cfg) != MPU9250_RESULT_OK) {
    while (1) {
      HAL_GPIO_TogglePin(useled_GPIO_Port, useled_Pin);
      HAL_Delay(400);
    }
  }
  if (MPU9250_Initialize(&cfg) != MPU9250_RESULT_OK) {
    while (1) {
      HAL_GPIO_TogglePin(useled_GPIO_Port, useled_Pin);
      HAL_Delay(50);
    }
  }
  if (MPU9250_Calibrate(&cfg) != MPU9250_RESULT_OK) {
    while (1) {
      HAL_GPIO_TogglePin(useled_GPIO_Port, useled_Pin);
      HAL_Delay(1000);
    }
  }*/

  // MPU9250_DATA_t data;
  // int8_t datastr[30] = "";

  while (true) {
    delay::ms(1000);
    led_blue::set();
    const uint8_t data[] = "11112222333344445555666677778888";
    if (!radio.write(data, 32, true)) {
      led_red::set();
      delay::ms(500);
      led_red::clear();
    }
    led_blue::clear();

    // MPU9250_Update7DOF(&cfg, &data);
    /*while (!radio.available())
      ;
    leds::set();
    delay::ms(1000);
    leds::clear();*/
  }
}
