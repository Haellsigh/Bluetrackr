#include "main_loop.h"
#include "main.h"

#include <blt/delay.hpp>
#include <blt/error_handler.hpp>
#include <blt/gpio.hpp>
#include <nrf24/nrf24.hpp>
#include <spi/spi.hpp>

//#include "mpu.h"

using namespace blt;
using namespace gpio;

void main_loop(ADC_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi) {
  // Initialises delay library
  delay::init();

  // Outputs
  using led_status = gpio::pin_out<PortA, pin2>;
  using led_power = gpio::pin_out<PortA, pin3>;
  using leds = gpio::pin_out<PortA, pin2, pin3>;

  // Inputs
  using btn_pair = gpio::pin_in<PortB, pin8>;

  // nRF24L01 pins
  using csn = gpio::pin_out<PortB, NRF_CSN_Pin>;
  using ce = gpio::pin_out<PortB, NRF_CE_Pin>;

  // Reset leds
  leds::off();

  blt::delay::ms(50);

  // Initialises the radio
  nrf24::device<spi::device, csn, ce> radio(hspi);
  radio.init();
  // delay::ms(500);

  if (!radio.setDataRate(nrf24::DataRate::k2Mbps)) {
    blt::error_handler();
  }

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

  led_status::on();

  while (true) {
    // MPU9250_Update7DOF(&cfg, &data);
  }
}
