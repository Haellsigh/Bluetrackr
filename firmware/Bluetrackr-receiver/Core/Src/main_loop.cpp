#include "main_loop.h"
#include "main.h"

#include <blt/gpio.hh>
#include <blt/time.hh>
#include <error_handler.hh>
//#include <nrf24_custom/nrf24.hh>
//#include <spi/spi.hh>

extern "C" {
#include <nrf24/nrf24.h>
}

//#include "mpu.h"

using namespace blt;
using namespace gpio;

// Buffer to store a payload of maximum width
uint8_t nRF24_payload[32];
// Pipe number
nRF24_RXResult pipe;
// Length of received payload
uint8_t payload_length;

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
  // nrf24::device<spi::device, csn, ce> radio(hspi);

  /// Begin radio init
  led_blue::set();

  // RX/TX disabled
  ce::clear();

  nRF24_LL_INIT(hspi);
  delay::ms(50);
  if (!nRF24_Check()) {
    blt::error_handler();
  }
  nRF24_Init();

  // This is simple receiver with Enhanced ShockBurst:
  //   - RX address: 'ESB'
  //   - payload: 10 bytes
  //   - RF channel: 40 (2440MHz)
  //   - data rate: 2Mbps
  //   - CRC scheme: 2 byte

  // The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK
  // (ShockBurst enabled)

  // Set RF channel
  nRF24_SetRFChannel(40);
  // Set data rate
  nRF24_SetDataRate(nRF24_DR_2Mbps);
  // Set CRC scheme
  nRF24_SetCRCScheme(nRF24_CRC_2byte);
  // Set address width, its common for all pipes (RX and TX)
  nRF24_SetAddrWidth(3);
  // Configure RX PIPE
  static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
  nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR);  // program address for pipe
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON,
                  10);  // Auto-ACK: enabled, payload length: 10 bytes
  // Set TX power for Auto-ACK (maximum, to ensure that transmitter will hear ACK reply)
  nRF24_SetTXPower(nRF24_TXPWR_0dBm);
  // Set operational mode (PRX == receiver)
  nRF24_SetOperationalMode(nRF24_MODE_RX);
  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();
  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);
  // Put the transceiver to the RX mode
  ce::set();

  /// End radio init
  led_blue::clear();
  delay::ms(500);
  led_blue::set();
  delay::ms(500);
  led_blue::clear();

  /*if (!radio.init())
    blt::error_handler();
  // radio.setAutoAck(true);
  // radio.enableAckPayload();
  radio.setAutoRetransmit(nrf24::AutoRetransmitDelay::k4000us, 15);
  // Single byte payloads to test the speed
  // radio.setPayloadSize(1);
  radio.setPALevel(nrf24::PowerAmplifier::kMinimum);

  radio.openWritingPipe(0xABCDABCD71);
  radio.openReadingPipe(0, 0x544D52687C);

  radio.stopListening();*/

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
    if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
      pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
      // Clear all pending IRQ flags
      nRF24_ClearIRQFlags();

      led_green::set();
      delay::ms(500);
      led_green::clear();
    }
  }

  /*while (true) {
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
    while (!radio.available())
      ;
    leds::set();
    delay::ms(1000);
    leds::clear();
  }*/
}
