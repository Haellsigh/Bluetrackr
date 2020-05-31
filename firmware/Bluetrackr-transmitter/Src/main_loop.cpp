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

// Buffer to store a payload of maximum width
uint8_t nRF24_payload[32];
// Pipe number
nRF24_RXResult pipe;
// Length of received payload
uint8_t payload_length;

// Result of packet transmission
typedef enum {
  nRF24_TX_ERROR = (uint8_t)0x00,  // Unknown error
  nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
  nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
  nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t* pBuf, uint8_t length) {
  volatile uint32_t wait = 10;
  uint8_t           status;

  // Deassert the CE pin (in case if it still high)
  ce::clear();

  // Transfer a data from the specified buffer to the TX FIFO
  nRF24_WritePayload(pBuf, length);

  // Start a transmission by asserting CE pin (must be held at least 10us)
  ce::set();

  // Poll the transceiver status register until one of the following flags will be set:
  //   TX_DS  - means the packet has been transmitted
  //   MAX_RT - means the maximum number of TX retransmits happened
  // note: this solution is far from perfect, better to use IRQ instead of polling the
  // status
  do {
    status = nRF24_GetStatus();
    if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
      break;
    }
    delay::us(100);
  } while (wait--);

  // Deassert the CE pin (Standby-II --> Standby-I)
  ce::clear();

  if (!wait) {
    // Timeout
    return nRF24_TX_TIMEOUT;
  }

  // Clear pending IRQ flags
  nRF24_ClearIRQFlags();

  if (status & nRF24_FLAG_MAX_RT) {
    // Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
    return nRF24_TX_MAXRT;
  }

  if (status & nRF24_FLAG_TX_DS) {
    // Successful transmission
    return nRF24_TX_SUCCESS;
  }

  // Some banana happens, a payload remains in the TX FIFO, flush it
  nRF24_FlushTX();

  return nRF24_TX_ERROR;
}

void main_loop(ADC_HandleTypeDef* hadc, SPI_HandleTypeDef* hspi) {
  delay::init();
  leds::clear();

  /// Begin radio init
  led_status::set();

  // RX/TX disabled
  ce::clear();
  // cs_mpu::clear();

  nRF24_LL_INIT(hspi);
  delay::ms(50);
  if (!nRF24_Check()) {
    blt::error_handler();
  }
  nRF24_Init();

  // This is simple transmitter with Enhanced ShockBurst (to one logic address):
  //   - TX address: 'ESB'
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
  // Configure TX PIPE
  static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR);  // program TX address
  nRF24_SetAddr(
      nRF24_PIPE0,
      nRF24_ADDR);  // program address for pipe#0, must be same as TX (for Auto-ACK)
  // Set TX power (maximum)
  nRF24_SetTXPower(nRF24_TXPWR_0dBm);
  // Configure auto retransmit: 10 retransmissions with pause of 2500s in between
  nRF24_SetAutoRetr(nRF24_ARD_2500us, 10);
  // Enable Auto-ACK for pipe#0 (for ACK packets)
  nRF24_EnableAA(nRF24_PIPE0);
  // Set operational mode (PTX == transmitter)
  nRF24_SetOperationalMode(nRF24_MODE_TX);
  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();
  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  /// End radio init
  led_status::clear();
  delay::ms(500);
  led_status::set();
  delay::ms(500);
  led_status::clear();

  // Some variables
  uint32_t packets_lost = 0;  // global counter of lost packets
  uint8_t  otx;
  uint8_t  otx_plos_cnt;  // lost packet count
  uint8_t  otx_arc_cnt;   // retransmit count

  payload_length = 10;
  uint32_t i, j = 0;

  while (true) {
    // Prepare data packet
    for (i = 0; i < payload_length; i++) {
      nRF24_payload[i] = j++;
      if (j > 0x000000FF)
        j = 0;
    }

    // Transmit a packet
    tx_res       = nRF24_TransmitPacket(nRF24_payload, payload_length);
    otx          = nRF24_GetRetransmitCounters();
    otx_plos_cnt = (otx & nRF24_MASK_PLOS_CNT) >> 4;  // packets lost counter
    otx_arc_cnt  = (otx & nRF24_MASK_ARC_CNT);        // auto retransmissions counter
    switch (tx_res) {
      case nRF24_TX_SUCCESS:
        led_status::set();
        delay::ms(500);
        led_status::clear();
        break;
      case nRF24_TX_TIMEOUT:
        led_power::set();
        delay::ms(500);
        led_power::clear();
        break;
      case nRF24_TX_MAXRT:
        led_power::set();
        delay::ms(500);
        led_power::clear();
        packets_lost += otx_plos_cnt;
        nRF24_ResetPLOS();
        break;
      default:
        led_power::set();
        delay::ms(500);
        led_power::clear();
        break;
    }
    leds::clear();

    delay::ms(500);
  }

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
