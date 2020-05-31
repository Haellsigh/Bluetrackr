#include "nrf24_hal.h"

SPI_HandleTypeDef* nRF24_hspi;

void nRF24_LL_INIT(SPI_HandleTypeDef* hspi) {
  nRF24_hspi = hspi;
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
  uint8_t result;
  // Default timeout is 2s
  while (nRF24_hspi->State == HAL_SPI_STATE_BUSY)
    ;
  if (HAL_SPI_TransmitReceive(nRF24_hspi, &data, &result, 1, 2000) != HAL_OK) {
    return 0;
  }

  return result;
}
