#pragma once

#include <blt/hal_include.hh>
#include <blt/utils.hh>

namespace blt::spi {

template <SPI_HandleTypeDef* fhSpi()>
struct device : public utils::noncopyable {
  static uint8_t rw(uint8_t data)
  {
    uint8_t result;
    // Default timeout is 2s
    if (HAL_SPI_TransmitReceive(fhSpi(), &data, &result, 1, 2000) != HAL_OK) {
      return 0;
    }

    return result;
  }
};

}  // namespace blt::spi