#ifndef BLT_LIBS_SPI_H_
#define BLT_LIBS_SPI_H_

#include <main.h>

#include <blt/utils.hpp>

namespace blt {

namespace spi {

/**
 * 'Automatically' generates functions to get the SPI Handles
 */
namespace peripheral {
#define SPI_HANDLE_FUNCTION(_NAME) \
  constexpr SPI_HandleTypeDef* spi##_NAME() { return _NAME; }
}  // namespace peripheral

class device : public utils::noncopyable {
 public:
  static uint8_t rw(SPI_HandleTypeDef* hspi, uint8_t data) {
    uint8_t result;
    // Default timeout is 2s
    if (HAL_SPI_TransmitReceive(hspi, &data, &result, 1, 2000) != HAL_OK) {
      Error_Handler();
    }

    return result;
  }
};

}  // namespace spi

}  // namespace blt

#endif  // BLT_LIBS_SPI_H_
