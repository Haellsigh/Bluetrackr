#include "sdbg.hh"

namespace sdbg {

static UART_HandleTypeDef* uart_h = nullptr;

void init(UART_HandleTypeDef* huart) {
  uart_h = huart;
}

void log(uint8_t* data, uint16_t len) {
  HAL_UART_Transmit(uart_h, data, len, HAL_UART_TIMEOUT_VALUE);
}

}  // namespace sdbg