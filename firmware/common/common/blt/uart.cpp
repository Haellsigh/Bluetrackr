#include <blt/uart.hh>

#include <blt/hal_include.hh>

#include <string>

namespace blt::uart {

UART_HandleTypeDef* uart_handle = nullptr;

void init(UART_HandleTypeDef* huart) {
  uart_handle = huart;
}

void transmit(uint8_t data) {
  HAL_UART_Transmit(uart_handle, &data, 1, HAL_UART_TIMEOUT_VALUE);
}

void transmit(uint8_t* data, uint16_t len) {
  HAL_UART_Transmit(uart_handle, data, len, HAL_UART_TIMEOUT_VALUE);
}

void transmit(std::string_view sv) {
  std::string str{sv};
  transmit(reinterpret_cast<uint8_t*>(str.data()), str.size());
}

}  // namespace blt::uart