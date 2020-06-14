#include "sdbg.hh"

#include <array>

namespace sdbg {

static UART_HandleTypeDef* sdbg_uart_h = nullptr;

void init(UART_HandleTypeDef* huart) {
  sdbg_uart_h = huart;
}

void log(uint8_t* data) {
  HAL_UART_Transmit(sdbg_uart_h, data, sizeof(data), HAL_UART_TIMEOUT_VALUE);
}

void log(std::string_view sv) {
  std::string str{sv};
  HAL_UART_Transmit(sdbg_uart_h, reinterpret_cast<uint8_t*>(str.data()), str.size(),
                    HAL_UART_TIMEOUT_VALUE);
}

}  // namespace sdbg