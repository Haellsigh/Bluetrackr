#pragma once

#include "stm32f3xx_hal.h"

#include <string_view>

namespace sdbg {

void init(UART_HandleTypeDef* huart);
void log(uint8_t* data);
void log(std::string_view sv);

}  // namespace sdbg
