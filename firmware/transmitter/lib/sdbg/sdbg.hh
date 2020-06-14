#pragma once

#include "stm32f3xx_hal.h"

namespace sdbg {

void init(UART_HandleTypeDef* huart);
void log(uint8_t* data, uint16_t len);

}  // namespace sdbg
