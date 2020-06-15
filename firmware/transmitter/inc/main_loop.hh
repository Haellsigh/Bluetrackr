#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void main_loop(UART_HandleTypeDef* huart, SPI_HandleTypeDef* hspi);

#ifdef __cplusplus
}
#endif